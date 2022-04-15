#include "OakRos.hpp"

#include <chrono>

#include <cv_bridge/cv_bridge.h>

void OakRos::init(const ros::NodeHandle &nh, const OakRosParams &params) {

    m_params = params;
    // ROS-related
    m_nh = nh;

    lastPublishedSeq = 0;
    lastSeq = 0;
    lastGyroTs = -1;
    m_stereo_seq_throttle = 1;

    spdlog::info("initialising device {}", m_params.device_id);

    m_stereo_is_rectified = false;

    if (m_params.stereo_fps_throttle) {
        m_stereo_seq_throttle = *(m_params.stereo_fps_throttle);
        spdlog::info("{} user code throttling images to publish every {} frames",
                     m_params.device_id, m_stereo_seq_throttle);
    }

    if (m_params.rates_workaround)
        configureRatesWorkaround();

    configureStereoRgbCamD();

    // alway try to see if IMU stream is there
    if (m_params.enable_imu)
        configureImu();

    auto xinControl = configureControl();

    // m_pipeline.setXLinkChunkSize(0);

    // Initialise device
    {
        dai::UsbSpeed usbSpeedMax;

        if (m_params.only_usb2_mode)
            usbSpeedMax = dai::UsbSpeed::HIGH;
        else
            usbSpeedMax = dai::UsbSpeed::SUPER;

        if (m_params.device_id.empty()) {
            spdlog::info("Creating device without specific id");
            m_device = std::make_shared<dai::Device>(m_pipeline, usbSpeedMax);
        } else {
            spdlog::info("Creating device with specific id {}", m_params.device_id);
            m_device = std::make_shared<dai::Device>(m_pipeline, getDeviceInfo(m_params.device_id),
                                                     usbSpeedMax);
        }

        dai::UsbSpeed usbSpeed = m_device->getUsbSpeed();

        if (usbSpeed == dai::UsbSpeed::UNKNOWN) {
            spdlog::info("{} device created with Ethernet link", m_params.device_id);
        } else if (usbSpeed != usbSpeedMax) {
            spdlog::warn("{} device created with speed {}, not {}, quitting", m_params.device_id,
                         usbSpeed, usbSpeedMax);
            throw std::runtime_error("USB SPEED NOT CORRECT");
        } else
            spdlog::info("{} device created with speed {}", m_params.device_id, usbSpeed);
    }

    // setup lights

    {
        if (m_params.ir_laser_dot > 0) {
            spdlog::info("Enable IR laser dot projection with {} mA", m_params.ir_laser_dot);
            m_device->setIrLaserDotProjectorBrightness(m_params.ir_laser_dot);
        }

        if (m_params.ir_floodlight > 0) {
            spdlog::info("Enable IR floodlight with {} mA", m_params.ir_floodlight);
            m_device->setIrFloodLightBrightness(m_params.ir_floodlight);
        }
    }

    // setup queues
    m_imageTransport = std::make_shared<image_transport::ImageTransport>(m_nh);
    if (m_params.enable_stereo)
        setupStereoQueue();

    if (m_params.enable_rgb)
        setupRgbQueue();

    if (m_params.enable_camD)
        setupCamDQueue();

    if (m_params.enable_depth) {
        m_depthQueue = m_device->getOutputQueue("depth", 2, false);
    }

    if (m_params.enable_imu) {
        m_imuQueue = m_device->getOutputQueue("imu", 50, false);
    }

    setupControlQueue(xinControl);

    m_run = std::thread(&OakRos::run, this);
}

void OakRos::configureRatesWorkaround() {
    spdlog::info("{} using rates workaround to cut framerate to half", m_params.device_id);

    if (m_params.enable_stereo || m_params.enable_depth) {
        m_scriptLeft = m_pipeline.create<dai::node::Script>();
        m_scriptRight = m_pipeline.create<dai::node::Script>();
        // Half the rates of all left and right images
        m_scriptLeft->setScript(R"(
            while True:
                frame = node.io['frameLeft'].get()
                if frame.getSequenceNum() % 3 == 1:
                    node.io['streamLeft'].send(frame)
            )");

        m_scriptRight->setScript(R"(
            while True:
                frame = node.io['frameRight'].get()
                if frame.getSequenceNum() % 3 == 1:
                    node.io['streamRight'].send(frame)
            )");
    }

    if (m_params.enable_rgb) {
        m_scriptRgb = m_pipeline.create<dai::node::Script>();
        m_scriptRgb->setScript(R"(
            while True:
                frame = node.io['frameRgb'].get()
                if frame.getSequenceNum() % 3 == 1:
                    node.io['streamRgb'].send(frame)
            )");
    }

    if (m_params.enable_camD) {
        m_scriptCamD = m_pipeline.create<dai::node::Script>();
        m_scriptCamD->setScript(R"(
            while True:
                frame = node.io['frameCamD'].get()
                if frame.getSequenceNum() % 3 == 1:
                    node.io['streamCamD'].send(frame)
            )");
    }
}

std::shared_ptr<dai::node::XLinkIn> OakRos::configureControl() {
    auto xinControl = m_pipeline.create<dai::node::XLinkIn>();
    xinControl->setStreamName("control");

    // Linking

    if (m_params.enable_stereo || m_params.enable_depth) {
        xinControl->out.link(m_monoLeft->inputControl);
        xinControl->out.link(m_monoRight->inputControl);
    }

    if (m_params.enable_rgb)
        xinControl->out.link(m_rgb->inputControl);

    if (m_params.enable_camD)
        xinControl->out.link(m_camD->inputControl);

    return xinControl;
}

void OakRos::setupControlQueue(std::shared_ptr<dai::node::XLinkIn> xinControl) {

    // camera exposure and ISO (gain) control
    m_controlQueue = m_device->getInputQueue(xinControl->getStreamName());

    // set camera configs
    dai::CameraControl ctrl;
    if (m_params.manual_exposure && m_params.manual_iso) {
        const int exposure = m_params.manual_exposure.value();
        const int iso = m_params.manual_iso.value();
        ctrl.setManualExposure(exposure, iso);

        spdlog::info("{} Enable manual exposure = {} and iso = {}", m_params.device_id, exposure,
                     iso);
    } else {
        spdlog::info("{} Enable auto exposure with compesnation {}", m_params.device_id,
                     m_params.exposure_compensation);
        ctrl.setAutoExposureEnable();
        ctrl.setAutoExposureCompensation(m_params.exposure_compensation);
    }

    m_controlQueue->send(ctrl);
}

void OakRos::configureImu() {
    auto imu = m_pipeline.create<dai::node::IMU>();
    auto xoutIMU = m_pipeline.create<dai::node::XLinkOut>();
    xoutIMU->setStreamName("imu");

    // enable ACCELEROMETER_RAW and GYROSCOPE_RAW
    // TODO: seems ACCELEROMETER_RAW has orientation X-Y wrong
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW},
                         m_params.imu_frequency);
    // imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER, dai::IMUSensor::GYROSCOPE_CALIBRATED},
    //  m_params.imu_frequency);

    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB
    // bandwidth is available
    imu->setBatchReportThreshold(5);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until
    // host can receive it if lower or equal to batchReportThreshold then the sending is always
    // blocking on device useful to reduce device's CPU load  and number of lost packets, if CPU
    // load is high on device side due to multiple nodes
    imu->setMaxBatchReports(20);

    // Link plugins IMU -> XLINK
    imu->out.link(xoutIMU->input);
}

void OakRos::configureStereoRgbCamD() {
    std::shared_ptr<dai::node::XLinkOut> xoutLeft, xoutRight, xoutRgb, xoutCamD;
    xoutLeft = m_pipeline.create<dai::node::XLinkOut>();
    xoutRight = m_pipeline.create<dai::node::XLinkOut>();
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // configure the stereo sensors' format
    std::shared_ptr<dai::node::StereoDepth> stereoDepth;
    m_monoLeft = m_pipeline.create<dai::node::MonoCamera>();
    m_monoRight = m_pipeline.create<dai::node::MonoCamera>();

    if (m_params.enable_rgb) {
        xoutRgb = m_pipeline.create<dai::node::XLinkOut>();
        xoutRgb->setStreamName("rgb");
        spdlog::info("Enable RGB socket camera...");
        m_rgb = m_pipeline.create<dai::node::MonoCamera>();
        if (m_params.stereo_resolution)
            m_rgb->setResolution(m_params.stereo_resolution.value());

        m_rgb->setBoardSocket(dai::CameraBoardSocket::RGB);

        if (m_params.stereo_fps) {
            m_rgb->setFps(m_params.stereo_fps.value());
        }

        if (m_params.rates_workaround) {
            m_rgb->out.link(m_scriptRgb->inputs["frameRgb"]);
            m_scriptRgb->outputs["streamRgb"].link(xoutRgb->input);
        } else {
            m_rgb->out.link(xoutRgb->input);
        }
    }

    if (m_params.enable_camD) {
        xoutCamD = m_pipeline.create<dai::node::XLinkOut>();
        xoutCamD->setStreamName("camd");
        spdlog::info("Enable CAM_D socket camera...");
        m_camD = m_pipeline.create<dai::node::MonoCamera>();
        if (m_params.stereo_resolution)
            m_camD->setResolution(m_params.stereo_resolution.value());

        m_camD->setBoardSocket(dai::CameraBoardSocket::CAM_D);

        if (m_params.stereo_fps) {
            m_camD->setFps(m_params.stereo_fps.value());
        }

        if (m_params.rates_workaround) {
            m_camD->out.link(m_scriptCamD->inputs["frameCamD"]);
            m_scriptCamD->outputs["streamCamD"].link(xoutCamD->input);
        } else {
            m_camD->out.link(xoutCamD->input);
        }
    }

    if (m_params.enable_stereo || m_params.enable_depth) {

        if (m_params.stereo_resolution) {
            m_monoLeft->setResolution(m_params.stereo_resolution.value());
            m_monoRight->setResolution(m_params.stereo_resolution.value());
        }

        m_monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        m_monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        if (m_params.stereo_fps) {
            spdlog::info("{} sets stereo fps to {}", m_params.device_id,
                         m_params.stereo_fps.value());
            m_monoLeft->setFps(m_params.stereo_fps.value());
            m_monoRight->setFps(m_params.stereo_fps.value());
        }

        // case where no depth node is required
        if (!m_params.enable_stereo_rectified && m_params.enable_stereo && !m_params.enable_depth) {
            spdlog::info("{} enabling both only raw stereo...", m_params.device_id);

            if (m_params.rates_workaround) {
                m_monoLeft->out.link(m_scriptLeft->inputs["frameLeft"]);
                m_scriptLeft->outputs["streamLeft"].link(xoutLeft->input);

                m_monoRight->out.link(m_scriptRight->inputs["frameRight"]);
                m_scriptRight->outputs["streamRight"].link(xoutRight->input);
            } else {
                m_monoLeft->out.link(xoutLeft->input);
                m_monoRight->out.link(xoutRight->input);
            }

        }
        // case where depth node is required
        else if (m_params.enable_depth || m_params.enable_stereo_rectified) {
            stereoDepth = m_pipeline.create<dai::node::StereoDepth>();
            stereoDepth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
            stereoDepth->setRectifyEdgeFillColor(0); // black, to better see the cutout
            // stereoDepth->setInputResolution(1280, 720);
            stereoDepth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
            stereoDepth->setLeftRightCheck(false);
            stereoDepth->setExtendedDisparity(false);
            stereoDepth->setSubpixel(false);

            // Load Mesh Data
            if (m_params.enable_mesh_dir.empty())
                spdlog::warn("No mesh file directory specified, skip mesh overriding");
            else {
                auto leftPath = m_params.enable_mesh_dir + "/left_mesh.calib";
                auto rightPath = m_params.enable_mesh_dir + "/right_mesh.calib";
                spdlog::warn("Load mesh files at \n - {}\n - {}", leftPath, rightPath);
                stereoDepth->loadMeshFiles(leftPath, rightPath);
            }

            // Linking
            if (m_params.rates_workaround) {
                m_monoLeft->out.link(m_scriptLeft->inputs["frameLeft"]);
                m_scriptLeft->outputs["streamLeft"].link(stereoDepth->left);

                m_monoRight->out.link(m_scriptRight->inputs["frameRight"]);
                m_scriptRight->outputs["streamRight"].link(stereoDepth->right);
            } else {
                m_monoLeft->out.link(stereoDepth->left);
                m_monoRight->out.link(stereoDepth->right);
            }

            // depth output stream
            if (m_params.enable_depth) {
                spdlog::info("{} enabling depth streams...", m_params.device_id);
                auto xoutDepth = m_pipeline.create<dai::node::XLinkOut>();
                xoutDepth->setStreamName("depth");
                stereoDepth->depth.link(xoutDepth->input);
            }

            // stereo rectified will need depth node
            if (m_params.enable_stereo) {

                if (!m_params.enable_stereo_rectified) {
                    spdlog::info("{} enabling raw stereo streams...", m_params.device_id);
                    // output raw images
                    stereoDepth->syncedLeft.link(xoutLeft->input);
                    stereoDepth->syncedRight.link(xoutRight->input);
                } else {
                    spdlog::info("{} enabling rectified stereo streams...", m_params.device_id);
                    // output rectified images
                    stereoDepth->rectifiedLeft.link(xoutLeft->input);
                    stereoDepth->rectifiedRight.link(xoutRight->input);
                    m_stereo_is_rectified = true;
                }
            }
        }
    }

    if (m_params.hardware_sync) {
        m_monoLeft->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
        m_monoRight->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OUTPUT);

        if (m_params.enable_rgb)
            m_rgb->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
        if (m_params.enable_camD)
            m_camD->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
    }
}

void OakRos::setupStereoQueue() {
    m_leftQueue = m_device->getOutputQueue("left", 10, false);
    m_rightQueue = m_device->getOutputQueue("right", 10, false);

    spdlog::info("{} advertising stereo cameras in ros topics...", m_params.device_id);

    m_leftPub.reset(new auto(
        m_imageTransport->advertiseCamera(m_params.topic_name + "/left/image_rect_raw", 3)));
    m_rightPub.reset(new auto(
        m_imageTransport->advertiseCamera(m_params.topic_name + "/right/image_rect_raw", 3)));
}

void OakRos::setupRgbQueue() {
    m_rgbQueue = m_device->getOutputQueue("rgb", 10, false);

    spdlog::info("{} advertising rgb cameras in ros topic...", m_params.device_id);
    m_rgbPub.reset(new auto(
        m_imageTransport->advertiseCamera(m_params.topic_name + "/rgb/image_rect_raw", 3)));
}

void OakRos::setupCamDQueue() {
    m_camDQueue = m_device->getOutputQueue("camd", 10, false);

    spdlog::info("{} advertising camd cameras in ros topic...", m_params.device_id);
    m_camDPub.reset(new auto(
        m_imageTransport->advertiseCamera(m_params.topic_name + "/camd/image_rect_raw", 3)));
}

void OakRos::run() {
    m_running = true;

    if (m_watchdog.joinable())
        m_watchdog.join();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    spdlog::info("{} OakRos running now", m_params.device_id);

    dai::DataOutputQueue::CallbackId depthCallbackId, imuCallbackId;

    if (m_depthQueue.get()) {
        spdlog::info("{} adds depth queue callback", m_params.device_id);
        depthCallbackId = m_depthQueue->addCallback(
            std::bind(&OakRos::depthCallback, this, std::placeholders::_1));
    }

    if (m_imuQueue.get()) {
        spdlog::info("{} adds imu queue callback", m_params.device_id);
        imuCallbackId =
            m_imuQueue->addCallback(std::bind(&OakRos::imuCallback, this, std::placeholders::_1));
    }

    cv::Mat leftCvFrame, rightCvFrame, rgbCvFrame, camDCvFrame;
    sensor_msgs::CameraInfo leftCameraInfo, rightCameraInfo, rgbCameraInfo, camDCameraInfo;

    try {

        bool runAllFourCams =
            m_leftQueue.get() && m_rightQueue.get() && m_rgbQueue.get() && m_camDQueue.get();

        if (runAllFourCams)
            spdlog::info("Streaming all four cameras");

        bool runStereo = !runAllFourCams && m_leftQueue.get() && m_rightQueue.get();

        if (runStereo)
            spdlog::info("Streaming stereo cameras only");

        long seqLeft, seqRight, seqRgb, seqCamD;
        double tsLeft = -1, tsRight = -1, tsRgb = -1, tsCamD = -1;
        double maxTs = 0;

        std::shared_ptr<dai::ImgFrame> left, right, rgb, camD;
        while (m_running) {
            // process stereo data
            constexpr double deltaT = 2e-3; // 2ms trigger delay
            if (runStereo || runAllFourCams) {

                try {
                    while (tsLeft < maxTs - deltaT) {
                        left = m_leftQueue->get<dai::ImgFrame>();
                        tsLeft = left->getTimestamp().time_since_epoch().count() / 1.0e9;
                    }

                    while (tsRight < maxTs - deltaT) {
                        right = m_rightQueue->get<dai::ImgFrame>();
                        tsRight = right->getTimestamp().time_since_epoch().count() / 1.0e9;
                    }

                    if (runAllFourCams) {
                        while (tsRgb < maxTs - deltaT) {
                            rgb = m_rgbQueue->get<dai::ImgFrame>();
                            tsRgb = rgb->getTimestamp().time_since_epoch().count() / 1.0e9;
                        }

                        while (tsCamD < maxTs - deltaT) {
                            camD = m_camDQueue->get<dai::ImgFrame>();
                            tsCamD = camD->getTimestamp().time_since_epoch().count() / 1.0e9;
                        }
                    }

                } catch (std::exception &e) {
                    spdlog::warn("get() for left or right image failed: {}", e.what());
                    m_watchdog = std::thread(&OakRos::restart, this);
                    m_running = false;
                    continue;
                }

                maxTs = std::max(tsLeft, tsRight);

                if (runAllFourCams)
                    maxTs = std::max(maxTs, std::max(tsRgb, tsCamD));

                // if any of the sequence does not match the latest one, try again
                bool retry = false;
                if (maxTs - tsLeft > deltaT) {
                    // seqLeft = -1;
                    retry = true;
                }
                if (maxTs - tsRight > deltaT) {
                    // seqRight = -1;
                    retry = true;
                }

                if (runAllFourCams && maxTs - tsRgb > deltaT) {
                    // seqRgb = -1;
                    retry = true;
                }
                if (runAllFourCams && maxTs - tsCamD > deltaT) {
                    // seqCamD = -1;
                    retry = true;
                }

                spdlog::info("left = {}, right = {}, rgb = {}, camd = {}, maxSeq = {}", tsLeft,
                             tsRight, tsRgb, tsCamD, maxTs);

                if (m_params.hardware_sync && retry)
                    continue;

                spdlog::info("synced on ts {} for all cameras", maxTs);
                maxTs += deltaT + deltaT; 

                seqLeft = left->getSequenceNum();
                seqRight = right->getSequenceNum();
                seqRgb = rgb->getSequenceNum();
                seqCamD = camD->getSequenceNum();

                // Here we have make sure the stereo have the same sequence number. According to
                // OAK, this ensures synchronisation

                // initialise camera_info msgs if needed
                if (leftCameraInfo.width == 0 || rightCameraInfo.width == 0) {
                    spdlog::info("{} Initialise camera_info messages, rectification = {}",
                                 m_params.device_id,
                                 m_stereo_is_rectified ? "Enabled" : "Disabled");

                    // if (!m_stereo_is_rectified) {
                    leftCameraInfo = getCameraInfo(left, dai::CameraBoardSocket::LEFT);
                    rightCameraInfo = getCameraInfo(right, dai::CameraBoardSocket::RIGHT);

                    if (runAllFourCams) {
                        rgbCameraInfo = getCameraInfo(rgb, dai::CameraBoardSocket::RGB);
                        camDCameraInfo = getCameraInfo(camD, dai::CameraBoardSocket::CAM_D);
                    }
                    // } else
                    //     leftCameraInfo = getCameraInfo(
                    //         right, dai::CameraBoardSocket::RIGHT); // after rectification in OAK,
                    //                                                // the intrinsics of left will
                    //                                                be
                    //                                                // the same as the right
                    //     rightCameraInfo = getCameraInfo(right, dai::CameraBoardSocket::RIGHT);
                    // }
                }

                int frameInterval;
                if (m_params.rates_workaround)
                    frameInterval = 3;
                else
                    frameInterval = 1;

                if (lastSeq && seqLeft - frameInterval != lastSeq)
                    spdlog::warn("jump detected in image frames, from {} to {}, should be to {}",
                                 lastSeq, seqLeft, lastSeq + frameInterval);
                lastSeq = seqLeft;

                if (lastPublishedSeq != 0 && (seqLeft - lastPublishedSeq < m_stereo_seq_throttle))
                    continue;

                lastPublishedSeq = seqLeft;

                spdlog::info("checking timestamp");

                spdlog::info("{} left seq = {}, ts = {}", m_params.device_id, seqLeft, tsLeft);
                spdlog::info("{} right seq = {}, ts = {}", m_params.device_id, seqRight, tsRight);

                if (runAllFourCams) {
                    spdlog::info("{} rgb seq = {}, ts = {}", m_params.device_id, seqRgb, tsRgb);
                    spdlog::info("{} camd seq = {}, ts = {}", m_params.device_id, seqCamD, tsCamD);
                }

                // publish left frame and camera info
                {
                    leftCvFrame = left->getFrame();

                    if (m_params.hardware_sync && m_params.align_ts_to_right)
                        leftCameraInfo.header.stamp = ros::Time().fromSec(tsRight);
                    else
                        leftCameraInfo.header.stamp = ros::Time().fromSec(tsLeft);
                    cv_bridge::CvImage leftBridge = cv_bridge::CvImage(
                        leftCameraInfo.header, sensor_msgs::image_encodings::MONO8, leftCvFrame);

                    m_leftPub->publish(*leftBridge.toImageMsg(), leftCameraInfo);
                }

                // publish right frame and camera info
                {
                    rightCvFrame = right->getFrame();

                    rightCameraInfo.header.stamp = ros::Time().fromSec(tsRight);
                    cv_bridge::CvImage rightBridge = cv_bridge::CvImage(
                        rightCameraInfo.header, sensor_msgs::image_encodings::MONO8, rightCvFrame);

                    m_rightPub->publish(*rightBridge.toImageMsg(), rightCameraInfo);
                }

                if (runAllFourCams) {
                    // publish rgb frame and camera info
                    {
                        rgbCvFrame = rgb->getFrame();
                        if (m_params.hardware_sync && m_params.align_ts_to_right)
                            rgbCameraInfo.header.stamp = ros::Time().fromSec(tsRight);
                        else
                            rgbCameraInfo.header.stamp = ros::Time().fromSec(tsRgb);
                        
                        cv_bridge::CvImage rgbBridge = cv_bridge::CvImage(
                            rgbCameraInfo.header, sensor_msgs::image_encodings::MONO8, rgbCvFrame);

                        m_rgbPub->publish(*rgbBridge.toImageMsg(), rgbCameraInfo);
                    }

                    // publish camd frame and camera info
                    {
                        camDCvFrame = camD->getFrame();
                        if (m_params.hardware_sync && m_params.align_ts_to_right)
                            camDCameraInfo.header.stamp = ros::Time().fromSec(tsRight);
                        else
                            camDCameraInfo.header.stamp = ros::Time().fromSec(tsCamD);

                        cv_bridge::CvImage camDBridge =
                            cv_bridge::CvImage(camDCameraInfo.header,
                                               sensor_msgs::image_encodings::MONO8, camDCvFrame);

                        m_camDPub->publish(*camDBridge.toImageMsg(), camDCameraInfo);
                    }
                }

                seqLeft = -1;
                seqRight = -1;
                seqRgb = -1;
                seqCamD = -1;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

            } else
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (std::exception &e) {
        spdlog::warn("while(m_running) failed: {}", e.what());
        m_watchdog = std::thread(&OakRos::restart, this);
    }

    if (m_depthQueue.get())
        m_depthQueue->removeCallback(depthCallbackId);

    if (m_imuQueue.get())
        m_imuQueue->removeCallback(imuCallbackId);

    spdlog::info("{} OakRos quitting", m_params.device_id);
}

void OakRos::depthCallback(std::shared_ptr<dai::ADatatype> data) {
    std::shared_ptr<dai::ImgFrame> depthFrame = std::static_pointer_cast<dai::ImgFrame>(data);

    unsigned int seq = depthFrame->getSequenceNum();
    double ts = depthFrame->getTimestamp().time_since_epoch().count() / 1.0e9;

    spdlog::debug("{} depth seq = {}, ts = {}", m_params.device_id, seq, ts);
}

void OakRos::imuCallback(std::shared_ptr<dai::ADatatype> data) {
    std::shared_ptr<dai::IMUData> imuData = std::static_pointer_cast<dai::IMUData>(data);

    // check if we are the first time getting IMU data

    if (!m_imuPub.get()) {
        m_imuPub.reset(
            new auto(m_nh.advertise<sensor_msgs::Imu>(m_params.topic_name + "/imu", 100)));
        spdlog::info("{} received first IMU message!", m_params.device_id);
    }

    auto imuPackets = imuData->packets;
    for (auto &imuPacket : imuPackets) {
        auto &acceleroValues = imuPacket.acceleroMeter;
        auto &gyroValues = imuPacket.gyroscope;

        double acceleroTs = acceleroValues.timestamp.get().time_since_epoch().count() / 1.0e9;
        double gyroTs = gyroValues.timestamp.get().time_since_epoch().count() / 1.0e9;

        spdlog::debug("{} imu accel ts = {}", m_params.device_id, acceleroTs);
        spdlog::debug("{} imu gyro ts = {}", m_params.device_id, gyroTs);

        sensor_msgs::Imu imuMsg;
        // TODO: here we assume to align with gyro timestamp
        bool errorDetect = false;
        if (std::abs(acceleroTs - gyroTs) > 0.015) {
            spdlog::warn("{} large ts difference between gyro and accel reading detected = {}",
                         m_params.device_id, std::abs(acceleroTs - gyroTs));
            errorDetect = true;
        }

        if (lastGyroTs > 0) {
            // check if the timestamp is regressing

            if (gyroTs <= lastGyroTs) {
                spdlog::warn("{} gyro ts regressing detected {} -> {}", m_params.device_id,
                             lastGyroTs, gyroTs);
                errorDetect = true;
            }

            if (gyroTs > lastGyroTs + 0.1) {
                spdlog::warn("{} gyro ts jump detected {} -> {}", m_params.device_id, lastGyroTs,
                             gyroTs);
                errorDetect = true;
            }
        }
        lastGyroTs = gyroTs;

        if (!errorDetect) {
            imuMsg.header.stamp = ros::Time().fromSec(gyroTs);

            imuMsg.angular_velocity.x = gyroValues.x;
            imuMsg.angular_velocity.y = gyroValues.y;
            imuMsg.angular_velocity.z = gyroValues.z;

            imuMsg.linear_acceleration.x = acceleroValues.x;
            imuMsg.linear_acceleration.y = acceleroValues.y;
            imuMsg.linear_acceleration.z = acceleroValues.z;

            m_imuPub->publish(imuMsg);
        }

        // printf("Accelerometer timestamp: %ld ms\n",
        // static_cast<long>(acceleroTs.time_since_epoch().count())); printf("Accelerometer [m/s^2]:
        // x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
        // printf("Gyroscope timestamp: %ld ms\n",
        // static_cast<long>(gyroTs.time_since_epoch().count())); printf("Gyroscope [rad/s]: x: %.3f
        // y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
    }
}

sensor_msgs::CameraInfo OakRos::getCameraInfo(std::shared_ptr<dai::ImgFrame> img,
                                              dai::CameraBoardSocket socket) {
    sensor_msgs::CameraInfo info;

    if (m_noCalib.count(socket)) {
        info.width = static_cast<uint32_t>(img->getWidth());
        info.height = static_cast<uint32_t>(img->getHeight());
        return info;
    }

    std::vector<double> flatIntrinsics, distCoeffsDouble;

    try {
        dai::CalibrationHandler calibData = m_device->readCalibration();

        std::vector<std::vector<float>> intrinsics, extrinsics;
        intrinsics = calibData.getCameraIntrinsics(socket);
        // extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, socket);

        // fill in K
        flatIntrinsics.resize(9);
        for (int i = 0; i < 3; i++) {
            std::copy(intrinsics[i].begin(), intrinsics[i].end(), flatIntrinsics.begin() + 3 * i);
        }

        std::copy(flatIntrinsics.begin(), flatIntrinsics.end(), info.K.begin());

        // fill in P
        info.P.at(0) = intrinsics[0][0];
        info.P.at(1) = intrinsics[0][1];
        info.P.at(2) = intrinsics[0][2];

        // camerainfo.P.at(3) = 0; // Tx, -fx * B
        // This is the translation term Tx for right camera, assuming left cam is the origin
        info.P.at(3) = 0;

        info.P.at(4) = intrinsics[1][0];
        info.P.at(5) = intrinsics[1][1];
        info.P.at(6) = intrinsics[2][2];
        info.P.at(7) = 0; // Ty
        info.P.at(8) = intrinsics[2][0];
        info.P.at(9) = intrinsics[2][1];
        info.P.at(10) = intrinsics[2][2];
        info.P.at(11) = 0;

        // set R (rotation matrix) values to identity matrix
        info.R.at(0) = 1.0;
        info.R.at(1) = 0.0;
        info.R.at(2) = 0.0;
        info.R.at(3) = 0.0;
        info.R.at(4) = 1.0;
        info.R.at(5) = 0.0;
        info.R.at(6) = 0.0;
        info.R.at(7) = 0.0;
        info.R.at(8) = 1.0;

        info.width = static_cast<uint32_t>(img->getWidth());
        info.height = static_cast<uint32_t>(img->getHeight());

        // undistrotion is always done in OAK camera, so distortion coefficient should be zero
        // always
        info.distortion_model = "opencv";
    } catch (std::exception &e) {
        spdlog::error("readCalibration() error: {}", e.what());
        m_noCalib.insert(socket);
    }

    return info;
}

dai::DeviceInfo OakRos::getDeviceInfo(const std::string &device_id) {
    auto DeviceInfo_vec = dai::Device::getAllAvailableDevices();

    for (auto &info : DeviceInfo_vec) {
        if (info.getMxId() == device_id) {
            spdlog::info("found device with specified id {}", device_id);
            return info;
        }
    }
    spdlog::error("failed to find device with id {}", device_id);
    throw std::runtime_error("");
}