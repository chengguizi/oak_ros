#include "OakRos.hpp"

#include <algorithm>
#include <chrono>

#include <cv_bridge/cv_bridge.h>

std::vector<sensor_msgs::Imu>
ImuInterpolation::updatePacket(const dai::IMUReportAccelerometer &accel,
                               const dai::IMUReportGyroscope &gyro) {
    if (m_accelHist.size() == 0 || m_accelHist.back().sequence != accel.sequence) {
        m_accelHist.push_back(accel);
    }

    if (m_gyroHist.size() == 0 || m_gyroHist.back().sequence != gyro.sequence) {
        m_gyroHist.push_back(gyro);
    }

    // interpolate accelerometer measurement to align to gyro

    std::vector<sensor_msgs::Imu> ret;

    while (m_accelHist.size() >= 3) {
        dai::IMUReportAccelerometer accel0, accel1;
        dai::IMUReportGyroscope currGyro;

        accel0 = m_accelHist.front();
        m_accelHist.pop_front();
        accel1 = m_accelHist.front();

        const double accel0Ts = accel0.timestamp.get().time_since_epoch().count() / 1.0e9;
        const double accel1Ts = accel1.timestamp.get().time_since_epoch().count() / 1.0e9;

        // sanity check for regression

        if (accel0Ts >= accel1Ts) {
            spdlog::warn("accelerometer ts regression detected: {} -> {}", accel0Ts, accel1Ts);
            continue;
        } else if (accel1Ts - accel0Ts > 3.0 / m_imuFrequency) {
            spdlog::warn("large gap between accel measurements, dt = {}, dropping",
                         accel1Ts - accel0Ts);
            continue;
        }

        if (m_gyroHist.size() == 0)
            spdlog::warn("No gyro message left for interpolation, shouldn't happen");

        while (m_gyroHist.size()) {
            currGyro = m_gyroHist.front();
            const double gyroTs = currGyro.timestamp.get().time_since_epoch().count() / 1.0e9;

            // if gyro reading is more recent than accel1, abort
            if (gyroTs > accel1Ts)
                break;
            else if (gyroTs < accel0Ts) {
                spdlog::warn("stray gyro measurement found at {}, older than oldest accel "
                             "measurement {}, dropping",
                             gyroTs, accel0Ts);
                break;
            }

            spdlog::debug(
                "imu interploation: from accel0 {} ({}) and accel1 {} ({}) to gyro {} ({})",
                accel0.sequence, accel0Ts, accel1.sequence, accel1Ts, currGyro.sequence, gyroTs);

            const double alpha = (gyroTs - accel0Ts) / (accel1Ts - accel0Ts);
            dai::IMUReportAccelerometer interpAccel = lerpImu(accel0, accel1, alpha);

            sensor_msgs::Imu imuMsg;
            imuMsg.header.frame_id = "imu";
            imuMsg.header.stamp = ros::Time().fromSec(gyroTs);

            imuMsg.angular_velocity.x = currGyro.x;
            imuMsg.angular_velocity.y = currGyro.y;
            imuMsg.angular_velocity.z = currGyro.z;

            imuMsg.linear_acceleration.x = interpAccel.x;
            imuMsg.linear_acceleration.y = interpAccel.y;
            imuMsg.linear_acceleration.z = interpAccel.z;

            ret.push_back(imuMsg);

            m_gyroHist.pop_front();
        }
    }

    return ret;
}

void OakRos::init(const ros::NodeHandle &nh, const OakRosParams &params) {

    m_params = params;
    // ROS-related
    m_nh = nh;

    lastGyroTs = -1;
    m_masterCamera = "";

    spdlog::info("initialising device {}", m_params.device_id);

    configureCameras();

    // alway try to see if IMU stream is there
    if (m_params.enable_imu)
        configureImu();

    auto xinControl = configureControl();

    m_pipeline.setXLinkChunkSize(0);

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
    for (auto &item : m_cameraMap) {
        auto &cameraName = item.first;

        m_cameraQueueMap[cameraName] = m_device->getOutputQueue(cameraName, 10, false);

        spdlog::info("{} advertising {} cameras in ros topic...", m_params.device_id, cameraName);
        m_cameraPubMap[cameraName].reset(new auto(m_imageTransport->advertiseCamera(
            m_params.topic_name + "/" + cameraName + "/image_rect_raw", 3)));
    }

    for (auto &item : m_stereoDepthMap) {
        auto &stereoDepthName = item.first;
        m_depthQueueMap[stereoDepthName] = m_device->getOutputQueue(stereoDepthName, 2, false);
    }

    if (m_params.enable_imu) {
        m_imuQueue = m_device->getOutputQueue("imu", 50, false);
    }

    // control for exposure and gain etc
    setupControlQueue(xinControl);

    m_run = std::thread(&OakRos::run, this);
}

std::shared_ptr<dai::node::XLinkIn> OakRos::configureControl() {
    auto xinControl = m_pipeline.create<dai::node::XLinkIn>();
    xinControl->setStreamName("control");

    // Linking

    for (auto &item : m_cameraMap) {
        auto &camera = item.second;
        xinControl->out.link(camera->inputControl);
    }

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

    if (m_params.imu_use_raw) {
        spdlog::info("IMU is taking raw (unfiltered) values from sensor");
        imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW},
                             m_params.imu_frequency);
    } else {
        spdlog::info("IMU is taking filtered values from sensor");
        imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER, dai::IMUSensor::GYROSCOPE_CALIBRATED},
                             m_params.imu_frequency);
    }

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

void OakRos::configureCamera(std::string cameraName) {
    auto xout = m_pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName(cameraName);

    spdlog::info("Enable {} socket camera...", cameraName);
    m_cameraMap[cameraName] = m_pipeline.create<dai::node::MonoCamera>();

    auto &camera = m_cameraMap[cameraName];

    if (m_params.resolutionMap.count(cameraName)) {
        camera->setResolution(m_params.resolutionMap[cameraName]);
    }

    camera->setBoardSocket(m_cameraSocketMap[cameraName]);

    if (m_params.all_cameras_fps) {
        camera->setFps(m_params.all_cameras_fps.value());
    }

    camera->out.link(xout->input);
}

void OakRos::configureCameras() {
    std::map<std::string, std::shared_ptr<dai::node::XLinkOut>> xoutCameraMap;

    // configure the stereo sensors' format
    std::shared_ptr<dai::node::StereoDepth> stereoDepth;

    if (m_params.enable_rgb) {
        m_cameraSocketMap["rgb"] = dai::CameraBoardSocket::RGB;
        configureCamera("rgb");
    }

    if (m_params.enable_left) {
        m_cameraSocketMap["left"] = dai::CameraBoardSocket::LEFT;
        configureCamera("left");
    }

    if (m_params.enable_right) {
        m_cameraSocketMap["right"] = dai::CameraBoardSocket::RIGHT;
        configureCamera("right");
    }

    if (m_params.enable_camD) {
        m_cameraSocketMap["camd"] = dai::CameraBoardSocket::CAM_D;
        configureCamera("camd");
    }

    if (m_params.hardware_sync) {

        // put all sensors as slave first
        for (auto &item : m_cameraMap) {
            // auto& cameraName = item.first;
            auto camera = item.second;
            camera->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
        }

        if (m_cameraMap.count("rgb")) {
            spdlog::info("setting rgb camera as master");
            m_cameraMap.at("rgb")->initialControl.setFrameSyncMode(
                dai::CameraControl::FrameSyncMode::OUTPUT);
            m_masterCamera = "rgb";
        }

        if (m_masterCamera.empty() && m_cameraMap.count("left")) {
            spdlog::info("setting left camera as master");
            m_cameraMap.at("left")->initialControl.setFrameSyncMode(
                dai::CameraControl::FrameSyncMode::OUTPUT);
            m_masterCamera = "left";
        }

        if (m_masterCamera.empty())
            throw std::runtime_error("no master camera configured!");
    }
}

void OakRos::setupCameraQueue(std::string cameraName) {}

void OakRos::run() {
    m_running = true;

    if (m_watchdog.joinable())
        m_watchdog.join();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    spdlog::info("{} OakRos running now", m_params.device_id);

    dai::DataOutputQueue::CallbackId depthCallbackId, imuCallbackId;

    // if (m_depthQueue.get()) {
    //     spdlog::info("{} adds depth queue callback", m_params.device_id);
    //     depthCallbackId = m_depthQueue->addCallback(
    //         std::bind(&OakRos::depthCallback, this, std::placeholders::_1));
    // }

    if (m_imuQueue.get()) {
        spdlog::info("{} adds imu queue callback", m_params.device_id);
        imuCallbackId =
            m_imuQueue->addCallback(std::bind(&OakRos::imuCallback, this, std::placeholders::_1));
    }

    std::map<std::string, cv::Mat> cvFrameMap;
    std::map<std::string, sensor_msgs::CameraInfo> cameraInfoMap;

    try {

        std::vector<std::string> cameraList;
        std::vector<long> seqList;
        std::vector<double> tsList;
        std::vector<std::shared_ptr<dai::ImgFrame>> frameList;

        std::vector<std::string> cameraCandidates = {"rgb", "left", "right", "camd"};
        for (std::string &cameraName : cameraCandidates) {
            if (m_cameraMap.count(cameraName)) {
                spdlog::info("Streaming {} at index {}", cameraName, cameraList.size());
                cameraList.push_back(cameraName);
                seqList.push_back(0);
                tsList.push_back(-1);
                frameList.push_back({});
            }
        }

        double maxTs = 0;

        if (m_params.debug_opencv_images) {
            cv::namedWindow("debug image", cv::WINDOW_NORMAL);
        }

        while (m_running) {
            // process stereo data
            constexpr double deltaT = 2e-3; // 2ms trigger delay
            if (cameraList.size() > 0) {

                try {

                    for (size_t i = 0; i < cameraList.size(); i++) {
                        auto &cameraName = cameraList[i];

                        while (tsList[i] < maxTs - deltaT) {
                            frameList[i] = m_cameraQueueMap.at(cameraName)->get<dai::ImgFrame>();
                            tsList[i] =
                                frameList[i]->getTimestamp().time_since_epoch().count() / 1.0e9;
                        }
                    }

                } catch (std::exception &e) {
                    spdlog::warn("get() for image failed: {}", e.what());
                    m_watchdog = std::thread(&OakRos::restart, this);
                    m_running = false;
                    continue;
                }

                maxTs = *std::max_element(tsList.begin(), tsList.end());

                // if any of the sequence does not match the latest one, try again
                bool retry = false;

                for (size_t i = 0; i < cameraList.size(); i++) {
                    auto &cameraName = cameraList[i];
                    if (maxTs - tsList[i] > deltaT)
                        retry = true;

                    spdlog::debug("{} = {}", cameraName, tsList[i]);
                }

                spdlog::debug("maxTs = {}", maxTs);

                if (m_params.hardware_sync && retry)
                    continue;

                spdlog::debug("synced on ts {} for all cameras", maxTs);
                maxTs += deltaT + deltaT;

                // detect for jumps
                constexpr int frameInterval = 1;

                for (size_t i = 0; i < cameraList.size(); i++) {
                    auto &cameraName = cameraList[i];

                    long seqNow = frameList[i]->getSequenceNum();

                    if (seqNow - frameInterval != seqList[i])
                        spdlog::warn(
                            "jump detected in {} image frames, from {} to {}, should be to {}",
                            cameraName, seqList[i], seqNow, seqList[i] + frameInterval);

                    seqList[i] = seqNow;

                    spdlog::debug("{} {} seq = {}, ts = {}", m_params.device_id, cameraName,
                                  seqList[i], tsList[i]);
                }

                // initialise camera_info msgs if needed
                for (size_t i = 0; i < cameraList.size(); i++) {
                    auto &cameraName = cameraList[i];

                    if (cameraInfoMap.count(cameraName) == 0) {
                        cameraInfoMap[cameraName] =
                            getCameraInfo(frameList[i], m_cameraSocketMap[cameraName]);
                    }
                }

                // publish frame and camera info
                for (size_t i = 0; i < cameraList.size(); i++) {
                    auto &cameraName = cameraList[i];

                    cvFrameMap[cameraName] = frameList[i]->getFrame();

                    if (m_params.hardware_sync)
                        cameraInfoMap[cameraName].header.stamp = ros::Time().fromSec(tsList[0]);
                    else
                        cameraInfoMap[cameraName].header.stamp = ros::Time().fromSec(tsList[i]);

                    cv_bridge::CvImage cvBridgeImage = cv_bridge::CvImage(
                        cameraInfoMap[cameraName].header, sensor_msgs::image_encodings::MONO8,
                        cvFrameMap[cameraName]);

                    m_cameraPubMap[cameraName]->publish(*cvBridgeImage.toImageMsg(),
                                                        cameraInfoMap[cameraName]);
                }

                if (m_params.debug_opencv_images) {
                    cv::Mat debugImage;

                    for (size_t i = 0; i < cameraList.size(); i++) {
                        auto &cameraName = cameraList[i];

                        if (debugImage.empty())
                            debugImage = cvFrameMap[cameraName].clone();
                        else
                            cv::hconcat(debugImage, cvFrameMap[cameraName], debugImage);
                    }

                    // sensor_msgs::CameraInfo info;
                    // info.width = debugImage.cols;
                    // info.height = debugImage.rows;

                    // if (runStereo && m_params.align_ts_to_right)
                    //     info.header.stamp = ros::Time().fromSec(tsRight);
                    // else if (runRgb && m_params.align_ts_to_right)
                    //     info.header.stamp = ros::Time().fromSec(tsRgb);

                    // cv_bridge::CvImage debugBridge =
                    //     cv_bridge::CvImage(info.header,
                    //                         sensor_msgs::image_encodings::MONO8, debugImage);

                    // m_debugImagePub->publish(*debugBridge.toImageMsg(), info);

                    cv::imshow("debug image", debugImage);
                    cv::resizeWindow("debug image", 1000, 280);
                    cv::waitKey(3);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(2));

            } else
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (std::exception &e) {
        spdlog::warn("while(m_running) failed: {}", e.what());
        m_watchdog = std::thread(&OakRos::restart, this);
    }

    // if (m_depthQueue.get())
    //     m_depthQueue->removeCallback(depthCallbackId);

    if (m_imuQueue.get())
        m_imuQueue->removeCallback(imuCallbackId);

    spdlog::info("{} OakRos quitting", m_params.device_id);
}

// void OakRos::depthCallback(std::shared_ptr<dai::ADatatype> data) {
//     std::shared_ptr<dai::ImgFrame> depthFrame = std::static_pointer_cast<dai::ImgFrame>(data);

//     unsigned int seq = depthFrame->getSequenceNum();
//     double ts = depthFrame->getTimestamp().time_since_epoch().count() / 1.0e9;

//     spdlog::debug("{} depth seq = {}, ts = {}", m_params.device_id, seq, ts);
// }

void OakRos::imuCallback(std::shared_ptr<dai::ADatatype> data) {
    std::shared_ptr<dai::IMUData> imuData = std::static_pointer_cast<dai::IMUData>(data);

    // check if we are the first time getting IMU data

    if (!m_imuPub.get()) {
        m_imuPub.reset(
            new auto(m_nh.advertise<sensor_msgs::Imu>(m_params.topic_name + "/imu", 100)));
        spdlog::info("{} received first IMU message!", m_params.device_id);
    }

    if (!imuInterpolation.get()) {
        imuInterpolation.reset(new ImuInterpolation(m_params.imu_frequency));
    }

    auto imuPackets = imuData->packets;
    for (auto &imuPacket : imuPackets) {
        auto &acceleroValues = imuPacket.acceleroMeter;
        auto &gyroValues = imuPacket.gyroscope;

        spdlog::debug("accel seq = {}, gyro seq = {}", acceleroValues.sequence,
                      gyroValues.sequence);

        constexpr bool do_interpolation = true;

        if (!do_interpolation) {
            double acceleroTs = acceleroValues.timestamp.get().time_since_epoch().count() / 1.0e9;
            double gyroTs = gyroValues.timestamp.get().time_since_epoch().count() / 1.0e9;

            spdlog::info("{} imu accel ts = {}", m_params.device_id, acceleroTs);
            spdlog::info("{} imu gyro ts = {}", m_params.device_id, gyroTs);

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
                    spdlog::warn("{} gyro ts jump detected {} -> {}", m_params.device_id,
                                 lastGyroTs, gyroTs);
                    errorDetect = true;
                }
            }
            lastGyroTs = gyroTs;

            if (!errorDetect) {
                imuMsg.header.frame_id = "imu";
                imuMsg.header.stamp = ros::Time().fromSec(gyroTs);

                imuMsg.angular_velocity.x = gyroValues.x;
                imuMsg.angular_velocity.y = gyroValues.y;
                imuMsg.angular_velocity.z = gyroValues.z;

                imuMsg.linear_acceleration.x = acceleroValues.x;
                imuMsg.linear_acceleration.y = acceleroValues.y;
                imuMsg.linear_acceleration.z = acceleroValues.z;

                m_imuPub->publish(imuMsg);
            }
        } else {
            auto imuMsgs = imuInterpolation->updatePacket(acceleroValues, gyroValues);

            for (auto &imuMsg : imuMsgs) {
                m_imuPub->publish(imuMsg);
            }
        }
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