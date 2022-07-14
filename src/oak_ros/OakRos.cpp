#include "OakRos.hpp"

#include <algorithm>
#include <chrono>
#include <algorithm>

#include <cv_bridge/cv_bridge.h>

#include "oak_ros/PointCloudConverter.hpp"
#include "oak_ros/MeshDataGenerator.hpp"

#include <opencv2/core/eigen.hpp>

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
                // we would like to throw away such a reading, and start again
                m_gyroHist.pop_front();
                continue;
            }

            if (m_lastGyroTs >= gyroTs) {

                spdlog::warn("skip imu reading due to timestamp, prev = {}, now = {}", m_lastGyroTs, gyroTs);
                m_gyroHist.pop_front();
                continue;
            }else
                m_lastGyroTs = gyroTs;

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

    // obtain calibration
    {
        spdlog::info("Obtaining Calibration Data");
        std::shared_ptr<dai::Device> device;
        dai::Pipeline pipeline;
        if (m_params.device_id.empty()) {
            // spdlog::info("Creating device without specific id");
            device = std::make_shared<dai::Device>(pipeline);
        } else {
            // spdlog::info("Creating device with specific id {}", m_params.device_id);
            device = std::make_shared<dai::Device>(pipeline, getDeviceInfo(m_params.device_id));
        }

        m_calibData = device->readCalibration();

        spdlog::info("Obtained Calibration Data");

    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    configureCameras();

    configureStereos();

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

    for (auto& item : m_stereoDepthMap) {

        std::string nodeName = item.first;

        if (m_params.enable_depth)
            m_depthQueueMap[nodeName] = m_device->getOutputQueue("depth_" + nodeName, 2, false);

        if (m_params.enable_disparity || m_params.enable_pointcloud) {
            
            m_disparityQueueMap[nodeName] = m_device->getOutputQueue("disparity_" + nodeName, 2, false);

            if (m_params.enable_pointcloud)
                m_depthMonoQueueMap[nodeName] = m_device->getOutputQueue("depthmono_" + nodeName, 2, false);
        }
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
    imu->setBatchReportThreshold(2);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until
    // host can receive it if lower or equal to batchReportThreshold then the sending is always
    // blocking on device useful to reduce device's CPU load  and number of lost packets, if CPU
    // load is high on device side due to multiple nodes
    imu->setMaxBatchReports(5);

    // Link plugins IMU -> XLINK
    imu->out.link(xoutIMU->input);
}

void OakRos::configureCamera(std::string cameraName) {
    
    const bool part_of_stereo_pairs = std::any_of(m_params.enabled_stereo_pairs.cbegin(), m_params.enabled_stereo_pairs.cend(), 
        [cameraName](const auto& e){ return e.second.first == cameraName || e.second.second == cameraName;});

    spdlog::info("Enable {} socket camera, part of stereo pair = {}", cameraName, part_of_stereo_pairs ? "true" : "false");
    m_cameraMap[cameraName] = m_pipeline.create<dai::node::MonoCamera>();

    auto &camera = m_cameraMap[cameraName];

    if (m_params.resolutionMap.count(cameraName)) {
        camera->setResolution(m_params.resolutionMap[cameraName]);
    }else {
        throw std::runtime_error("expect resolution to be defined explicitly for each camera");
    }

    camera->setBoardSocket(m_cameraSocketMap[cameraName]);

    if (m_params.all_cameras_fps) {
        camera->setFps(m_params.all_cameras_fps.value());
    }

    // we only establish xlink with the raw camera output if the camera:
    // - is not part of the stereo pairs

    if (!part_of_stereo_pairs) {
        auto xout = m_pipeline.create<dai::node::XLinkOut>();
        xout->setStreamName(cameraName);
        camera->out.link(xout->input);

        spdlog::info("output raw (un-rectified) images");
    }
}

void OakRos::configureCameras() {
    std::map<std::string, std::shared_ptr<dai::node::XLinkOut>> xoutCameraMap;

    // configure the stereo sensors' format
    std::shared_ptr<dai::node::StereoDepth> stereoDepth;

    if (m_params.enable_rgb) {
        m_cameraSocketMap["rgb"] = m_socketMapping["rgb"];
        configureCamera("rgb");
    }

    if (m_params.enable_left) {
        m_cameraSocketMap["left"] = m_socketMapping["left"];
        configureCamera("left");
    }

    if (m_params.enable_right) {
        m_cameraSocketMap["right"] = m_socketMapping["right"];
        configureCamera("right");
    }

    if (m_params.enable_camD) {
        m_cameraSocketMap["camd"] = m_socketMapping["camd"];
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

        if (m_masterCamera.empty() && m_cameraMap.count("camd")) {
            spdlog::info("setting camd camera as master");
            m_cameraMap.at("camd")->initialControl.setFrameSyncMode(
                dai::CameraControl::FrameSyncMode::OUTPUT);
            m_masterCamera = "camd";
        }

        if (m_masterCamera.empty())
            throw std::runtime_error("no master camera configured!");
    }
}

void OakRos::configureStereos() {

    for (const auto& item : m_params.enabled_stereo_pairs) {

        auto& name = item.first;
        auto& pair = item.second;

        spdlog::info("Configure stereo pair {}", name);
        
        const auto& leftName = pair.first;
        const auto& rightName = pair.second;

        const std::string nameStereo = leftName + "_" + rightName;

        spdlog::info("configure stereo pair {}", nameStereo);

        auto xoutLeft = m_pipeline.create<dai::node::XLinkOut>();
        auto xoutRight = m_pipeline.create<dai::node::XLinkOut>();
        xoutLeft->setStreamName(leftName);
        xoutRight->setStreamName(rightName);

        auto monoLeft = m_cameraMap.at(leftName);
        auto monoRight = m_cameraMap.at(rightName);

        if (!(m_params.enable_depth || m_params.enable_disparity || m_params.enable_pointcloud ||
                 m_params.enable_stereo_rectified)) {
            
            spdlog::warn("stereo configured but not used for any outputs (depth, disparity, point cloud, or rectification)");
            monoLeft->out.link(xoutLeft->input);
            monoRight->out.link(xoutRight->input);

        }else {
            
            auto stereoDepth = m_pipeline.create<dai::node::StereoDepth>();

            m_stereoDepthMap[name] = stereoDepth;

            stereoDepth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
            stereoDepth->setRectifyEdgeFillColor(0); // black, to better see the cutout
            // stereoDepth->setInputResolution(1280, 720);
            stereoDepth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
            stereoDepth->setLeftRightCheck(true);
            stereoDepth->setExtendedDisparity(false);
            stereoDepth->setSubpixel(true);

            auto PostConfig = stereoDepth->initialConfig.get();

            PostConfig.postProcessing.speckleFilter.enable = true;

            // Following realsense D435i
            PostConfig.postProcessing.temporalFilter.enable = false;
            // PostConfig.postProcessing.temporalFilter.alpha = 0.5;
            // PostConfig.postProcessing.temporalFilter.delta = 20;

            PostConfig.postProcessing.spatialFilter.enable = true;
            // Following realsense D435i
            PostConfig.postProcessing.spatialFilter.holeFillingRadius = 2;
            PostConfig.postProcessing.spatialFilter.numIterations = 1;
            // PostConfig.postProcessing.spatialFilter.alpha = 0.5;
            // PostConfig.postProcessing.spatialFilter.delta = 20;

            PostConfig.postProcessing.thresholdFilter.minRange = 200;
            PostConfig.postProcessing.thresholdFilter.maxRange = 8000;

            // Following realsense D435i
            PostConfig.postProcessing.decimationFilter.decimationFactor =
                m_params.depth_decimation_factor;

            // median filter generate quite some delay
            // PostConfig.postProcessing.decimationFilter.decimationMode =
            //     dai::RawStereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::NON_ZERO_MEDIAN;

            stereoDepth->initialConfig.set(PostConfig);

            // setup mesh
            if (m_params.use_mesh) {
                for (auto& item : m_stereoDepthMap) {

                    std::string name = item.first;
                    auto& stereoDepth = item.second;

                    constexpr int meshStep = 10;
                    stereoDepth->useHomographyRectification(false);
                    stereoDepth->setMeshStep(meshStep, meshStep);

                    std::vector<std::uint8_t> dataLeft, dataRight;

                    OakMeshDataGenerator meshGen;
                    auto& leftName = m_params.enabled_stereo_pairs[name].first;
                    auto& rightName = m_params.enabled_stereo_pairs[name].second;

                    assert(m_params.resolutionMap[leftName] == m_params.resolutionMap[rightName]);

                    meshGen.getRectificationTransformFromOpenCV(m_calibData, 
                        m_socketMapping[leftName], 
                        m_socketMapping[rightName], 
                        m_params.resolutionMap[leftName], m_params.mesh_alpha);
                    
                    meshGen.calculateMeshData(meshStep, dataLeft, dataRight);

                    m_newMMap[name] = meshGen.getNewM();

                    spdlog::warn("Use Generated mesh data for un-distortion and rectification on {}-{} pair", leftName, rightName);
                    stereoDepth->loadMeshData(dataLeft, dataRight);

                    // publish static tf
                    {
                        std::string cameraFrameId = m_params.tf_prefix + rightName + "_camera_rect_nwu";
                        std::string imuFrameId = m_params.tf_prefix + "imu";
                        std::string bodyFrameId = m_params.tf_prefix + "base_link_nwu";

                        Eigen::Isometry3f rdf_T_nwu;
                        rdf_T_nwu.matrix() <<   0, -1, 0, 0,
                                                0, 0, -1, 0,
                                                1, 0, 0, 0,
                                                0, 0, 0, 1;

                        Eigen::Isometry3f nwu_T_frd; // IMU
                        nwu_T_frd.matrix() <<   1, 0, 0, 0,
                                                0, -1, 0, 0,
                                                0, 0, -1, 0,
                                                0, 0, 0, 1;

                        Eigen::Matrix3f rightCamRect_R_rightCam;
                        cv2eigen(meshGen.getR2(), rightCamRect_R_rightCam);

                        Eigen::Isometry3f rightCamRect_T_rightCam = Eigen::Isometry3f::Identity();
                        rightCamRect_T_rightCam.linear() = rightCamRect_R_rightCam.inverse();

                        Eigen::Isometry3f rightCam_T_rightCamRect = rightCamRect_T_rightCam.inverse();
                        
                        // spdlog::info("rightCam_T_rightCamRect");
                        // std::cout << rightCam_T_rightCamRect.matrix() << std::endl;

                        Eigen::Isometry3f imu_T_rightCam;
                        cv2eigen(toMat(m_calibData.getCameraToImuExtrinsics(m_socketMapping[rightName])), imu_T_rightCam.matrix());

                        if (imu_T_rightCam.linear().maxCoeff() < 0.1) {
                            throw std::runtime_error("imu exintrisic not initialised on eeprom");
                        }

                        imu_T_rightCam.translation() /= 100.;

                        // spdlog::info("imu_T_rightCam");
                        // std::cout << imu_T_rightCam.matrix() << std::endl;

                        // tf between imu and rectified camera (nwu)
                        publishStaticTransform(imu_T_rightCam * rightCam_T_rightCamRect * rdf_T_nwu, imuFrameId, cameraFrameId);

                        // tf between body and imu
                        Eigen::Isometry3f body_T_imu = Eigen::Isometry3f::Identity();

                        body_T_imu = nwu_T_frd;

                        publishStaticTransform(body_T_imu, bodyFrameId, imuFrameId);
                    }
                }
            }else {
                for (auto& item : m_stereoDepthMap) {
                    stereoDepth->useHomographyRectification(false);
                }
            }

            // Linking
            monoLeft->out.link(stereoDepth->left);
            monoRight->out.link(stereoDepth->right);

            // depth output stream
            if (m_params.enable_depth) {
                spdlog::info("{} enabling depth streams for {}...", m_params.device_id, nameStereo);
                auto xoutDepth = m_pipeline.create<dai::node::XLinkOut>();
                xoutDepth->setStreamName("depth_" + nameStereo);
                stereoDepth->depth.link(xoutDepth->input);
            }

            if (m_params.enable_disparity || m_params.enable_pointcloud) {
                spdlog::info("{} enabling disparity stream for {}...", m_params.device_id, nameStereo);
                auto xoutDisparity = m_pipeline.create<dai::node::XLinkOut>();
                xoutDisparity->setStreamName("disparity_" + nameStereo);
                stereoDepth->disparity.link(xoutDisparity->input);
            }

            // output rectified image for point cloud colorisation

            if (m_params.enable_pointcloud) {

                spdlog::info("{} enabling pointcloud stream for {}...", m_params.device_id, nameStereo);

                auto xoutDepthMono = m_pipeline.create<dai::node::XLinkOut>();
                xoutDepthMono->setStreamName("depthmono_" + nameStereo);

                if ( m_params.depth_decimation_factor > 1) {
                    auto imageManip = m_imageManipMap[nameStereo] = m_pipeline.create<dai::node::ImageManip>();

                    const int width = monoRight->getResolutionWidth();
                    const int height = monoRight->getResolutionHeight();

                    imageManip->initialConfig.setResize(width / m_params.depth_decimation_factor, height / m_params.depth_decimation_factor);
                    // imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::RAW8);

                    // hm: workaround?
                    // imageManip->setMaxOutputFrameSize(3072000);
                    
                    stereoDepth->rectifiedRight.link(imageManip->inputImage);
                    imageManip->out.link(xoutDepthMono->input);
                }else {
                    stereoDepth->rectifiedRight.link(xoutDepthMono->input);
                }
                
            }

            if (!m_params.enable_stereo_rectified) {
                spdlog::info("{} enabling raw stereo streams for {}...", m_params.device_id, nameStereo);
                // output raw images

                if (m_params.enable_stereo_half_resolution_output && m_params.resolutionMap[rightName] == dai::MonoCameraProperties::SensorResolution::THE_800_P) {
                    
                    spdlog::warn("{}: configured to output images at 400p while depth is processed at 800p", nameStereo);

                    const int width = monoRight->getResolutionWidth();
                    const int height = monoRight->getResolutionHeight();
                    
                    auto imageManipLeft = m_pipeline.create<dai::node::ImageManip>();
                    auto imageManipRight = m_pipeline.create<dai::node::ImageManip>();

                    imageManipLeft->initialConfig.setResize(width / 2, height / 2);
                    imageManipRight->initialConfig.setResize(width / 2, height / 2);

                    monoLeft->out.link(imageManipLeft->inputImage);
                    monoRight->out.link(imageManipRight->inputImage);

                    imageManipLeft->out.link(xoutLeft->input);
                    imageManipRight->out.link(xoutRight->input);

                }else{
                    monoLeft->out.link(xoutLeft->input);
                    monoRight->out.link(xoutRight->input);
                }
                
            } else {
                spdlog::info("{} enabling rectified stereo streams for {}...", m_params.device_id, nameStereo);
                // output rectified images
                stereoDepth->rectifiedLeft.link(xoutLeft->input);
                stereoDepth->rectifiedRight.link(xoutRight->input);          
            }
        }
    }

}

void OakRos::run() {
    m_running = true;

    if (m_watchdog.joinable())
        m_watchdog.join();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    spdlog::info("{} OakRos running now", m_params.device_id);

    dai::DataOutputQueue::CallbackId imuCallbackId;
    std::map<std::string, dai::DataOutputQueue::CallbackId> depthCallbackIdMap, disparityCallbackIdMap;

    for (auto& item : m_depthQueueMap) {
        auto& name = item.first;
        auto& queue = item.second;

        spdlog::info("{} adds depth queue callback for {}", m_params.device_id, name);
        depthCallbackIdMap[name] = queue->addCallback(
            std::bind(&OakRos::depthCallback, this, std::placeholders::_1, name));
    }

    for (auto& item : m_disparityQueueMap) {
        auto& name = item.first;
        auto& queue = item.second;

        spdlog::info("{} adds disparity queue callback for {}", m_params.device_id, name);
        disparityCallbackIdMap[name] = queue->addCallback(
            std::bind(&OakRos::disparityCallback, this, std::placeholders::_1, name));

        m_rightFrameHistoryMap[name] = std::queue<std::shared_ptr<dai::ImgFrame>>();
    }

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

    for (auto& item : m_depthQueueMap)
        item.second->removeCallback(depthCallbackIdMap[item.first]);

    for (auto& item : m_disparityQueueMap)
        item.second->removeCallback(disparityCallbackIdMap[item.first]);

    if (m_imuQueue.get())
        m_imuQueue->removeCallback(imuCallbackId);

    spdlog::info("{} OakRos quitting", m_params.device_id);
}

void OakRos::depthCallback(std::shared_ptr<dai::ADatatype> data, std::string name) {
    std::shared_ptr<dai::ImgFrame> depthFrame = std::static_pointer_cast<dai::ImgFrame>(data);

    unsigned int seq = depthFrame->getSequenceNum();
    double ts = depthFrame->getTimestamp().time_since_epoch().count() / 1.0e9;

    spdlog::info("{} depth seq = {}, ts = {}", m_params.device_id, seq, ts);
}

void OakRos::disparityCallback(std::shared_ptr<dai::ADatatype> data, std::string name) {
    std::shared_ptr<dai::ImgFrame> disparityFrame = std::static_pointer_cast<dai::ImgFrame>(data);

    auto& leftName = m_params.enabled_stereo_pairs[name].first;
    auto& rightName = m_params.enabled_stereo_pairs[name].second;

    unsigned int seq = disparityFrame->getSequenceNum();
    

    if (m_lastDispSeqMap.count(name) && seq > m_lastDispSeqMap[name] + 1) {
        spdlog::warn("jump detected in disp frame, from {} to {}. should be {}", m_lastDispSeqMap[name], seq,
                     m_lastDispSeqMap[name] + 1);
    }
    m_lastDispSeqMap[name] = seq;

    double ts = disparityFrame->getTimestamp().time_since_epoch().count() / 1.0e9;

    // spdlog::info("{} disparity seq = {}, ts = {}", m_params.device_id, seq, ts);

    const float baseline = m_calibData.getBaselineDistance(m_socketMapping[rightName],
                                                           m_socketMapping[leftName], false) /
                             100.;

    float fx, fy, cu, cv;


    // TODO: debug this, and put this outside the callback
    if (!m_params.use_mesh) {

        auto imageSize = getImageSize(m_cameraMap[rightName]->getResolution());

        // here we assume the on-device rectification uses the right camera's intrinsic
        std::vector<std::vector<float>> intrinsics = m_calibData.getCameraIntrinsics(
                m_socketMapping[rightName], imageSize.width,
                imageSize.height);

        fx = intrinsics[0][0];
        fy = intrinsics[1][1];
        cu = intrinsics[0][2];
        cv = intrinsics[1][2];
    }else {
        cv::Mat_<float> intrinsics =  m_newMMap[name];

        fx = intrinsics(0, 0);
        fy = intrinsics(1, 1);
        cu = intrinsics(0, 2);
        cv = intrinsics(1, 2);
    }

    if (m_params.enable_disparity) {

        if (!m_disparityPubMap.count(name)) {
            m_disparityPubMap.emplace(name, new auto(m_nh.advertise<stereo_msgs::DisparityImage>(
                m_params.topic_name + "/" + name + "_disparity", 10)));
            spdlog::info("{} received first disparity image message at pair {}!", m_params.device_id, name);

            m_outDispImageMsgMap.emplace(name, new stereo_msgs::DisparityImage);

            m_outDispImageMsgMap[name]->header.frame_id = m_params.tf_prefix + rightName + "_camera_optical_frame";

            

            // float _focalLength = 880;
            // float _baseline = 0.075;
            // float _maxDepth = 10;
            // float _minDepth = 0.1;
            m_outDispImageMsgMap[name]->f = (fx + fy) / 2.;
            // outDispImageMsg.min_disparity = _focalLength * _baseline / _maxDepth;
            // outDispImageMsg.max_disparity = _focalLength * _baseline / _minDepth;

            m_outDispImageMsgMap[name]->min_disparity = 2;
            m_outDispImageMsgMap[name]->max_disparity = 95;
        }

        sensor_msgs::Image &outImageMsg = m_outDispImageMsgMap[name]->image;

        outImageMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        outImageMsg.header = m_outDispImageMsgMap[name]->header;

        // const size_t size = disparityFrame->getData().size() * sizeof(float);
        const size_t size =
            disparityFrame->getHeight() * disparityFrame->getWidth() * sizeof(float);
        outImageMsg.data.resize(size);
        outImageMsg.height = disparityFrame->getHeight();
        outImageMsg.width = disparityFrame->getWidth();
        outImageMsg.step = size / disparityFrame->getHeight();
        outImageMsg.is_bigendian = true;

        // timestamp
        m_outDispImageMsgMap[name]->header.stamp = ros::Time().fromSec(ts);

        if (disparityFrame->getType() == dai::RawImgFrame::Type::RAW8) {
            // spdlog::info("raw 8 type");
            m_outDispImageMsgMap[name]->delta_d = 1.0;

            // spdlog::info("disparity frame width {}, height {}, size {}", outImageMsg.width,
            // outImageMsg.height, disparityFrame->getData().size());

            std::vector<float> convertedData(disparityFrame->getData().begin(),
                                             disparityFrame->getData().end());

            unsigned char *imageMsgDataPtr =
                reinterpret_cast<unsigned char *>(outImageMsg.data.data());

            unsigned char *daiImgData = reinterpret_cast<unsigned char *>(convertedData.data());

            // TODO(Sachin): Try using assign since it is a vector
            // img->data.assign(packet.data->cbegin(), packet.data->cend());
            memcpy(imageMsgDataPtr, daiImgData, size);

            m_disparityPubMap[name]->publish(m_outDispImageMsgMap[name]);

        } else if (disparityFrame->getType() == dai::RawImgFrame::Type::RAW16) {

            m_outDispImageMsgMap[name]->delta_d = 1.0 / 8;

            unsigned char *daiImgData =
                reinterpret_cast<unsigned char *>(disparityFrame->getData().data());

            std::vector<uint16_t> raw16Data(disparityFrame->getHeight() *
                                            disparityFrame->getWidth());
            unsigned char *raw16DataPtr = reinterpret_cast<unsigned char *>(raw16Data.data());
            memcpy(raw16DataPtr, daiImgData,
                   disparityFrame->getHeight() * disparityFrame->getWidth() * sizeof(uint16_t));
            std::vector<float> convertedData;
            std::transform(
                raw16Data.begin(), raw16Data.end(), std::back_inserter(convertedData),
                [](uint16_t disp) -> std::size_t { return static_cast<float>(disp) / 8.0; });

            unsigned char *imageMsgDataPtr =
                reinterpret_cast<unsigned char *>(outImageMsg.data.data());
            unsigned char *convertedDataPtr =
                reinterpret_cast<unsigned char *>(convertedData.data());
            memcpy(imageMsgDataPtr, convertedDataPtr, size);

            m_disparityPubMap[name]->publish(m_outDispImageMsgMap[name]);
        } else {
            spdlog::info("type : {}", disparityFrame->getType());
            throw std::runtime_error("not implemented");
        }
    }


    if (m_params.enable_pointcloud) {
        // tries to find matching
        std::shared_ptr<dai::ImgFrame> right;

        auto& depthMonoQueue = m_depthMonoQueueMap[name];

        while (true) {
            auto frame = depthMonoQueue->get<dai::ImgFrame>();

            m_rightFrameHistoryMap[name].push(frame);

            if (frame->getSequenceNum() >= seq)
                break;
        }

        while (m_rightFrameHistoryMap[name].size()) {
            std::shared_ptr<dai::ImgFrame> frame = m_rightFrameHistoryMap[name].front();
            
            unsigned int rightSeq = frame->getSequenceNum();
            if (rightSeq < seq) {
                m_rightFrameHistoryMap[name].pop();
                continue;
            }
            else if (rightSeq > seq) {
                spdlog::warn("fail to match the disparity frame seq {}, with next in line right frame seq {}", seq, rightSeq);
                break;
            }else {
                m_rightFrameHistoryMap[name].pop();
                right = frame;
                break; // found
            }

        }

        if (!right.get()){
            spdlog::warn("failed to find depthmono frame with seq {}, skip", seq);
            return;
        }

        double tsRight = right->getTimestamp().time_since_epoch().count() / 1.0e9;

        assert (ts == tsRight);

        if (!m_cloudPubFromDispMap.count(name)) {
            m_cloudPubFromDispMap.emplace(name, new auto(
                m_nh.advertise<sensor_msgs::PointCloud2>(m_params.topic_name + "/pointcloud", 10)));
            spdlog::info(
                "{} received first disparity image message! used for pointcloud generation",
                m_params.device_id);

            m_cloudMsgFromDispMap.emplace(name, new sensor_msgs::PointCloud2);
        }

        if (!m_disparity2PointCloudConverterMap.count(name)) {

            m_disparity2PointCloudConverterMap.emplace(name, new OakPointCloudConverter(
                fx, fy, cu, cv, baseline, m_params.depth_decimation_factor));

            // TODO: the frame should be right_camera instead of directly base_link, need to fix
            m_cloudMsgFromDispMap[name]->header.frame_id = m_params.tf_prefix + rightName + "_camera_rect_nwu"; // it should have a tf relation with base_link_nwu
            m_cloudMsgFromDispMap[name]->height = disparityFrame->getHeight();
            m_cloudMsgFromDispMap[name]->width = disparityFrame->getWidth();
            m_cloudMsgFromDispMap[name]->is_dense = false;
            m_cloudMsgFromDispMap[name]->is_bigendian = false;
        }

        // timestamp
        m_cloudMsgFromDispMap[name]->header.stamp = ros::Time().fromSec(tsRight);

        if (disparityFrame->getType() == dai::RawImgFrame::Type::RAW8) {
            m_disparity2PointCloudConverterMap[name]->Disparity2PointCloud<uint8_t>(disparityFrame, right, m_cloudMsgFromDispMap[name]);
        } else if (disparityFrame->getType() == dai::RawImgFrame::Type::RAW16) {
            m_disparity2PointCloudConverterMap[name]->setScale(8); // default 3 bit fractional
            m_disparity2PointCloudConverterMap[name]->Disparity2PointCloud<uint16_t>(disparityFrame, right, m_cloudMsgFromDispMap[name]);
        } else
            throw std::runtime_error("not implemented");

        m_cloudPubFromDispMap[name]->publish(m_cloudMsgFromDispMap[name]);
    }
}

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

void OakRos::publishStaticTransform (Eigen::Isometry3f T, std::string parentId, std::string childId) {
    geometry_msgs::TransformStamped tf;
    
    tf.header.stamp = ros::Time().fromNSec(ros::SteadyTime::now().toNSec());
    tf.header.frame_id = parentId;
    tf.child_frame_id = childId;

    Eigen::Quaternionf q(T.rotation());

    tf.transform.translation.x = T.translation()[0];
    tf.transform.translation.y = T.translation()[1];
    tf.transform.translation.z = T.translation()[2];
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();

    m_broadcaster.sendTransform(tf);

}