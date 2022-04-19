#pragma once

#include <thread>

#include <spdlog/spdlog.h>

#include <depthai/depthai.hpp>

#include <oak_ros/OakRosInterface.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>

#include <set>

class OakRos : public OakRosInterface
{
public:
    void init(const ros::NodeHandle &nh, const OakRosParams &params);

    void deinit(){
        m_running = false;

        if (m_run.joinable())
            m_run.join();

        m_imageTransport.reset();
        m_leftPub.reset();
        m_rightPub.reset();
        m_imuPub.reset();

        m_device.reset();

        m_pipeline = dai::Pipeline();
    }

    void restart(){
        restartCount++;

        // if (restartCount > 2){
        //     throw std::runtime_error("Too many restart attempted, failed to recover");
        // }

        spdlog::warn("restarting oak camera {}", m_params.device_id);
        deinit();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        init(m_nh, m_params);
        spdlog::warn("restarting compeleted for camera {}", m_params.device_id);
    }

    static std::vector<std::string> getAllAvailableDeviceIds();

    dai::DeviceInfo getDeviceInfo(const std::string& device_id);

    ~OakRos(){
        deinit();
        spdlog::info("{} OakRos class destructor done.", m_device_id);
    }

private:

    bool m_running;
    OakRosParams m_params;
    bool m_stereo_is_rectified;

    int restartCount = 0;

    // 1 means no throttling, only publish every N frames
    double lastGyroTs;
    
    bool m_ts_align_to_right;

    std::string m_device_id;
    std::string m_topic_name;

    dai::Pipeline m_pipeline;

    std::shared_ptr<dai::Device> m_device;
    std::shared_ptr<dai::DataOutputQueue> m_leftQueue, m_rightQueue, m_depthQueue, m_rgbQueue, m_camDQueue, m_imuQueue;

    std::shared_ptr<dai::DataInputQueue> m_controlQueue;

    // WORKAROUND FOR OV7251, inability to reduce framerate
    std::shared_ptr<dai::node::Script> m_scriptLeft, m_scriptRight;
    std::shared_ptr<dai::node::Script> m_scriptRgb, m_scriptCamD;

    std::shared_ptr<dai::node::MonoCamera> m_monoLeft, m_monoRight;
    std::shared_ptr<dai::node::StereoDepth> m_stereoDepth;

    std::shared_ptr<dai::node::MonoCamera> m_rgb, m_camD;

    std::shared_ptr<dai::node::IMU> m_imu;

    std::thread m_run, m_watchdog;
    void run();

    void runOnceStereo();
    void runOnceAllFourCams();

    // the fuctions below is to setup pipeline before m_device
    void configureRatesWorkaround();
    std::shared_ptr<dai::node::XLinkIn> configureControl();
    void configureImu();
    void configureStereoRgbCamD();

    // the functions below assumes m_device is properly setup
    void setupControlQueue(std::shared_ptr<dai::node::XLinkIn>);
    void setupImuQueue();
    void setupStereoQueue();
    void setupRgbQueue();
    void setupCamDQueue();

    void depthCallback(std::shared_ptr<dai::ADatatype> data);
    void imuCallback(std::shared_ptr<dai::ADatatype> data);

    std::set<dai::CameraBoardSocket> m_noCalib;
    sensor_msgs::CameraInfo getCameraInfo(std::shared_ptr<dai::ImgFrame> img, dai::CameraBoardSocket socket); // In Oak convention, right camera is the main camera

    // ROS related functionalities
    ros::NodeHandle m_nh;
    std::shared_ptr<image_transport::ImageTransport> m_imageTransport;
    std::shared_ptr<image_transport::CameraPublisher> m_leftPub, m_rightPub, m_rgbPub, m_camDPub, m_debugImagePub;
    std::shared_ptr<ros::Publisher> m_imuPub;

};