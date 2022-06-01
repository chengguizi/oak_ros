#pragma once

#include <thread>

#include <spdlog/spdlog.h>

#include <depthai/depthai.hpp>

#include <oak_ros/OakRosInterface.hpp>

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

#include <set>

class ImuInterpolation {
  public:
    ImuInterpolation(int imuFrequency) : m_imuFrequency(imuFrequency) {};
    std::vector<sensor_msgs::Imu> updatePacket(const dai::IMUReportAccelerometer &accel,
                                                 const dai::IMUReportGyroscope &gyro);

  private:
    int m_imuFrequency;
    std::deque<dai::IMUReportAccelerometer> m_accelHist;
    std::deque<dai::IMUReportGyroscope> m_gyroHist;

    template <typename T> T lerp(const T &a, const T &b, const double t) {
        return a * (1.0 - t) + b * t;
    }

    template <typename T> T lerpImu(const T &a, const T &b, const double t) {
        T res;
        res.x = lerp(a.x, b.x, t);
        res.y = lerp(a.y, b.y, t);
        res.z = lerp(a.z, b.z, t);
        return res;
    }
};

class OakRos : public OakRosInterface {
  public:
    void init(const ros::NodeHandle &nh, const OakRosParams &params);

    void deinit() {
        m_running = false;

        if (m_run.joinable())
            m_run.join();

        m_imageTransport.reset();

        for (auto &item : m_cameraPubMap) {
            item.second.reset();
        }

        m_imuPub.reset();
        m_device.reset();

        m_pipeline = dai::Pipeline();
    }

    void restart() {
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

    dai::DeviceInfo getDeviceInfo(const std::string &device_id);

    ~OakRos() {
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
    std::string m_masterCamera;

    std::string m_device_id;
    std::string m_topic_name;

    dai::Pipeline m_pipeline;

    std::shared_ptr<dai::Device> m_device;
    std::shared_ptr<dai::DataOutputQueue> m_imuQueue;
    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> m_cameraQueueMap;
    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> m_depthQueueMap;

    std::shared_ptr<dai::DataInputQueue> m_controlQueue;

    std::shared_ptr<dai::node::IMU> m_imu;
    std::map<std::string, std::shared_ptr<dai::node::MonoCamera>> m_cameraMap;
    std::map<std::string, dai::CameraBoardSocket> m_cameraSocketMap;
    std::map<std::string, std::shared_ptr<dai::node::StereoDepth>> m_stereoDepthMap;

    std::thread m_run, m_watchdog;
    void run();

    // the fuctions below is to setup pipeline before m_device creation
    std::shared_ptr<dai::node::XLinkIn> configureControl();
    void configureImu();
    void configureCameras();
    void configureCamera(std::string cameraName);

    // the functions below assumes m_device is properly setup
    void setupControlQueue(std::shared_ptr<dai::node::XLinkIn>);
    void setupImuQueue();
    void setupCameraQueue(std::string cameraName);

    void configureDepthNode(std::shared_ptr<dai::node::StereoDepth> stereoDepth,
                            std::shared_ptr<dai::node::MonoCamera> left,
                            std::shared_ptr<dai::node::MonoCamera> right,
                            std::shared_ptr<dai::node::XLinkOut> xoutDepth);

    // void depthCallback(std::shared_ptr<dai::ADatatype> data);
    void imuCallback(std::shared_ptr<dai::ADatatype> data);

    std::set<dai::CameraBoardSocket> m_noCalib;
    sensor_msgs::CameraInfo getCameraInfo(
        std::shared_ptr<dai::ImgFrame> img,
        dai::CameraBoardSocket socket); // In Oak convention, right camera is the main camera

    // ROS related functionalities
    ros::NodeHandle m_nh;
    std::shared_ptr<image_transport::ImageTransport> m_imageTransport;
    std::map<std::string, std::shared_ptr<image_transport::CameraPublisher>> m_cameraPubMap;
    std::shared_ptr<ros::Publisher> m_imuPub;
    std::shared_ptr<ImuInterpolation> imuInterpolation;
};