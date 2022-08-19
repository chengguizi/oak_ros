#pragma once

#include <thread>

#include <spdlog/spdlog.h>

#include <depthai/depthai.hpp>

#include <oak_ros/OakRosInterface.hpp>

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"
#include "stereo_msgs/DisparityImage.h"

#include <Eigen/Eigen>

class OakPointCloudConverter;

#include <set>

class ImuInterpolation {
  public:
    ImuInterpolation(int imuFrequency) : m_imuFrequency(imuFrequency) {};
    std::vector<sensor_msgs::Imu> updatePacket(const dai::IMUReportAccelerometer &accel,
                                                 const dai::IMUReportGyroscope &gyro);

  private:
    int m_imuFrequency;
    double m_lastGyroTs = -1;
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

struct GenericCamera {
    dai::CameraBoardSocket socket;
    std::string name;
    std::string sensorModel;
    bool monochrome;
    bool partOfStereoPair;

    std::shared_ptr<dai::node::MonoCamera> monoCamera;
    std::shared_ptr<dai::node::ColorCamera> colorCamera;

    dai::MonoCameraProperties::SensorResolution monoResolution;
    dai::ColorCameraProperties::SensorResolution colorResolution;

    GenericCamera() = delete;

    GenericCamera(const dai::CameraBoardSocket& s) : socket(s) {}
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

    int restartCount = 0;

    double lastGyroTs; // use only in non-interpolation mode of imu
    std::map<std::string, unsigned int> m_lastDispSeqMap;
    std::string m_masterCamera; // store the name of the master triggering camera

    std::string m_device_id;
    std::string m_topic_name;

    dai::Pipeline m_pipeline;
    dai::CalibrationHandler m_calibData;

    std::shared_ptr<dai::Device> m_device;
    std::shared_ptr<dai::DataOutputQueue> m_imuQueue;
    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> m_cameraQueueMap;
    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> m_depthQueueMap, m_disparityQueueMap, m_depthMonoQueueMap;

    std::shared_ptr<dai::DataInputQueue> m_controlQueue;

    std::shared_ptr<dai::node::IMU> m_imu; // assume there is only one imu sensor in the system
    std::map<std::string, GenericCamera> m_cameraMap;

    // this is a constant mapping from the string to the corresponding socket enum
    std::map<std::string, dai::CameraBoardSocket> m_socketMapping = {
        {"rgb", dai::CameraBoardSocket::RGB},
        {"left", dai::CameraBoardSocket::LEFT},
        {"right", dai::CameraBoardSocket::RIGHT},
        {"camd", dai::CameraBoardSocket::CAM_D}};

    // map of pairs of stereo
    std::map<std::string, std::shared_ptr<dai::node::StereoDepth>> m_stereoDepthMap;
    std::map<std::string, std::shared_ptr<dai::node::ImageManip>> m_imageManipMap;

    std::map<dai::CameraBoardSocket, std::string> m_connectedSensorModels;

    double m_averageCameraSyncLatency;
    double m_averageImuLatency;
    std::map<std::string, double> m_averageStereoLatency;

    std::thread m_run, m_watchdog;
    void run();

    // the fuctions below is to setup pipeline before m_device creation
    std::shared_ptr<dai::node::XLinkIn> configureControl();
    void configureImu();
    void configureCameras();
    void configureStereos();
    void configureCamera(std::string cameraName);
    void configureMonoCamera(GenericCamera& genericCamera);
    void configureColorCamera(GenericCamera& genericCamera);

    // the functions below assumes m_device is properly setup
    void setupControlQueue(std::shared_ptr<dai::node::XLinkIn>);
    void setupImuQueue();

    void configureDepthNode(std::shared_ptr<dai::node::StereoDepth> stereoDepth,
                            std::shared_ptr<dai::node::MonoCamera> left,
                            std::shared_ptr<dai::node::MonoCamera> right,
                            std::shared_ptr<dai::node::XLinkOut> xoutDepth);

    void depthCallback(std::shared_ptr<dai::ADatatype> data, std::string name);
    void disparityCallback(std::shared_ptr<dai::ADatatype> data, std::string name);
    void imuCallback(std::shared_ptr<dai::ADatatype> data);

    void publishStaticTransform(Eigen::Isometry3f T, std::string parentId, std::string childId);

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

    // for disparity, depth and point cloud
    std::map<std::string, std::shared_ptr<ros::Publisher>> m_disparityPubMap, m_cloudPubFromDispMap;
    std::map<std::string, sensor_msgs::PointCloud2::Ptr> m_cloudMsgFromDispMap;
    std::map<std::string, stereo_msgs::DisparityImage::Ptr> m_outDispImageMsgMap;

    std::map<std::string, cv::Mat_<float>> m_newMMap;
    std::map<std::string, cv::Mat_<uint8_t>> m_maskRightMap;
    std::map<std::string, std::shared_ptr<OakPointCloudConverter>> m_disparity2PointCloudConverterMap;

    // store a short history of the right frames (one of the stereo pair cameras) for monod / rgbd pointcloud output
    std::map<std::string, std::queue<std::shared_ptr<dai::ImgFrame>>> m_rightFrameHistoryMap;

    // tf2_ros::TransformBroadcaster m_broadcaster;
    tf2_ros::StaticTransformBroadcaster m_broadcaster;
};