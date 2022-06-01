#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>

#include <ros/ros.h>

#include <depthai-shared/properties/ColorCameraProperties.hpp>
#include <depthai-shared/properties/MonoCameraProperties.hpp>
#include <depthai-shared/properties/StereoDepthProperties.hpp>

/**
 * @brief Oak Device parameters to configure output
 *
 *
 * @param enable_depth boolean to enable depth stream (TODO: not yet implemented)
 * @param enable_depth_pointcloud TODO: not yet implemented
 *
 * @param enable_rgb boolean to enable rgb stream (TODO: not yet implemented)
 * @param rgb_resolution optional parameter to specify the preferred rgb resolution
 * @param enable_imu boolean to enable imu stream
 * @param imu_frequency the frequency IMU should sample upon
 *
 * @param align_ts_to_right boolean to enable copy the exact ts of right camera to the left camera
 *
 * @param manual_exposure optional parameter to set manual exposure
 * @param manual_iso optional parameter to set manual iso, has to be used together with
 * @manual_exposure
 *
 * @param enable_apriltag boolean to enable apriltag detection streaming
 *
 * @note
 */

struct OakRosParams {
    std::string device_id;
    std::string topic_name = "oak";

    bool only_usb2_mode = false;

    std::optional<float> all_cameras_fps = {};

    bool enable_depth_left_right = false;
    bool enable_depth_rgb_camd = false;
    bool enable_depth_pointcloud = false;
    // dai::StereoDepthProperties

    bool enable_rgb = false;
    bool enable_left = false;
    bool enable_right = false;
    bool enable_camD = false;
    bool hardware_sync = true;

    std::map<std::string, dai::MonoCameraProperties::SensorResolution>
        resolutionMap; // dai::ColorCameraProperties::SensorResolution::THE_1080_P;

    bool enable_imu = false;
    int imu_frequency = 100;

    std::optional<int> manual_exposure; // 1 - 33000
    std::optional<int> manual_iso;      // 100 - 1600
    int exposure_compensation = 0;      // -9 - 9

    int ir_laser_dot = 0;  // in mA: 0 - 1200
    int ir_floodlight = 0; // in mA: 0 - 1500

    bool debug_opencv_images = false;
};

class OakRosInterface {
  public:
    typedef std::shared_ptr<OakRosInterface> Ptr;

    virtual void init(const ros::NodeHandle &nh, const OakRosParams &params) = 0;

  protected:
};