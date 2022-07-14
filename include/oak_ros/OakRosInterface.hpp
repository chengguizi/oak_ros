#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <ros/ros.h>

#include <depthai-shared/properties/ColorCameraProperties.hpp>
#include <depthai-shared/properties/MonoCameraProperties.hpp>
#include <depthai-shared/properties/StereoDepthProperties.hpp>

/**
 * @brief Oak Device parameters to configure output
 *
 *
 * @param enable_depth boolean to enable depth stream (TODO: not yet implemented)
 * @param enable_disparity boolean to enable disparity stream
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
 *
 * @note
 */

struct OakRosParams {
    std::string device_id;
    std::string topic_name = "oak";
    std::string tf_prefix;

    bool only_usb2_mode = false;

    std::optional<float> all_cameras_fps = {};

    // configuration of individual cameras
    bool enable_rgb = false;
    bool enable_left = false;
    bool enable_right = false;
    bool enable_camD = false;
    bool hardware_sync = true;

    std::map<std::string, dai::MonoCameraProperties::SensorResolution>
        resolutionMap; // dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    bool enable_stereo_half_resolution_output = true; // only effective when at 800P

    bool use_mesh = false;
    float mesh_alpha = 0; // range [0, 1], 0 means no black boarder, 1 means no pixel losses

    // configuration for stereo

    bool enable_stereo_rectified = false;
    bool enable_depth = false;
    bool enable_disparity = false;
    bool enable_pointcloud = false;
    int depth_decimation_factor = 1;

    // define pairs of stereo: rgb, left, right, camd
    std::map<std::string, std::pair<std::string, std::string>> enabled_stereo_pairs = {{"left_rgb", {"left", "rgb"}}};

    bool enable_imu = false;
    int imu_frequency = 100;
    bool imu_use_raw = false; // whether to use raw or filtered readings from IMU sensor

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