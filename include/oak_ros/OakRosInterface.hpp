#pragma once

#include <memory>
#include <string>
#include <optional>

#include <ros/ros.h>

#include <depthai-shared/properties/MonoCameraProperties.hpp>
#include <depthai-shared/properties/ColorCameraProperties.hpp>
#include <depthai-shared/properties/StereoDepthProperties.hpp>

/**
 * @brief Oak Device parameters to configure output
 * 
 * @param enable_stereo boolean to enable stereo images streaming (/{left,right}/image_rect_raw)
 * @param enable_stereo_rectified boolean to stream rectified stereo, instead of `raw` stereo. if @enable_depth is false, this does nothing
 * @param stereo_resolution optional parameter to specify the preferred stereo resolution
 * @param stereo_fps optional parameter to specify the framerate
 * @param stereo_fps_throttle optional parameter to specify framerate by using on-device script for frame dropping
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
 * @param manual_iso optional parameter to set manual iso, has to be used together with @manual_exposure
 * 
 * @param enable_apriltag boolean to enable apriltag detection streaming
 * 
 * @note
 */

struct OakRosParams
{
    std::string device_id;
    std::string topic_name = "oak";

    bool only_usb2_mode = false;

    bool enable_stereo = true;
    bool enable_stereo_rectified = true;
    std::string enable_mesh_dir; 
    std::optional<dai::MonoCameraProperties::SensorResolution> stereo_resolution = {}; // dai::MonoCameraProperties::SensorResolution::THE_480_P

    std::optional<float> stereo_fps = {};

    bool enable_depth = true;
    bool enable_depth_pointcloud = false;
    // dai::StereoDepthProperties

    bool enable_rgb = false;
    bool enable_camD = false;
    bool hardware_sync = true;
    std::optional<dai::ColorCameraProperties::SensorResolution> rgb_resolution = {}; // dai::ColorCameraProperties::SensorResolution::THE_1080_P;

    bool enable_imu = false;
    int imu_frequency = 100;

    bool align_ts_to_right = true;

    std::optional<int> manual_exposure; // 1 - 33000
    std::optional<int> manual_iso; // 100 - 1600
    int exposure_compensation = 0; // -9 - 9

    int ir_laser_dot = 0; // in mA: 0 - 1200
    int ir_floodlight = 0; // in mA: 0 - 1500

    bool enable_apriltag = false;

    bool debug_opencv_images = false;
};

class OakRosInterface
{
public:
    typedef std::shared_ptr<OakRosInterface> Ptr;

    virtual void init(const ros::NodeHandle &nh, const OakRosParams &params) = 0;

protected:
};