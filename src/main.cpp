#include <oak_ros/OakRosFactory.hpp>

#include <spdlog/spdlog.h>

// Preset good for indoor calibration

OakRosParams getIndoorLightingParams() {
    OakRosParams params;
    params.manual_exposure = 1600; // in usec
    params.manual_iso = 200;       // 100 to 1600

    return params;
}

// preset that good for indoor low light
OakRosParams getLowLightParams() {
    OakRosParams params;
    params.manual_exposure = 8000; // in usec
    params.manual_iso = 1600;      // 100 to 1600

    return params;
}

void configureResolution(OakRosParams &params, const std::string& option_resolution) {

    if (params.enable_camA)
        params.resolutionMap["cama"] = option_resolution;
    if (params.enable_camB)
        params.resolutionMap["camb"] = option_resolution;
    if (params.enable_camC)
        params.resolutionMap["camc"] = option_resolution;
    if (params.enable_camD)
        params.resolutionMap["camd"] = option_resolution;
}

int main(int argc, char **argv) {
    // spdlog::set_level(spdlog::level::debug);

    ros::init(argc, argv, "oak_ros");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    std::string option_tf_prefix;
    int option_frequency;
    std::string option_resolution;
    bool option_enable_stereo_half_resolution_output;
    std::string option_exposure_mode;
    bool option_use_mesh;
    float option_mesh_alpha;
    // bool option_rectified;
    bool option_poe_mode;
    bool option_only_usb2_mode;
    int option_shutter_speed_us;
    int option_iso;
    int option_exposure_compensation;
    int option_ir_laser_dot;
    int option_ir_floodlight;
    int option_imu_frequency;
    bool option_imu_use_raw;
    bool option_imu;
    bool option_cama;
    bool option_camb;
    bool option_camc;
    bool option_camd;
    bool option_enable_stereo_rectified;
    bool option_enable_disparity;
    bool option_enable_depth;
    bool option_enable_pointcloud;
    int option_depth_decimation_factor;
    bool option_hardware_sync;
    bool option_debug_opencv_image;

    nh_local.param<std::string>("tf_prefix", option_tf_prefix, "");
    nh_local.param<int>("frequency", option_frequency, -1);
    nh_local.param<std::string>("resolution", option_resolution, "400");
    nh_local.param<bool>("enable_stereo_half_resolution_output", option_enable_stereo_half_resolution_output, false);
    nh_local.param<bool>("use_mesh", option_use_mesh, false);
    nh_local.param<float>("mesh_alpha", option_mesh_alpha, 0.0);
    // nh_local.param<bool>("rectified", option_rectified, true);
    nh_local.param<std::string>("exposure_mode", option_exposure_mode, "auto");
    nh_local.param<int>("shutter_speed_us", option_shutter_speed_us, 1000);
    nh_local.param<int>("iso", option_iso, 300);
    nh_local.param<int>("exposure_compensation", option_exposure_compensation, 0);
    nh_local.param<int>("ir_laser_dot", option_ir_laser_dot, 0);
    nh_local.param<int>("ir_floodlight", option_ir_floodlight, 0);
    nh_local.param<int>("imu_frequency", option_imu_frequency, 200);
    nh_local.param<bool>("imu_use_raw", option_imu_use_raw, false);
    nh_local.param<bool>("poe_mode", option_poe_mode, false);
    nh_local.param<bool>("only_usb2_mode", option_only_usb2_mode, false);
    nh_local.param<bool>("imu", option_imu, true);
    nh_local.param<bool>("cama", option_cama, true);
    nh_local.param<bool>("camb", option_camb, true);
    nh_local.param<bool>("camc", option_camc, false);
    nh_local.param<bool>("camd", option_camd, false);
    nh_local.param<bool>("enable_stereo_rectified", option_enable_stereo_rectified, false);
    nh_local.param<bool>("enable_disparity", option_enable_disparity, false);
    nh_local.param<bool>("enable_depth", option_enable_depth, false);
    nh_local.param<bool>("enable_pointcloud", option_enable_pointcloud, false);
    nh_local.param<int>("depth_decimation_factor", option_depth_decimation_factor, 1);
    nh_local.param<bool>("hardware_sync", option_hardware_sync, true);
    nh_local.param<bool>("debug_opencv_image", option_debug_opencv_image, true);

    auto device_ids = OakRosFactory::getAllAvailableDeviceIds();

    std::vector<OakRosInterface::Ptr> oak_handlers;

    size_t topic_name_seq = 1;
    for (auto &id : device_ids) {
        bool isPoeDevice = (id.compare(0, 4, "192.") == 0);

        if (option_poe_mode) {
            if (!isPoeDevice) {
                spdlog::info("{} is not a POE device, skipping", id);
                continue;
            }
        } else {
            if (isPoeDevice) {
                spdlog::info("{} is a POE device, skipping", id);
                continue;
            }
        }
        spdlog::info("main: start device with id {}", id);

        OakRosInterface::Ptr handler = oak_handlers.emplace_back(OakRosFactory::getOakRosHandler());

        OakRosParams params;

        // decide what params to use based on command-line inputs
        constexpr unsigned int FULL_FPS = 30;
        {
            if (option_exposure_mode == "auto")
                ;
            else if (option_exposure_mode == "low-light")
                params = getLowLightParams();
            else if (option_exposure_mode == "indoor")
                params = getIndoorLightingParams();
            else if (option_exposure_mode == "manual") {
                params.manual_exposure = option_shutter_speed_us; // in usec
                params.manual_iso = option_iso;                   // 100 to 1600
            } else if (option_exposure_mode == "calibration") {
                params = getIndoorLightingParams();

                option_frequency = 4;

            } else {
                spdlog::warn("invalid mode {}", option_exposure_mode);
                return -1;
            }

            if (option_frequency > 0) {
                params.all_cameras_fps = option_frequency;
            }
        }

        params.enable_stereo_half_resolution_output = option_enable_stereo_half_resolution_output;

        params.tf_prefix = option_tf_prefix;

        params.exposure_compensation = option_exposure_compensation;
        params.ir_laser_dot = option_ir_laser_dot;
        params.ir_floodlight = option_ir_floodlight;

        params.only_usb2_mode = option_only_usb2_mode;

        params.enable_stereo_rectified = option_enable_stereo_rectified;
        params.enable_disparity = option_enable_disparity;
        params.enable_depth = option_enable_depth;
        params.enable_pointcloud = option_enable_pointcloud;
        params.depth_decimation_factor = option_depth_decimation_factor;

        params.use_mesh = option_use_mesh;
        params.mesh_alpha = option_mesh_alpha;

        params.device_id = id;
        params.topic_name = "oak" + std::to_string(topic_name_seq);

        // params.enable_mesh_dir = option_mesh_dir;

        params.enable_camA = option_cama;
        params.enable_camB = option_camb;
        params.enable_camC = option_camc;
        params.enable_camD = option_camd;

        params.enable_imu = option_imu;
        params.imu_frequency = option_imu_frequency;
        params.imu_use_raw = option_imu_use_raw;

        params.hardware_sync = option_hardware_sync;

        params.debug_opencv_images = option_debug_opencv_image;

        if (params.enable_stereo_rectified || params.enable_disparity || params.enable_depth || params.enable_pointcloud)
            params.enabled_stereo_pairs = {{"rgb_left", {"rgb", "left"}}};
        else
            params.enabled_stereo_pairs = {};

        configureResolution(params, option_resolution);

        handler->init(nh_local, params);

        topic_name_seq++;
    }

    ros::spin();

    spdlog::info("main exits cleanly");
}