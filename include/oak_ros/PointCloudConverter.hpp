#pragma once

#include <spdlog/spdlog.h>

#include <limits>
#include <type_traits>

#include <sensor_msgs/point_cloud2_iterator.h>

// template<typename T>
class OakPointCloudConverter {
  public:
    OakPointCloudConverter(float fx, float fy, float cu, float cv, float baseline,
                           int depth_decimation_factor = 1)
        : m_fx(fx), m_fy(fy), m_cu(cu), m_cv(cv), m_baseline(baseline),
          m_depth_decimation_factor(depth_decimation_factor) {
        spdlog::info("initialised OakPointCloudConverter with fx = {}, fy = {}, cu = {}, cv = {}, "
                     "baseline = {}, decimation factor = {}",
                     m_fx, m_fy, m_cu, m_cv, m_baseline, m_depth_decimation_factor);
    };

    void setScale(int scale) { m_scale = scale; }

    // depth = focal_length_in_pixels * baseline / disparity_in_pixels
    template <typename T>
    void Disparity2PointCloud(std::shared_ptr<dai::ImgFrame> disparityFrame,
                              std::shared_ptr<dai::ImgFrame> imageFrame,
                              sensor_msgs::PointCloud2::Ptr &cloudMsg, float baseline = 0);

    // template <typename T>
    // void Depth2PointCloud(std::shared_ptr<dai::ImgFrame> depthFrame,
    //                       sensor_msgs::PointCloud2::Ptr &cloudMsg);

  private:
    float m_fx, m_fy, m_cu, m_cv, m_baseline;
    float m_maximum_depth = 10;
    int m_depth_decimation_factor;
    int m_scale = 1;
};

template <typename T>
void OakPointCloudConverter::Disparity2PointCloud(std::shared_ptr<dai::ImgFrame> disparityFrame,
                                                  std::shared_ptr<dai::ImgFrame> imageFrame,
                                                  sensor_msgs::PointCloud2::Ptr &cloudMsg,
                                                  float baseline) {

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloudMsg);

    // https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h

    if (!imageFrame.get()) {
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        float constant_x = m_fx;
        float constant_y = m_fy;
        float bad_point = std::numeric_limits<float>::quiet_NaN();

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloudMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloudMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloudMsg, "z");
        const T *disparity_row = reinterpret_cast<const T *>(&disparityFrame->getData()[0]);
        int row_step = disparityFrame->getWidth();
        for (int v = 0; v < (int)cloudMsg->height; ++v, disparity_row += row_step) {
            for (int u = 0; u < (int)cloudMsg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
                float disparity = (float)disparity_row[u] / m_scale;

                // Missing points denoted by zero
                if (disparity == 0) {
                    *iter_x = *iter_y = *iter_z = bad_point;
                    continue;
                }

                float depth = constant_x * m_baseline / disparity;

                if (depth >= m_maximum_depth) {
                    *iter_x = *iter_y = *iter_z = bad_point;
                    continue;
                }

                // camera is originally in RDF, but we would like to have NWU

                // Fill in XYZ
                *iter_x = depth;
                *iter_y = - (u * m_depth_decimation_factor - m_cu) * depth / constant_x;
                *iter_z = - (v * m_depth_decimation_factor - m_cv) * depth / constant_y;
            }
        }
    } else {

        pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                          sensor_msgs::PointField::FLOAT32, "z", 1,
                                          sensor_msgs::PointField::FLOAT32, "intensity", 1,
                                          sensor_msgs::PointField::FLOAT32);
        
        if (imageFrame->getType() != dai::RawImgFrame::Type::RAW8)
            throw std::runtime_error("expect mono8 type");
        
        float constant_x = m_fx;
        float constant_y = m_fy;
        float bad_point = std::numeric_limits<float>::quiet_NaN();

        // https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/src/nodelets/point_cloud_xyzi.cpp
        // https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloudMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloudMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloudMsg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(*cloudMsg, "intensity");
        const T *disparity_row = reinterpret_cast<const T *>(&disparityFrame->getData()[0]);
        const uint8_t *image_row = reinterpret_cast<const uint8_t *>(&imageFrame->getData()[0]);

        int row_step = disparityFrame->getWidth();
        for (int v = 0; v < (int)cloudMsg->height; ++v, disparity_row += row_step, image_row += imageFrame->getWidth()) {
            for (int u = 0; u < (int)cloudMsg->width; ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
                float disparity = (float)disparity_row[u] / m_scale;
                float intensity = image_row[u];

                // Missing points denoted by zero
                if (disparity == 0) {
                    *iter_x = *iter_y = *iter_z = *iter_i = bad_point;
                    continue;
                }

                float depth = constant_x * m_baseline / disparity;

                if (depth >= m_maximum_depth) {
                    *iter_x = *iter_y = *iter_z = *iter_i = bad_point;
                    continue;
                }

                // Fill in XYZ

                // convert RDF to NWU
                *iter_x = depth;
                *iter_y = - (u * m_depth_decimation_factor - m_cu) * depth / constant_x;
                *iter_z = - (v * m_depth_decimation_factor - m_cv) * depth / constant_y;
                *iter_i = intensity;
            }
        }
    }
}

// template <typename T>
// void OakPointCloudConverter::Depth2PointCloud(std::shared_ptr<dai::ImgFrame> depthFrame,
//                                               sensor_msgs::PointCloud2::Ptr &cloudMsg) {

//     // https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h

//     float constant_x = m_fx;
//     float constant_y = m_fy;
//     float bad_point = std::numeric_limits<float>::quiet_NaN();

//     sensor_msgs::PointCloud2Iterator<float> iter_x(*cloudMsg, "x");
//     sensor_msgs::PointCloud2Iterator<float> iter_y(*cloudMsg, "y");
//     sensor_msgs::PointCloud2Iterator<float> iter_z(*cloudMsg, "z");
//     const T *depth_row = reinterpret_cast<const T *>(&depthFrame->getData()[0]);
//     int row_step = depthFrame->getWidth();
//     for (int v = 0; v < (int)cloudMsg->height; ++v, depth_row += row_step) {
//         for (int u = 0; u < (int)cloudMsg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
//             float depth = (float)depth_row[u] / m_scale;

//             // Missing points denoted by zero
//             if (depth == 0) {
//                 *iter_x = *iter_y = *iter_z = bad_point;
//                 continue;
//             }

//             // if (depth >= m_maximum_depth) {
//             //     *iter_x = *iter_y = *iter_z = bad_point;
//             //     co ntinue;
//             // }

//             // Fill in XYZ
//             *iter_x = (u - m_cu) * depth / constant_x;
//             *iter_y = (v - m_cv) * depth / constant_y;
//             *iter_z = depth;
//         }
//     }
// }