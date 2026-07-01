/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

// =============================================================================
// data_preprocess.hpp —— 图像 + 点云的加载
// -----------------------------------------------------------------------------
// 图像（cv::imread）和点云（pcd_io.hpp::loadPointCloudInput）各自独立加载，
// 任一失败只打印错误、不影响另一个字段的正常返回，调用方需要分别判空。
// =============================================================================

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <opencv2/opencv.hpp>
#include "common_lib.h"
#include "pcd_io.hpp"

class DataPreprocess
{
public:
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(const std::string &image_path, const std::string &pointcloud_path,
                   const std::string &lidar_type_override)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            ROS_ERROR_STREAM("[DataPreprocess] Loading the image " << image_path << " failed");
        }

        PointCloudLoadResult result;
        if (!loadPointCloudInput(pointcloud_path, lidar_type_override, result))
        {
            ROS_ERROR_STREAM("[DataPreprocess] Loading the point cloud " << pointcloud_path << " failed");
            return;
        }
        cloud_input_ = result.cloud;
        lidar_type_ = result.type;
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif  // DATA_PREPROCESS_HPP