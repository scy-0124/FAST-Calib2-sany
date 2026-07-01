#ifndef VEHICLE_CONFIG_READER_HPP
#define VEHICLE_CONFIG_READER_HPP

#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "common_lib.h"

namespace vehicle_config_detail
{
inline std::string toLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}
}  // namespace vehicle_config_detail

// 从 vehicle_config 的 camera.yaml 里按 camera_name 读取内参 + 畸变模型，写入 params。
// 成功返回 true；文件打不开/不是序列/找不到 camera_name/缺 camera_model_param 都返回 false
// 并打印具体原因（找不到 camera_name 时列出文件里实际有哪些 camera_name）。
inline bool loadCameraFromVehicleConfig(const std::string &vehicle_config_dir,
                                        const std::string &camera_name, Params &params)
{
    std::string yaml_path = vehicle_config_dir;
    while (!yaml_path.empty() && yaml_path.back() == '/') yaml_path.pop_back();
    yaml_path += "/camera.yaml";

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(yaml_path);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("[VehicleConfig] Failed to open " << yaml_path << ": " << e.what());
        return false;
    }

    if (!root.IsSequence())
    {
        ROS_ERROR_STREAM("[VehicleConfig] " << yaml_path << " is not a YAML sequence.");
        return false;
    }

    std::vector<std::string> available_names;
    for (const auto &cam : root)
    {
        if (!cam["camera_name"]) continue;
        const std::string name = cam["camera_name"].as<std::string>();
        available_names.push_back(name);
        if (name != camera_name) continue;

        if (!cam["camera_model_param"])
        {
            ROS_ERROR_STREAM("[VehicleConfig] camera '" << camera_name << "' has no camera_model_param.");
            return false;
        }
        const YAML::Node &mp = cam["camera_model_param"];
        params.fx = mp["fx"].as<double>();
        params.fy = mp["fy"].as<double>();
        params.cx = mp["cx"].as<double>();
        params.cy = mp["cy"].as<double>();

        params.camera_model = cam["model_type"] ? cam["model_type"].as<std::string>() : std::string("pinhole");
        params.image_width  = cam["width"]  ? cam["width"].as<int>()  : 0;
        params.image_height = cam["height"] ? cam["height"].as<int>() : 0;

        const std::string model_lower = vehicle_config_detail::toLower(params.camera_model);
        params.dist_coeffs.clear();
        if (model_lower == "fisheye" || model_lower == "equidistant")
        {
            // 鱼眼/等距模型：4 系数 [k1,k2,k3,k4]，供 cv::fisheye::* 用
            params.dist_coeffs = {
                mp["k1"] ? mp["k1"].as<double>() : 0.0,
                mp["k2"] ? mp["k2"].as<double>() : 0.0,
                mp["k3"] ? mp["k3"].as<double>() : 0.0,
                mp["k4"] ? mp["k4"].as<double>() : 0.0
            };
        }
        else
        {
            // pinhole/rational: 只取 k1,k2,p1,p2 喂给现有 cv::undistort 路径。k3~k6 不参与
            // 计算——这是沿用去 ROS 化之前既有的 4 系数畸变模型限制，不在本次改造范围内修正。
            params.k1 = mp["k1"] ? mp["k1"].as<double>() : 0.0;
            params.k2 = mp["k2"] ? mp["k2"].as<double>() : 0.0;
            params.p1 = mp["p1"] ? mp["p1"].as<double>() : 0.0;
            params.p2 = mp["p2"] ? mp["p2"].as<double>() : 0.0;
        }
        return true;
    }

    ROS_ERROR_STREAM("[VehicleConfig] camera_name '" << camera_name << "' not found in " << yaml_path);
    for (const auto &n : available_names)
    {
        std::cerr << "  available camera_name: " << n << std::endl;
    }
    return false;
}

// fisheye/equidistant 相机：用原始 K（不重估）+ D + R=I 把图像 undistort 成 pinhole 等效图，
// 之后清空畸变系数、camera_model 标记为 pinhole。非鱼眼族直接返回 false，不做任何处理。
// img_size 用实际加载图像尺寸，不用 camera.yaml 里的 width/height 元数据。
inline bool applyFisheyeUndistortIfNeeded(Params &params, cv::Mat &image)
{
    const std::string model_lower = vehicle_config_detail::toLower(params.camera_model);
    if (model_lower != "fisheye" && model_lower != "equidistant") return false;

    if (image.empty())
    {
        ROS_WARN("[Fisheye] image empty, skip undistort.");
        return false;
    }
    if (params.dist_coeffs.size() < 4)
    {
        ROS_WARN("[Fisheye] dist_coeffs has %zu (<4) values, skip undistort.", params.dist_coeffs.size());
        return false;
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << params.fx, 0, params.cx,
                                            0, params.fy, params.cy,
                                            0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(4, 1) << params.dist_coeffs[0], params.dist_coeffs[1],
                                            params.dist_coeffs[2], params.dist_coeffs[3]);

    const cv::Size img_size(image.cols, image.rows);
    cv::Mat mapx, mapy;
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F), K, img_size, CV_16SC2, mapx, mapy);

    cv::Mat undistorted;
    cv::remap(image, undistorted, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    ROS_INFO("[Fisheye] model='%s' undistorted to pinhole equivalent, size=%dx%d",
             params.camera_model.c_str(), undistorted.cols, undistorted.rows);

    params.k1 = params.k2 = params.p1 = params.p2 = 0.0;
    params.dist_coeffs.clear();
    params.camera_model = "pinhole";
    image = undistorted;
    return true;
}

#endif  // VEHICLE_CONFIG_READER_HPP
