/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

LiDAR-only batch test entry for target annulus center extraction.
*/

#include "lidar_detect.hpp"
#include "pcd_io.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

namespace
{
// 将 LiDAR 类型枚举转换为日志可读字符串
std::string lidarTypeName(LiDARType type)
{
    switch (type)
    {
        case LiDARType::Solid: return "solid";
        case LiDARType::Mech: return "mech";
        default: return "unknown";
    }
}

// 去掉路径末尾多余的斜杠
std::string trimTrailingSlash(std::string path)
{
    while (!path.empty() && path.back() == '/')
    {
        path.pop_back();
    }
    return path;
}

// 获取路径最后一级文件名或目录名
std::string pathBaseName(const std::string& path)
{
    const std::string clean_path = trimTrailingSlash(path);
    const size_t pos = clean_path.find_last_of('/');
    return pos == std::string::npos ? clean_path : clean_path.substr(pos + 1);
}

// 获取路径父目录的最后一级名称
std::string pathParentName(const std::string& path)
{
    const std::string clean_path = trimTrailingSlash(path);
    const size_t last_slash = clean_path.find_last_of('/');
    if (last_slash == std::string::npos) return "";
    return pathBaseName(clean_path.substr(0, last_slash));
}

// 去掉文件名扩展名
std::string stripExtension(const std::string& filename)
{
    const size_t pos = filename.find_last_of('.');
    return pos == std::string::npos ? filename : filename.substr(0, pos);
}

// 将任意字符串转换为安全的文件名前缀片段
std::string sanitizeFilePart(std::string value)
{
    for (char& c : value)
    {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_'))
        {
            c = '_';
        }
    }
    return value;
}

// 解析测试输出目录：CLI 必须显式传 -o，这里只需要去掉尾部斜杠。
// 目录本身的创建/校验已经在 main() 里检测开始前用 common_lib.h::ensureOutputDirectory() 做过了，
// 这里不再重复创建。
std::string resolveOutputDirectory(const Params& params)
{
    return trimTrailingSlash(params.output_path);
}

// 根据点云路径（单个 .pcd 文件或文件夹）所在目录和文件名生成输出文件前缀
std::string outputPrefixForPointcloud(const std::string& pointcloud_path)
{
    return sanitizeFilePart(pathParentName(pointcloud_path) + "_" +
                            stripExtension(pathBaseName(pointcloud_path)));
}

// 构造带 RGB 颜色的 PCL 点
pcl::PointXYZRGB makeRgbPoint(float x, float y, float z, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.r = r;
    p.g = g;
    p.b = b;
    return p;
}

// 在线性颜色表中按比例插值
std::array<std::uint8_t, 3> lerpColor(const std::array<std::uint8_t, 3>& a,
                                      const std::array<std::uint8_t, 3>& b,
                                      float t)
{
    t = std::max(0.0f, std::min(1.0f, t));
    return {{
        static_cast<std::uint8_t>(std::round(a[0] + t * (b[0] - a[0]))),
        static_cast<std::uint8_t>(std::round(a[1] + t * (b[1] - a[1]))),
        static_cast<std::uint8_t>(std::round(a[2] + t * (b[2] - a[2])))
    }};
}

// 计算浮点数组的指定分位数
float percentile(std::vector<float> values, float ratio)
{
    if (values.empty()) return 0.0f;
    ratio = std::max(0.0f, std::min(1.0f, ratio));
    const size_t idx = static_cast<size_t>(std::round(ratio * static_cast<float>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + idx, values.end());
    return values[idx];
}

// 将 intensity 映射为便于观察的伪彩色
std::array<std::uint8_t, 3> intensityColor(float intensity, float min_intensity, float max_intensity)
{
    float t = 0.0f;
    if (max_intensity > min_intensity + 1e-3f)
    {
        t = (intensity - min_intensity) / (max_intensity - min_intensity);
    }
    t = std::max(0.0f, std::min(1.0f, t));
    t = std::pow(t, 0.75f);

    const std::array<std::uint8_t, 3> low = {{30, 38, 55}};
    const std::array<std::uint8_t, 3> mid = {{70, 135, 190}};
    const std::array<std::uint8_t, 3> high = {{245, 220, 145}};
    if (t < 0.55f)
    {
        return lerpColor(low, mid, t / 0.55f);
    }
    return lerpColor(mid, high, (t - 0.55f) / 0.45f);
}

// 在调试点云中生成实心球标记
void addSphereMarker(const pcl::PointXYZ& center,
                     float radius,
                     float step,
                     const std::array<std::uint8_t, 3>& color,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    for (float dx = -radius; dx <= radius; dx += step)
    {
        for (float dy = -radius; dy <= radius; dy += step)
        {
            for (float dz = -radius; dz <= radius; dz += step)
            {
                if (dx * dx + dy * dy + dz * dz <= radius * radius)
                {
                    output->push_back(makeRgbPoint(center.x + dx, center.y + dy, center.z + dz,
                                                   color[0], color[1], color[2]));
                }
            }
        }
    }
}

// 用纯白小球标记最终圆心
void addCenterMarker(const pcl::PointXYZ& center,
                     const std::array<std::uint8_t, 3>& color,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    addSphereMarker(center, 0.040f, 0.006f, color, output);
}

// 通用落盘：把某一条流水线中间点云原样存成 pcd，空点云直接跳过（不当作错误）
template <typename PointT>
bool savePipelineCloud(const std::string& dir, const std::string& name,
                       const typename pcl::PointCloud<PointT>::Ptr& cloud)
{
    if (!cloud || cloud->empty()) return false;
    const std::string path = dir + "/" + name;
    try
    {
        if (pcl::io::savePCDFileBinaryCompressed(path, *cloud) != 0)
        {
            ROS_ERROR_STREAM("[LiDAR Test] Failed to save pipeline stage: " << path);
            return false;
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[LiDAR Test] Failed to save pipeline stage: " << path << " (" << e.what() << ")");
        return false;
    }
    std::cout << "[LiDAR Test] Saved pipeline stage: " << path
              << " (" << cloud->size() << " points)" << std::endl;
    return true;
}

// 在 (cx, cy, z) 处生成一圈拟合圆环采样点，用于圆拟合结果可视化
void addCircleRing(double cx, double cy, double radius, float z,
                   const std::array<std::uint8_t, 3>& color,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
                   int num_points = 240)
{
    if (radius <= 0.0) return;
    for (int i = 0; i < num_points; ++i)
    {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
        output->push_back(makeRgbPoint(static_cast<float>(cx + radius * std::cos(theta)),
                                       static_cast<float>(cy + radius * std::sin(theta)),
                                       z, color[0], color[1], color[2]));
    }
}

// 落盘每个候选聚类的原始点（对齐坐标系），按结局着色：绿=被采纳为候选圆心，
// 黄=圆拟合成功但被半径/误差/去重等门限剔除，红=圆拟合本身没收敛
void saveClusterClouds(const std::vector<LidarDetect::ClusterFitDebug>& records, const std::string& dir)
{
    for (size_t i = 0; i < records.size(); ++i)
    {
        const auto& rec = records[i];
        if (!rec.points || rec.points->empty()) continue;

        std::array<std::uint8_t, 3> color;
        const char* status = "fit_failed";
        if (rec.accepted) { color = {{40, 220, 40}}; status = "accepted"; }
        else if (rec.fit_ok) { color = {{230, 200, 40}}; status = "fit_ok_rejected"; }
        else { color = {{220, 40, 40}}; }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored->reserve(rec.points->size());
        for (const auto& p : *rec.points)
        {
            colored->push_back(makeRgbPoint(p.x, p.y, p.z, color[0], color[1], color[2]));
        }
        colored->width = static_cast<uint32_t>(colored->size());
        colored->height = 1;
        colored->is_dense = false;

        char name[64];
        std::snprintf(name, sizeof(name), "07_cluster_%02zu.pcd", i);
        const std::string path = dir + "/" + name;
        try
        {
            if (pcl::io::savePCDFileBinaryCompressed(path, *colored) == 0)
            {
                std::cout << "[LiDAR Test] Saved pipeline stage: " << path
                          << " (" << colored->size() << " points, " << status << ")" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("[LiDAR Test] Failed to save pipeline stage: " << path << " (" << e.what() << ")");
        }
    }
}

// 落盘每个圆拟合成功的聚类可视化：原始点(灰) + 拟合圆环(青=内圈/单圆, 品红=外圈) + 圆心标记
// (白=被采纳, 橙=被剔除)。圆拟合本身失败的聚类没有圆可画，跳过。
void saveCircleFitVisualizations(const std::vector<LidarDetect::ClusterFitDebug>& records, const std::string& dir)
{
    for (size_t i = 0; i < records.size(); ++i)
    {
        const auto& rec = records[i];
        if (!rec.fit_ok || !rec.points || rec.points->empty()) continue;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr viz(new pcl::PointCloud<pcl::PointXYZRGB>);
        const std::array<std::uint8_t, 3> point_color = {{120, 120, 130}};
        viz->reserve(rec.points->size() + 500);
        for (const auto& p : *rec.points)
        {
            viz->push_back(makeRgbPoint(p.x, p.y, p.z, point_color[0], point_color[1], point_color[2]));
        }

        const std::array<std::uint8_t, 3> inner_color = {{40, 220, 220}};
        const std::array<std::uint8_t, 3> outer_color = {{220, 40, 220}};
        if (rec.inner_radius > 0.0 && std::fabs(rec.outer_radius - rec.inner_radius) > 1e-6)
        {
            addCircleRing(rec.center_x, rec.center_y, rec.inner_radius, 0.0f, inner_color, viz);
            addCircleRing(rec.center_x, rec.center_y, rec.outer_radius, 0.0f, outer_color, viz);
        }
        else
        {
            addCircleRing(rec.center_x, rec.center_y, rec.outer_radius, 0.0f, inner_color, viz);
        }

        const std::array<std::uint8_t, 3> center_color =
            rec.accepted ? std::array<std::uint8_t, 3>{{255, 255, 255}}
                        : std::array<std::uint8_t, 3>{{255, 140, 0}};
        addSphereMarker(pcl::PointXYZ(static_cast<float>(rec.center_x), static_cast<float>(rec.center_y), 0.0f),
                        0.020f, 0.004f, center_color, viz);

        viz->width = static_cast<uint32_t>(viz->size());
        viz->height = 1;
        viz->is_dense = false;

        char name[64];
        std::snprintf(name, sizeof(name), "08_circle_fit_%02zu.pcd", i);
        const std::string path = dir + "/" + name;
        try
        {
            if (pcl::io::savePCDFileBinaryCompressed(path, *viz) == 0)
            {
                std::cout << "[LiDAR Test] Saved pipeline stage: " << path << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("[LiDAR Test] Failed to save pipeline stage: " << path << " (" << e.what() << ")");
        }
    }
}

// 无论最终检测成功与否，都把完整流水线的中间点云落盘，命名按 00_/01_/... 表示产生的先后顺序，
// 方便定位到底是哪一步丢的点。落盘路径复用现有 -o 输出目录，按点云前缀单独建一个子目录归档，
// 不新增 CLI 参数。聚类/圆拟合数据依赖 LidarDetect::getLastClusterFitRecords()，取的是最终
// 实际生效（胜出）那一次尝试的记录，不是每次内部重试都留痕。
void savePipelineStages(const pcl::PointCloud<Common::Point>::Ptr& raw_cloud,
                        const LidarDetect& lidar_detect,
                        const Params& params,
                        const std::string& pointcloud_path)
{
    const std::string output_dir = resolveOutputDirectory(params);
    const std::string dir = output_dir + "/" + outputPrefixForPointcloud(pointcloud_path) + "_pipeline";
    if (!ensureOutputDirectory(dir))
    {
        return;
    }

    savePipelineCloud<Common::Point>(dir, "00_raw_cloud.pcd", raw_cloud);
    savePipelineCloud<Common::Point>(dir, "01_roi_filtered.pcd", lidar_detect.getFilteredCloud());
    savePipelineCloud<Common::Point>(dir, "02_plane_cloud.pcd", lidar_detect.getPlaneCloud());
    savePipelineCloud<pcl::PointXYZ>(dir, "03_aligned_plane.pcd", lidar_detect.getAlignedCloud());
    savePipelineCloud<Common::Point>(dir, "04_annulus_original.pcd", lidar_detect.getAnnulusOriginalCloud());
    savePipelineCloud<pcl::PointXYZ>(dir, "05_boundary_original.pcd", lidar_detect.getBoundaryOriginalCloud());
    savePipelineCloud<pcl::PointXYZ>(dir, "06_edge_aligned.pcd", lidar_detect.getEdgeCloud());

    const auto& records = lidar_detect.getLastClusterFitRecords();
    saveClusterClouds(records, dir);
    saveCircleFitVisualizations(records, dir);

    savePipelineCloud<pcl::PointXYZ>(dir, "09_center_z0.pcd", lidar_detect.getCenterZ0Cloud());

    std::cout << "[LiDAR Test] Pipeline stage dump directory: " << dir
              << " (" << records.size() << " candidate clusters)" << std::endl;
}

// 保存调试 PCD：板子按 intensity 着色，annulus 为绿色，边界为红色，圆心为白色
bool saveDebugCloud(const pcl::PointCloud<Common::Point>::Ptr& board_cloud,
                    const pcl::PointCloud<Common::Point>::Ptr& annulus_cloud,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& boundary_cloud,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& centers,
                    const Params& params,
                    const std::string& bag_path)
{
    const std::string output_dir = resolveOutputDirectory(params);
    const std::string prefix = outputPrefixForPointcloud(bag_path);
    const std::string output_path = output_dir + "/" + prefix + "_debug_cloud.pcd";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    const size_t board_size = board_cloud ? board_cloud->size() : 0;
    const size_t annulus_size = annulus_cloud ? annulus_cloud->size() : 0;
    const size_t boundary_size = boundary_cloud ? boundary_cloud->size() : 0;
    const size_t board_stride = std::max<size_t>(1, board_size / 220000);
    const size_t visual_board_size = board_stride > 0 ? (board_size + board_stride - 1) / board_stride : board_size;
    debug_cloud->reserve(visual_board_size + annulus_size + boundary_size + 6000);

    std::vector<float> board_intensities;
    board_intensities.reserve(std::min<size_t>(board_size, 300000));
    if (board_cloud)
    {
        for (size_t i = 0; i < board_cloud->size(); i += board_stride)
        {
            const auto& p = board_cloud->points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            if (!std::isfinite(p.intensity)) continue;
            board_intensities.push_back(p.intensity);
        }
    }

    float min_intensity = percentile(board_intensities, 0.02f);
    float max_intensity = percentile(board_intensities, 0.98f);
    if (!std::isfinite(min_intensity) || !std::isfinite(max_intensity) ||
        max_intensity <= min_intensity + 1e-3f)
    {
        min_intensity = 0.0f;
        max_intensity = 255.0f;
    }

    size_t board_output_count = 0;
    if (board_cloud)
    {
        for (size_t i = 0; i < board_cloud->size(); i += board_stride)
        {
            const auto& p = board_cloud->points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                !std::isfinite(p.intensity))
            {
                continue;
            }
            const auto color = intensityColor(p.intensity, min_intensity, max_intensity);
            debug_cloud->push_back(makeRgbPoint(p.x, p.y, p.z, color[0], color[1], color[2]));
            ++board_output_count;
        }
    }

    const std::array<std::uint8_t, 3> annulus_color = {{40, 255, 40}};
    if (annulus_cloud)
    {
        for (const auto& p : *annulus_cloud)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            {
                continue;
            }
            debug_cloud->push_back(makeRgbPoint(p.x, p.y, p.z,
                                                annulus_color[0],
                                                annulus_color[1],
                                                annulus_color[2]));
        }
    }

    const std::array<std::uint8_t, 3> boundary_color = {{255, 35, 35}};
    if (boundary_cloud)
    {
        for (const auto& p : *boundary_cloud)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            {
                continue;
            }
            debug_cloud->push_back(makeRgbPoint(p.x, p.y, p.z,
                                                boundary_color[0],
                                                boundary_color[1],
                                                boundary_color[2]));
        }
    }

    const std::array<std::uint8_t, 3> center_color = {{255, 255, 255}};

    for (size_t i = 0; i < centers->size(); ++i)
    {
        addCenterMarker(centers->points[i], center_color, debug_cloud);
    }

    debug_cloud->width = static_cast<uint32_t>(debug_cloud->size());
    debug_cloud->height = 1;
    debug_cloud->is_dense = false;

    try
    {
        if (pcl::io::savePCDFileBinaryCompressed(output_path, *debug_cloud) != 0)
        {
            ROS_ERROR_STREAM("[LiDAR Test] Failed to save debug cloud to " << output_path);
            return false;
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[LiDAR Test] Failed to save debug cloud to " << output_path << ": " << e.what());
        return false;
    }

    std::cout << "[LiDAR Test] Saved debug cloud: " << output_path
              << " (" << board_output_count << "/" << board_size << " board points, "
              << annulus_size << " green annulus points, "
              << boundary_size << " red boundary points, "
              << centers->size() << " white centers)" << std::endl;
    return true;
}

// 保存最终圆心坐标到 output 文本文件
bool saveCenterCoordinates(const pcl::PointCloud<pcl::PointXYZ>::Ptr& centers,
                           const Params& params,
                           const std::string& bag_path)
{
    const std::string output_dir = resolveOutputDirectory(params);
    const std::string output_path = output_dir + "/" + outputPrefixForPointcloud(bag_path) + "_centers.txt";

    std::ofstream fout(output_path);
    if (!fout.is_open())
    {
        ROS_ERROR_STREAM("[LiDAR Test] Failed to save center coordinates to " << output_path);
        return false;
    }

    fout << std::fixed << std::setprecision(9);
    fout << "# x y z\n";
    for (size_t i = 0; i < centers->size(); ++i)
    {
        const auto& p = centers->points[i];
        fout << i << " " << p.x << " " << p.y << " " << p.z << "\n";
    }

    std::cout << "[LiDAR Test] Saved center coordinates: " << output_path << std::endl;
    return true;
}

// 计算 double 数组的指定分位数
double quantileDouble(std::vector<double> values, double ratio)
{
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    ratio = std::max(0.0, std::min(1.0, ratio));
    const size_t idx = static_cast<size_t>(std::round(ratio * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + idx, values.end());
    return values[idx];
}

// 输出固态 LiDAR 单圆半径质检统计
void printSolidRadiusQuality(const std::array<std::vector<double>, TARGET_NUM_CIRCLES>& radii_by_center,
                             double target_radius)
{
    std::vector<double> radius_errors;
    radius_errors.reserve(TARGET_NUM_CIRCLES);

    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < TARGET_NUM_CIRCLES; ++i)
    {
        const double radius = quantileDouble(radii_by_center[i], 0.5);
        if (!std::isfinite(radius))
        {
            std::cout << "[Radius][LiDAR] circle " << i << ": insufficient support" << std::endl;
            continue;
        }
        const double p10 = quantileDouble(radii_by_center[i], 0.1);
        const double p90 = quantileDouble(radii_by_center[i], 0.9);
        const double error = radius - target_radius;
        radius_errors.push_back(error);
        std::cout << "[Radius][LiDAR] circle " << i
                  << ": support=" << radii_by_center[i].size()
                  << ", radius=" << radius * 1000.0 << " mm"
                  << ", error=" << error * 1000.0 << " mm"
                  << ", p10-p90=[" << p10 * 1000.0 << ", " << p90 * 1000.0 << "] mm"
                  << std::endl;
    }

    double max_abs_error = 0.0;
    double rmse = 0.0;
    for (double error : radius_errors)
    {
        max_abs_error = std::max(max_abs_error, std::fabs(error));
        rmse += error * error;
    }
    if (!radius_errors.empty())
    {
        rmse = std::sqrt(rmse / static_cast<double>(radius_errors.size()));
    }
    std::cout << "[Radius][LiDAR] max radius error = " << max_abs_error * 1000.0
              << " mm, RMSE = " << rmse * 1000.0 << " mm" << std::endl;
}

// 输出机械 LiDAR 内外边界半径和环宽质检统计
void printMechanicalRadiusQuality(
    const std::array<std::vector<double>, TARGET_NUM_CIRCLES>& inner_radii_by_center,
    const std::array<std::vector<double>, TARGET_NUM_CIRCLES>& outer_radii_by_center,
    double target_radius,
    double target_half_width)
{
    std::vector<double> centerline_errors;
    std::vector<double> half_width_errors;
    centerline_errors.reserve(TARGET_NUM_CIRCLES);
    half_width_errors.reserve(TARGET_NUM_CIRCLES);

    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < TARGET_NUM_CIRCLES; ++i)
    {
        const double inner_radius = quantileDouble(inner_radii_by_center[i], 0.5);
        const double outer_radius = quantileDouble(outer_radii_by_center[i], 0.5);
        if (!std::isfinite(inner_radius) || !std::isfinite(outer_radius))
        {
            std::cout << "[Radius][LiDAR] circle " << i << ": insufficient inner/outer support"
                      << " (inner=" << inner_radii_by_center[i].size()
                      << ", outer=" << outer_radii_by_center[i].size() << ")" << std::endl;
            continue;
        }

        const double centerline_radius = 0.5 * (inner_radius + outer_radius);
        const double half_width = 0.5 * (outer_radius - inner_radius);
        const double centerline_error = centerline_radius - target_radius;
        const double half_width_error = half_width - target_half_width;
        centerline_errors.push_back(centerline_error);
        half_width_errors.push_back(half_width_error);

        std::cout << "[Radius][LiDAR] circle " << i
                  << ": inner=" << inner_radius * 1000.0 << " mm"
                  << ", outer=" << outer_radius * 1000.0 << " mm"
                  << ", centerline error=" << centerline_error * 1000.0 << " mm"
                  << ", half-width error=" << half_width_error * 1000.0 << " mm"
                  << ", support=[" << inner_radii_by_center[i].size()
                  << ", " << outer_radii_by_center[i].size() << "]"
                  << std::endl;
    }

    double max_centerline_error = 0.0;
    double centerline_rmse = 0.0;
    double max_half_width_error = 0.0;
    for (double error : centerline_errors)
    {
        max_centerline_error = std::max(max_centerline_error, std::fabs(error));
        centerline_rmse += error * error;
    }
    for (double error : half_width_errors)
    {
        max_half_width_error = std::max(max_half_width_error, std::fabs(error));
    }
    if (!centerline_errors.empty())
    {
        centerline_rmse = std::sqrt(centerline_rmse / static_cast<double>(centerline_errors.size()));
    }

    std::cout << "[Radius][LiDAR] max centerline radius error = "
              << max_centerline_error * 1000.0
              << " mm, centerline RMSE = " << centerline_rmse * 1000.0
              << " mm, max half-width error = " << max_half_width_error * 1000.0
              << " mm" << std::endl;
}

// 根据提取到的 annulus/边界点统计半径误差，用作圆心几何质检之外的辅助检查
void validateRadiusQuality(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_cloud,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& centers_z0,
                           const Params& params,
                           LiDARType run_type)
{
    if (!edge_cloud || !centers_z0 || centers_z0->size() != TARGET_NUM_CIRCLES)
    {
        std::cout << "[Radius][LiDAR] skipped: need 4 centers in aligned board frame." << std::endl;
        return;
    }

    const double target_radius = params.circle_radius;
    const double annulus_half_width = params.annulus_half_width;
    const double inner_target = std::max(0.02, target_radius - annulus_half_width);
    const double outer_target = target_radius + annulus_half_width;

    if (run_type == LiDARType::Mech)
    {
        std::array<std::vector<double>, TARGET_NUM_CIRCLES> inner_radii_by_center;
        std::array<std::vector<double>, TARGET_NUM_CIRCLES> outer_radii_by_center;
        const double gate = 0.055;
        for (const auto& p : edge_cloud->points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;

            int best_center = -1;
            double best_residual = std::numeric_limits<double>::max();
            double best_radius = 0.0;
            bool best_inner = true;
            for (int i = 0; i < TARGET_NUM_CIRCLES; ++i)
            {
                const auto& center = centers_z0->points[i];
                const double dx = static_cast<double>(p.x) - static_cast<double>(center.x);
                const double dy = static_cast<double>(p.y) - static_cast<double>(center.y);
                const double radius = std::sqrt(dx * dx + dy * dy);
                const double inner_residual = std::fabs(radius - inner_target);
                const double outer_residual = std::fabs(radius - outer_target);
                const bool use_inner = inner_residual <= outer_residual;
                const double residual = use_inner ? inner_residual : outer_residual;
                if (residual < best_residual)
                {
                    best_center = i;
                    best_residual = residual;
                    best_radius = radius;
                    best_inner = use_inner;
                }
            }
            if (best_center < 0 || best_residual > gate) continue;
            if (best_inner)
            {
                inner_radii_by_center[best_center].push_back(best_radius);
            }
            else
            {
                outer_radii_by_center[best_center].push_back(best_radius);
            }
        }
        printMechanicalRadiusQuality(inner_radii_by_center, outer_radii_by_center,
                                     target_radius, annulus_half_width);
        return;
    }

    std::array<std::vector<double>, TARGET_NUM_CIRCLES> radii_by_center;
    const double gate = 0.07;
    for (const auto& p : edge_cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;

        int best_center = -1;
        double best_residual = std::numeric_limits<double>::max();
        double best_radius = 0.0;
        for (int i = 0; i < TARGET_NUM_CIRCLES; ++i)
        {
            const auto& center = centers_z0->points[i];
            const double dx = static_cast<double>(p.x) - static_cast<double>(center.x);
            const double dy = static_cast<double>(p.y) - static_cast<double>(center.y);
            const double radius = std::sqrt(dx * dx + dy * dy);
            const double residual = std::fabs(radius - target_radius);
            if (residual < best_residual)
            {
                best_center = i;
                best_residual = residual;
                best_radius = radius;
            }
        }
        if (best_center >= 0 && best_residual <= gate)
        {
            radii_by_center[best_center].push_back(best_radius);
        }
    }
    printSolidRadiusQuality(radii_by_center, target_radius);
}

}  // namespace

// LiDAR 圆心提取批量测试入口
int main(int argc, char** argv)
{
    std::string pointcloud_path, settings_path, output_dir, lidar_type_override = "auto";
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc)
            {
                std::cerr << "[Args] Missing value for " << arg << std::endl;
                std::exit(2);
            }
            return std::string(argv[++i]);
        };
        if (arg == "--pointcloud") pointcloud_path = next();
        else if (arg == "--settings") settings_path = next();
        else if (arg == "-o" || arg == "--output") output_dir = next();
        else if (arg == "--lidar-type") lidar_type_override = next();
        else
        {
            std::cerr << "[Args] Unknown argument: " << arg << std::endl;
            return 2;
        }
    }
    if (pointcloud_path.empty() || settings_path.empty() || output_dir.empty())
    {
        std::cerr << "Usage: " << argv[0]
                  << " --pointcloud <path|folder> --settings <path> -o <output_dir>"
                  << " [--lidar-type auto|solid|mech]" << std::endl;
        return 2;
    }

    while (!output_dir.empty() && output_dir.back() == '/') output_dir.pop_back();
    if (!ensureOutputDirectory(output_dir))
    {
        return 1;
    }

    Params params = loadSettingsYaml(settings_path);
    params.output_path = output_dir;

    PointCloudLoadResult load_result;
    if (!loadPointCloudInput(pointcloud_path, lidar_type_override, load_result))
    {
        return 1;
    }
    pcl::PointCloud<Common::Point>::Ptr cloud = load_result.cloud;
    const LiDARType run_type = load_result.type;

    std::cout << "[LiDAR Test] Pointcloud: " << pointcloud_path << std::endl;
    std::cout << "[LiDAR Test] Run type: " << lidarTypeName(run_type) << std::endl;

    LidarDetect lidar_detect(params);
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_centers(new pcl::PointCloud<pcl::PointXYZ>);

    if (run_type == LiDARType::Solid)
    {
        lidar_detect.detect_solid_lidar(cloud, raw_centers);
    }
    else if (run_type == LiDARType::Mech)
    {
        lidar_detect.detect_mech_lidar(cloud, raw_centers);
    }
    else
    {
        ROS_ERROR("[LiDAR Test] Unknown LiDAR type.");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr centers(new pcl::PointCloud<pcl::PointXYZ>);
    sortPatternCenters(raw_centers, centers, "lidar");

    std::cout << "[LiDAR Test] Raw center count: " << raw_centers->size() << std::endl;
    std::cout << "[LiDAR Test] Sorted center count: " << centers->size() << std::endl;
    for (size_t i = 0; i < centers->size(); ++i)
    {
        const auto& p = centers->points[i];
        std::cout << "[LiDAR Test] Center " << i << ": "
                  << std::fixed << std::setprecision(6)
                  << p.x << ", " << p.y << ", " << p.z << std::endl;
    }
    validateTargetGeometry(centers, params.delta_width_circles, params.delta_height_circles, "LiDAR");
    validateRadiusQuality(lidar_detect.getEdgeCloud(), lidar_detect.getCenterZ0Cloud(), params, run_type);
    saveCenterCoordinates(centers, params, pointcloud_path);
    saveDebugCloud(lidar_detect.getPlaneCloud(), lidar_detect.getAnnulusOriginalCloud(),
                   lidar_detect.getBoundaryOriginalCloud(),
                   centers, params, pointcloud_path);
    savePipelineStages(cloud, lidar_detect, params, pointcloud_path);

    return centers->size() == TARGET_NUM_CIRCLES ? 0 : 1;
}
