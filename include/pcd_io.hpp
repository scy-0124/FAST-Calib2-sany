#ifndef PCD_IO_HPP
#define PCD_IO_HPP

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <cstring>
#include <dirent.h>
#include <string>
#include <sys/stat.h>
#include <vector>
#include "common_lib.h"

// LiDAR 类型枚举：根据点云是否带 ring(线号) 字段判定，供 LidarDetect 选择 solid/mech 检测路径。
enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // 固态（如 Livox）：无固定线号
    Mech    = 2    // 机械式多线：每点带 ring 线号
};

struct PointCloudLoadResult
{
    pcl::PointCloud<Common::Point>::Ptr cloud;
    LiDARType type = LiDARType::Unknown;
};

namespace pcd_io_detail
{

// 按字段的声明类型（datatype）读出数值并转成 double，屏蔽存储字节数/是否浮点的差异
// （样例数据里 ring 是 1 字节 UINT8，不能假设固定字长）。
inline double readFieldAsDouble(const pcl::PCLPointCloud2 &blob, size_t point_index,
                                 const pcl::PCLPointField &field)
{
    const std::uint8_t *base = blob.data.data() + point_index * blob.point_step + field.offset;
    switch (field.datatype)
    {
        case pcl::PCLPointField::INT8:    { std::int8_t v;   std::memcpy(&v, base, sizeof(v)); return static_cast<double>(v); }
        case pcl::PCLPointField::UINT8:   { std::uint8_t v;  std::memcpy(&v, base, sizeof(v)); return static_cast<double>(v); }
        case pcl::PCLPointField::INT16:   { std::int16_t v;  std::memcpy(&v, base, sizeof(v)); return static_cast<double>(v); }
        case pcl::PCLPointField::UINT16:  { std::uint16_t v; std::memcpy(&v, base, sizeof(v)); return static_cast<double>(v); }
        case pcl::PCLPointField::INT32:   { std::int32_t v;  std::memcpy(&v, base, sizeof(v)); return static_cast<double>(v); }
        case pcl::PCLPointField::UINT32:  { std::uint32_t v; std::memcpy(&v, base, sizeof(v)); return static_cast<double>(v); }
        case pcl::PCLPointField::FLOAT32: { float v;         std::memcpy(&v, base, sizeof(v)); return static_cast<double>(v); }
        case pcl::PCLPointField::FLOAT64: { double v;        std::memcpy(&v, base, sizeof(v)); return v; }
        default: return 0.0;
    }
}

inline const pcl::PCLPointField *findField(const pcl::PCLPointCloud2 &blob, const std::string &name)
{
    for (const auto &f : blob.fields)
    {
        if (f.name == name) return &f;
    }
    return nullptr;
}

// 读一个 .pcd 文件，按字段名转换后累加进 cloud；探测到 ring 字段就把 saw_ring 置 true。
inline bool loadSinglePcdInto(const std::string &pcd_path,
                              pcl::PointCloud<Common::Point>::Ptr cloud, bool &saw_ring)
{
    pcl::PCLPointCloud2 blob;
    if (pcl::io::loadPCDFile(pcd_path, blob) != 0)
    {
        ROS_ERROR_STREAM("[pcd_io] Failed to load pcd: " << pcd_path);
        return false;
    }

    const pcl::PCLPointField *fx = findField(blob, "x");
    const pcl::PCLPointField *fy = findField(blob, "y");
    const pcl::PCLPointField *fz = findField(blob, "z");
    if (!fx || !fy || !fz)
    {
        ROS_ERROR_STREAM("[pcd_io] pcd missing x/y/z fields: " << pcd_path);
        return false;
    }
    const pcl::PCLPointField *fring = findField(blob, "ring");
    const pcl::PCLPointField *fintensity = findField(blob, "intensity");
    if (!fintensity) fintensity = findField(blob, "reflectivity");

    const size_t n = static_cast<size_t>(blob.width) * blob.height;
    cloud->reserve(cloud->size() + n);
    for (size_t i = 0; i < n; ++i)
    {
        Common::Point p;
        p.x = static_cast<float>(readFieldAsDouble(blob, i, *fx));
        p.y = static_cast<float>(readFieldAsDouble(blob, i, *fy));
        p.z = static_cast<float>(readFieldAsDouble(blob, i, *fz));
        p.intensity = fintensity ? static_cast<float>(readFieldAsDouble(blob, i, *fintensity)) : 0.0f;
        p.ring = fring ? static_cast<std::uint16_t>(readFieldAsDouble(blob, i, *fring)) : 0xFFFF;
        cloud->push_back(p);
    }
    if (fring) saw_ring = true;
    return true;
}

// 非递归列出目录下所有 .pcd 文件，按文件名升序排序
inline bool listPcdFilesInFolder(const std::string &folder, std::vector<std::string> &files)
{
    std::string clean_folder = folder;
    while (!clean_folder.empty() && clean_folder.back() == '/') clean_folder.pop_back();

    DIR *dir = opendir(clean_folder.c_str());
    if (!dir) return false;

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string name(entry->d_name);
        if (name.size() > 4 && name.compare(name.size() - 4, 4, ".pcd") == 0)
        {
            files.push_back(clean_folder + "/" + name);
        }
    }
    closedir(dir);
    std::sort(files.begin(), files.end());
    return true;
}

}  // namespace pcd_io_detail

// 统一点云输入入口：path 是目录就合并该目录下所有 .pcd（非递归、按文件名升序、全部累加，
// 不做降采样/截断）；是单个 .pcd 文件就只读一帧。type_override 传 "solid"/"mech" 时直接采用，
// 传 "auto" 时按合并范围内任意一帧是否探测到 ring 字段判定（Mech 优先于 Solid）。
inline bool loadPointCloudInput(const std::string &path, const std::string &type_override,
                                PointCloudLoadResult &result)
{
    result.cloud.reset(new pcl::PointCloud<Common::Point>);
    result.type = LiDARType::Unknown;

    struct stat st;
    if (stat(path.c_str(), &st) != 0)
    {
        ROS_ERROR_STREAM("[pcd_io] Path does not exist: " << path);
        return false;
    }

    bool saw_ring = false;
    if (S_ISDIR(st.st_mode))
    {
        std::vector<std::string> files;
        if (!pcd_io_detail::listPcdFilesInFolder(path, files) || files.empty())
        {
            ROS_ERROR_STREAM("[pcd_io] No .pcd files found in folder: " << path);
            return false;
        }
        for (const auto &f : files)
        {
            if (!pcd_io_detail::loadSinglePcdInto(f, result.cloud, saw_ring)) return false;
        }
        ROS_INFO("[pcd_io] Merged %zu pcd files from folder %s, total %zu points",
                 files.size(), path.c_str(), result.cloud->size());
    }
    else
    {
        if (!pcd_io_detail::loadSinglePcdInto(path, result.cloud, saw_ring)) return false;
        ROS_INFO("[pcd_io] Loaded single pcd %s, %zu points", path.c_str(), result.cloud->size());
    }

    if (type_override == "solid") result.type = LiDARType::Solid;
    else if (type_override == "mech") result.type = LiDARType::Mech;
    else result.type = saw_ring ? LiDARType::Mech : LiDARType::Solid;

    return !result.cloud->empty();
}

#endif  // PCD_IO_HPP
