# FAST-Calib2-sany 去 ROS 化 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 FAST-Calib2-sany 从 ROS1 catkin 包改造成不依赖 ROS 运行时、纯 CMake 编译的 CLI 工具，图片/点云路径由命令行显式传入，三个可执行文件（`fast_calib`/`multi_fast_calib`/`lidar_center_test`）一次性完成。

**Architecture:** 新增 `include/pcd_io.hpp`（统一 PCD 单帧/文件夹合并加载 + LiDAR 类型探测）和 `include/vehicle_config_reader.hpp`（vehicle_config `camera.yaml` 读取 + 鱼眼转 pinhole）；`common_lib.h` 去 ROS 化并新增 yaml-cpp 驱动的 settings 加载 + 非 ROS 日志宏 shim；`qr_detect.hpp`/`lidar_detect.hpp` 只做去掉 `ros::NodeHandle&`/`ros::Publisher` 的机械改动，算法本体不动；三个 `main` 入口改成 argv 解析；`CMakeLists.txt` 改纯 CMake，删除 `package.xml`。

**Tech Stack:** C++17, PCL 1.14（已装, 要求 >=1.10）, OpenCV 4（已装, 含 `opencv_aruco`）, yaml-cpp 0.8.0（已装, `find_package(yaml-cpp)` 导出 `yaml-cpp::yaml-cpp`）, 无第三方 CLI 解析库（手写 `--flag value` 解析，沿用 `lidar_center_test.cpp` 现有风格）。

**本机环境重要事实：本机没有安装 ROS**（`/opt/ros` 不存在，无 `ros-*` 包）。这意味着当前 catkin 版代码在本机根本编译不了，也印证了每个任务的验证步骤必须是真实可跑的（standalone g++ 编译或最终 CMake 构建），不能假设 ROS 环境存在。

## Global Constraints

- C++17；`#define PCL_NO_PRECOMPILE` + 自定义 `Common::Point`（XYZ+intensity+ring）不变，所有点云路径继续用它。
- PCL >= 1.10（本机 1.14.0），OpenCV >= 4.0 含 `opencv_aruco`（本机已装），新增依赖 yaml-cpp（本机 0.8.0，CMake 目标名 `yaml-cpp::yaml-cpp`）。
- 不改动标定算法本体：`lidar_detect.hpp` 的 solid/mech 检测流水线、`common_lib.h` 的 `sortPatternCenters`/`validateTargetGeometry`/`Square`/SVD 外参逻辑，原样保留。
- 不新增自动化测试（仓库历来没有 test target）；每个任务的"测试"是真实编译+对样例数据跑一次，人工核对输出结构/数值是否合理。
- 必填 CLI 参数缺失时打印 usage 到 stderr 并 `return 1`/`2`，不设默认值静默兜底；settings YAML 缺字段报错退出，不落到硬编码默认值；不设编译期默认 settings 路径。
- 注释以中文为主，不翻译/删减既有中文注释；GPLv2 许可证与原作者署名不变。
- 临时验证脚本一律写到 scratchpad 目录 `/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad`（下称 `$SCRATCH`），**不提交到仓库**。
- 样例数据（本机真实存在，供每个任务验证用）：
  - 图像：`/home/calib/音乐/code/FAST-Calib2-sany/calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/camera/encoding_front_fisheye/1782788887590.png`
  - 点云单帧：`/home/calib/音乐/code/FAST-Calib2-sany/calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/1782788887674.pcd`
  - 点云文件夹（27 帧）：`/home/calib/音乐/code/FAST-Calib2-sany/calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/`
  - 相机内参：`/home/calib/workspace/project-calib/vehicle_config/L2_data_023/camera.yaml`，`camera_name: front_fisheye`（`model_type: fisheye`）
- 参考设计文档：`docs/superpowers/specs/2026-07-01-去ros化-design.md`（本计划的每个决策都来自那份 spec，冲突时以 spec 为准）。

---

### Task 1: `common_lib.h` 去 ROS 化 + settings YAML 瘦身

**Files:**
- Modify: `include/common_lib.h`（全文件，重点是第 8-112 行：includes / Params / loadParameters）
- Modify: `config/qr_params.yaml`（瘦身，去掉相机内参和路径字段）
- Test: `$SCRATCH/task1_test.cpp`（临时文件，不提交）

**Interfaces:**
- Consumes: 无（本任务是最底层基础设施）
- Produces:
  - `struct Params`（字段见下方 Step 3 代码，供后续所有任务使用）
  - `Params loadSettingsYaml(const std::string& path)` —— 读取标定板几何/LiDAR ROI 类字段，缺字段直接 `std::exit(1)`
  - 宏 `ROS_INFO(...)` / `ROS_WARN(...)` / `ROS_ERROR(...)` / `ROS_ERROR_STREAM(x)` / `ROS_WARN_STREAM(x)`（非 ROS，纯 stdio/iostream 实现，调用语法与原 roscpp 宏一致）
  - 其余已有函数签名不变：`computeRMSE`/`alignPointCloud`/`comb`/`projectPointCloudToImage`/`saveTargetHoleCenters`/`saveCalibrationResults`/`sortPatternCenters`/`distance3D`/`validateTargetGeometry`/`Square`

- [ ] **Step 1: 备份当前 `config/qr_params.yaml` 到 scratchpad，供对照**

```bash
cp "/home/calib/音乐/code/FAST-Calib2-sany/config/qr_params.yaml" \
   "/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad/qr_params.yaml.orig"
```

- [ ] **Step 2: 改写 `config/qr_params.yaml`，去掉相机内参和路径字段**

把整个文件替换为：

```yaml
# Calibration target parameters
marker_size: 0.20 # ArUco marker size (our test data uses 0.16m; adjust to match your marker size)
delta_width_qr_center: 0.55 # Half the distance between the centers of two markers in the horizontal direction
delta_height_qr_center: 0.35 # Half the distance between the centers of two markers in the vertical direction
delta_width_circles: 0.5 # Distance between the centers of two annuli in the horizontal direction
delta_height_circles: 0.4 # Distance between the centers of two annuli in the vertical direction
circle_radius: 0.12 # Radius of the reflective annulus centerline
annulus_half_width: 0.025 # Half width of annulus band: (outer radius - inner radius) / 2
board_width: 1.4
board_height: 1.0
board_roi_margin: 0.08
board_roi_depth: 0.12
auto_roi_voxel_leaf: 0.01
annulus_voxel_leaf: 0.005
min_detected_markers: 3

# LiDAR ROI
use_auto_lidar_roi: true

x_min: 2.6
x_max: 3.0
y_min: -1.0
y_max: 0.8
z_min: -0.3
z_max: 1.1
```

- [ ] **Step 3: 改写 `include/common_lib.h` 第 8-112 行**

把原文件开头到 `loadParameters` 函数结束（原第 8-112 行）整体替换为：

```cpp
#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#define PCL_NO_PRECOMPILE

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "color.h"

using namespace std;
using namespace cv;
using namespace pcl;

#define TARGET_NUM_CIRCLES 4
#define DEBUG 1
#define GEOMETRY_TOLERANCE 0.08

// ===== 非 ROS 日志宏：保留 ROS_INFO/WARN/ERROR 调用语法，去掉 roscpp 依赖 =====
#define ROS_INFO(...)          do { std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#define ROS_WARN(...)          do { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define ROS_ERROR(...)         do { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define ROS_ERROR_STREAM(x)    do { std::cerr << x << std::endl; } while (0)
#define ROS_WARN_STREAM(x)     do { std::cerr << x << std::endl; } while (0)

// ===== 自定义点类型：XYZ + intensity + ring =====
namespace Common
{
  struct Point
  {
    PCL_ADD_POINT4D;
    float intensity = 0.0f;      // LiDAR intensity / reflectivity
    std::uint16_t ring = 0;      // 线号（机械雷达/多线雷达）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Common::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
);

// 参数结构体：相机内参（vehicle_config 填入）+ 标定板几何/ROI（settings YAML 填入）+ 输出路径（CLI 填入）
struct Params {
  // 相机内参 + 畸变模型
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double k1 = 0, k2 = 0, p1 = 0, p2 = 0;   // pinhole/rational 路径喂给 cv::undistort
  std::vector<double> dist_coeffs;          // fisheye/equidistant 路径用（k1..k4）
  std::string camera_model = "pinhole";     // 来自 camera.yaml 的 model_type
  int image_width = 0, image_height = 0;    // camera.yaml 元数据，仅供参考不参与计算

  // 标定板几何 + LiDAR ROI
  double x_min, x_max, y_min, y_max, z_min, z_max;
  bool use_auto_lidar_roi;
  double marker_size, delta_width_qr_center, delta_height_qr_center;
  double delta_width_circles, delta_height_circles, circle_radius, annulus_half_width;
  double board_width, board_height, board_roi_margin, board_roi_depth;
  double auto_roi_voxel_leaf, annulus_voxel_leaf;
  int min_detected_markers;

  // 输出目录
  std::string output_path;
};

// 从 settings YAML 读取标定板几何/LiDAR ROI 参数；缺任意必需字段直接报错退出
// （不像原来 nh.param 那样静默落到硬编码默认值）。
inline Params loadSettingsYaml(const std::string &path)
{
  Params params;
  YAML::Node y;
  try
  {
    y = YAML::LoadFile(path);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("[Settings] Failed to load " << path << ": " << e.what());
    std::exit(1);
  }

  auto need = [&](const char *key) -> YAML::Node {
    if (!y[key])
    {
      ROS_ERROR_STREAM("[Settings] Missing required key '" << key << "' in " << path);
      std::exit(1);
    }
    return y[key];
  };

  params.marker_size            = need("marker_size").as<double>();
  params.delta_width_qr_center  = need("delta_width_qr_center").as<double>();
  params.delta_height_qr_center = need("delta_height_qr_center").as<double>();
  params.delta_width_circles    = need("delta_width_circles").as<double>();
  params.delta_height_circles   = need("delta_height_circles").as<double>();
  params.min_detected_markers   = need("min_detected_markers").as<int>();
  params.circle_radius          = need("circle_radius").as<double>();
  params.annulus_half_width     = need("annulus_half_width").as<double>();
  params.board_width            = need("board_width").as<double>();
  params.board_height           = need("board_height").as<double>();
  params.board_roi_margin       = need("board_roi_margin").as<double>();
  params.board_roi_depth        = need("board_roi_depth").as<double>();
  params.auto_roi_voxel_leaf    = need("auto_roi_voxel_leaf").as<double>();
  params.annulus_voxel_leaf     = need("annulus_voxel_leaf").as<double>();
  params.use_auto_lidar_roi     = need("use_auto_lidar_roi").as<bool>();
  params.x_min = need("x_min").as<double>();
  params.x_max = need("x_max").as<double>();
  params.y_min = need("y_min").as<double>();
  params.y_max = need("y_max").as<double>();
  params.z_min = need("z_min").as<double>();
  params.z_max = need("z_max").as<double>();

  return params;
}
```

其余函数（`computeRMSE` 及以后，原第 114-575 行）**原样保留，不要改动**——包括结尾的 `#endif`。

- [ ] **Step 4: 写临时验证程序 `$SCRATCH/task1_test.cpp`**

```cpp
#include "common_lib.h"

int main()
{
  Params p = loadSettingsYaml(
      "/home/calib/音乐/code/FAST-Calib2-sany/config/qr_params.yaml");
  std::cout << "marker_size=" << p.marker_size
            << " delta_width_circles=" << p.delta_width_circles
            << " circle_radius=" << p.circle_radius
            << " use_auto_lidar_roi=" << p.use_auto_lidar_roi
            << " output_path(should be empty)='" << p.output_path << "'"
            << std::endl;
  ROS_INFO("ROS_INFO shim works, value=%d", 42);
  ROS_WARN("ROS_WARN shim works");
  ROS_ERROR_STREAM("ROS_ERROR_STREAM shim works: " << 3.14);
  return 0;
}
```

- [ ] **Step 5: 编译并运行验证（不需要 ROS，只需要 PCL/OpenCV/yaml-cpp）**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
g++ -std=c++17 -Iinclude \
  $(pkg-config --cflags pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp) \
  /tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad/task1_test.cpp \
  -o /tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad/task1_test \
  $(pkg-config --libs pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp)
/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad/task1_test
```

Expected: 编译无 ROS 相关报错（不需要任何 `-I/opt/ros/...` 路径就能过编译，证明 `common_lib.h` 真正脱离了 ROS）；运行输出打印出 YAML 里的真实数值（`marker_size=0.2 delta_width_circles=0.5 circle_radius=0.12 use_auto_lidar_roi=1 output_path(should be empty)=''`），以及三行日志宏 shim 的输出（后两行应出现在 stderr）。

- [ ] **Step 6: 清理临时文件，提交**

```bash
rm -f /tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad/task1_test*
cd /home/calib/音乐/code/FAST-Calib2-sany
git add include/common_lib.h config/qr_params.yaml
git commit -m "$(cat <<'EOF'
去ROS化: common_lib.h 去掉 ROS 依赖，settings 参数改走 yaml-cpp

Params 拆成"相机内参(vehicle_config填)/标定板几何+ROI(settings YAML填)/
输出路径(CLI填)"三块；loadParameters(ros::NodeHandle&) 替换成
loadSettingsYaml(path)，缺字段直接报错退出；新增非ROS的
ROS_INFO/WARN/ERROR(_STREAM) 宏 shim，供后续文件的既有调用点原样复用。
qr_params.yaml 同步瘦身，去掉相机内参和路径字段。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: 新增 `include/pcd_io.hpp`（单帧/文件夹合并 + LiDAR 类型探测）

**Files:**
- Create: `include/pcd_io.hpp`
- Test: `$SCRATCH/task2_test.cpp`

**Interfaces:**
- Consumes: `Common::Point`、`ROS_INFO`/`ROS_ERROR_STREAM`（Task 1 的 `common_lib.h`）
- Produces:
  - `enum class LiDARType : int { Unknown = 0, Solid = 1, Mech = 2 }`
  - `struct PointCloudLoadResult { pcl::PointCloud<Common::Point>::Ptr cloud; LiDARType type; }`
  - `bool loadPointCloudInput(const std::string& path, const std::string& type_override, PointCloudLoadResult& result)` —— `type_override` 取值 `"auto"`/`"solid"`/`"mech"`；`path` 是目录就合并该目录下所有 `.pcd`（非递归、按文件名升序），是单个 `.pcd` 文件就只读一帧；成功返回 `true` 并填好 `result.cloud`/`result.type`，失败返回 `false`（已打印错误）。

- [ ] **Step 1: 写 `include/pcd_io.hpp`**

```cpp
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
```

- [ ] **Step 2: 写临时验证程序 `$SCRATCH/task2_test.cpp`**

```cpp
#include "pcd_io.hpp"

int main()
{
  const std::string single = "/home/calib/音乐/code/FAST-Calib2-sany/calib_data/"
      "Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/1782788887674.pcd";
  const std::string folder = "/home/calib/音乐/code/FAST-Calib2-sany/calib_data/"
      "Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/";

  PointCloudLoadResult r1;
  bool ok1 = loadPointCloudInput(single, "auto", r1);
  std::cout << "single: ok=" << ok1 << " points=" << r1.cloud->size()
            << " type=" << static_cast<int>(r1.type) << " (expect 2=Mech)" << std::endl;

  PointCloudLoadResult r2;
  bool ok2 = loadPointCloudInput(folder, "auto", r2);
  std::cout << "folder: ok=" << ok2 << " points=" << r2.cloud->size()
            << " type=" << static_cast<int>(r2.type)
            << " (expect ~27x more points than single frame)" << std::endl;

  PointCloudLoadResult r3;
  bool ok3 = loadPointCloudInput(single, "solid", r3);
  std::cout << "override solid: type=" << static_cast<int>(r3.type) << " (expect 1=Solid)" << std::endl;

  return (ok1 && ok2 && ok3) ? 0 : 1;
}
```

- [ ] **Step 3: 编译并运行验证**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
SCRATCH=/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad
g++ -std=c++17 -Iinclude \
  $(pkg-config --cflags pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp) \
  $SCRATCH/task2_test.cpp -o $SCRATCH/task2_test \
  $(pkg-config --libs pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp)
$SCRATCH/task2_test
```

Expected: `single` 行 `ok=1`，`points=115200`，`type=2`；`folder` 行 `ok=1`，`points` 约为 `115200 * 27`（各帧点数若有差异也应接近这个量级），`type=2`；`override solid` 行 `type=1`。退出码 0。

- [ ] **Step 4: 清理临时文件，提交**

```bash
rm -f $SCRATCH/task2_test*
cd /home/calib/音乐/code/FAST-Calib2-sany
git add include/pcd_io.hpp
git commit -m "$(cat <<'EOF'
去ROS化: 新增 pcd_io.hpp 统一处理 PCD 单帧/文件夹合并 + LiDAR 类型探测

替代原来分别在 data_preprocess.hpp 和 lidar_center_test.cpp 里各写一遍的
rosbag 解析逻辑。文件夹合并非递归扫 .pcd、按文件名升序全部累加，不做降
采样/截断；字段按声明的 datatype 读取再转换，不假设 ring/intensity 的
存储字长（样例数据 ring 是 1 字节 UINT8）。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: 新增 `include/vehicle_config_reader.hpp`（camera.yaml 读取 + 鱼眼转 pinhole）

**Files:**
- Create: `include/vehicle_config_reader.hpp`
- Test: `$SCRATCH/task3_test.cpp`

**Interfaces:**
- Consumes: `Params`（Task 1）
- Produces:
  - `bool loadCameraFromVehicleConfig(const std::string& vehicle_config_dir, const std::string& camera_name, Params& params)`
  - `bool applyFisheyeUndistortIfNeeded(Params& params, cv::Mat& image)` —— fisheye/equidistant 时原地把 `image` 换成 pinhole 等效图并清空 `params` 里的畸变系数、`camera_model` 改成 `"pinhole"`，返回 `true`；非鱼眼族什么都不做，返回 `false`。

- [ ] **Step 1: 写 `include/vehicle_config_reader.hpp`**

```cpp
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
```

- [ ] **Step 2: 写临时验证程序 `$SCRATCH/task3_test.cpp`**

```cpp
#include "vehicle_config_reader.hpp"
#include <opencv2/imgcodecs.hpp>

int main()
{
  Params params;
  bool ok = loadCameraFromVehicleConfig(
      "/home/calib/workspace/project-calib/vehicle_config/L2_data_023",
      "front_fisheye", params);
  std::cout << "loadCameraFromVehicleConfig ok=" << ok
            << " fx=" << params.fx << " model=" << params.camera_model
            << " dist_coeffs.size()=" << params.dist_coeffs.size() << std::endl;
  if (!ok) return 1;

  cv::Mat img = cv::imread(
      "/home/calib/音乐/code/FAST-Calib2-sany/calib_data/"
      "Taijia_001_20260630110807641Z8_20260630110810414Z8/camera/"
      "encoding_front_fisheye/1782788887590.png", cv::IMREAD_UNCHANGED);
  std::cout << "loaded image: " << img.cols << "x" << img.rows << std::endl;

  bool undistorted = applyFisheyeUndistortIfNeeded(params, img);
  std::cout << "applyFisheyeUndistortIfNeeded=" << undistorted
            << " new size=" << img.cols << "x" << img.rows
            << " model after=" << params.camera_model
            << " dist_coeffs.size() after=" << params.dist_coeffs.size() << std::endl;

  cv::imwrite("/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/"
              "fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad/task3_undistorted.png", img);
  return 0;
}
```

- [ ] **Step 3: 编译并运行验证**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
SCRATCH=/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad
g++ -std=c++17 -Iinclude \
  $(pkg-config --cflags pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp) \
  $SCRATCH/task3_test.cpp -o $SCRATCH/task3_test \
  $(pkg-config --libs pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp)
$SCRATCH/task3_test
```

Expected: `loadCameraFromVehicleConfig ok=1 fx=515.88... model=fisheye dist_coeffs.size()=4`；图像尺寸 `1920x1536`（或实际样例图片尺寸）；`applyFisheyeUndistortIfNeeded=1`，undistort 后尺寸不变，`model after=pinhole`，`dist_coeffs.size() after=0`。用 `Read` 工具打开 `$SCRATCH/task3_undistorted.png` 目视检查：标定板边缘应该比原图更直（鱼眼弯曲消除），不应该是全黑/全花的错误图像。

- [ ] **Step 4: 清理临时文件，提交**

```bash
rm -f $SCRATCH/task3_test $SCRATCH/task3_test.cpp $SCRATCH/task3_undistorted.png
cd /home/calib/音乐/code/FAST-Calib2-sany
git add include/vehicle_config_reader.hpp
git commit -m "$(cat <<'EOF'
去ROS化: 新增 vehicle_config_reader.hpp 读取相机内参 + 鱼眼转 pinhole

按 -c <vehicle_config_dir> --camera <camera_name> 读 camera.yaml，
model_type 是 fisheye/equidistant 时用原始 K（不重估）+ R=I 做
initUndistortRectifyMap + remap 转成 pinhole 等效图，之后清空畸变系数，
后续 QRDetect 走现有 pinhole 逻辑；rational/pinhole 只取 k1,k2,p1,p2。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: 重写 `src/data_preprocess.hpp`（图像/点云解耦，去掉 rosbag）

**Files:**
- Modify: `src/data_preprocess.hpp`（整个文件重写，篇幅比原来小很多）
- Test: `$SCRATCH/task4_test.cpp`

**Interfaces:**
- Consumes: `loadPointCloudInput`/`LiDARType`（Task 2 的 `pcd_io.hpp`）
- Produces: `class DataPreprocess`，构造函数签名变为
  `DataPreprocess(const std::string& image_path, const std::string& pointcloud_path, const std::string& lidar_type_override)`，
  成员 `cloud_input_`/`img_input_`/`lidarType()` 不变（供 Task 7 的 `main.cpp` 用）。

- [ ] **Step 1: 整体重写 `src/data_preprocess.hpp`**

```cpp
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
```

- [ ] **Step 2: 写临时验证程序 `$SCRATCH/task4_test.cpp`**

```cpp
#include "data_preprocess.hpp"

int main()
{
  const std::string img = "/home/calib/音乐/code/FAST-Calib2-sany/calib_data/"
      "Taijia_001_20260630110807641Z8_20260630110810414Z8/camera/"
      "encoding_front_fisheye/1782788887590.png";
  const std::string single_pcd = "/home/calib/音乐/code/FAST-Calib2-sany/calib_data/"
      "Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/1782788887674.pcd";
  const std::string folder = "/home/calib/音乐/code/FAST-Calib2-sany/calib_data/"
      "Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/";

  DataPreprocess dp1(img, single_pcd, "auto");
  std::cout << "single: img_empty=" << dp1.img_input_.empty()
            << " cloud_size=" << dp1.cloud_input_->size()
            << " type=" << static_cast<int>(dp1.lidarType()) << std::endl;

  DataPreprocess dp2(img, folder, "auto");
  std::cout << "folder: img_empty=" << dp2.img_input_.empty()
            << " cloud_size=" << dp2.cloud_input_->size()
            << " type=" << static_cast<int>(dp2.lidarType()) << std::endl;

  DataPreprocess dp3("/no/such/image.png", single_pcd, "auto");
  std::cout << "bad image path: img_empty=" << dp3.img_input_.empty()
            << " (expect 1, error already printed above)" << std::endl;

  return (!dp1.img_input_.empty() && !dp1.cloud_input_->empty() &&
          !dp2.img_input_.empty() && !dp2.cloud_input_->empty()) ? 0 : 1;
}
```

- [ ] **Step 3: 编译并运行验证**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
SCRATCH=/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad
g++ -std=c++17 -Iinclude -Isrc \
  $(pkg-config --cflags pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp) \
  $SCRATCH/task4_test.cpp -o $SCRATCH/task4_test \
  $(pkg-config --libs pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp)
$SCRATCH/task4_test
```

Expected: `single`/`folder` 两行 `img_empty=0`、`cloud_size` 分别约 `115200` 和 `115200*27`、`type=2`；`bad image path` 那行先打印一条 `[DataPreprocess] Loading the image ... failed` 错误，再打印 `img_empty=1`。退出码 0。

- [ ] **Step 4: 清理临时文件，提交**

```bash
rm -f $SCRATCH/task4_test*
cd /home/calib/音乐/code/FAST-Calib2-sany
git add src/data_preprocess.hpp
git commit -m "$(cat <<'EOF'
去ROS化: data_preprocess.hpp 改用 pcd_io.hpp，去掉 rosbag/livox 依赖

图像(cv::imread)和点云(loadPointCloudInput)解耦成两步独立加载，任一失败
只报错不影响另一个字段；不再需要区分 livox_ros_driver::CustomMsg 和
sensor_msgs::PointCloud2 两条消息类型分支——PCD 文件没有这个概念。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: `src/qr_detect.hpp` 去掉 `ros::NodeHandle&`/`ros::Publisher`

**Files:**
- Modify: `src/qr_detect.hpp:8-53`（includes + 类成员 + 构造函数，其余 detect_qr 等函数体不动）
- Test: `$SCRATCH/task5_test.cpp`

**Interfaces:**
- Consumes: `Params`（Task 1）、`loadCameraFromVehicleConfig`/`applyFisheyeUndistortIfNeeded`（Task 3）
- Produces: `class QRDetect`，构造函数签名变为 `QRDetect(Params& params)`（去掉 `ros::NodeHandle&` 参数），公开成员 `cameraMatrix_`/`distCoeffs_`/`imageCopy_` 不变，`detect_qr(cv::Mat&, pcl::PointCloud<pcl::PointXYZ>::Ptr)` 签名不变。

- [ ] **Step 1: 替换 `src/qr_detect.hpp` 第 8-53 行**

原第 8-53 行（从 `#ifndef QR_DETECT_HPP` 到构造函数结束的 `}`）替换为：

```cpp
#ifndef QR_DETECT_HPP
#define QR_DETECT_HPP
#include <opencv2/aruco.hpp>
#include "common_lib.h"

class QRDetect
{
  private:
    double marker_size_, delta_width_qr_center_, delta_height_qr_center_;
    double delta_width_circles_, delta_height_circles_;
    int min_detected_markers_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

  public:
    cv::Mat imageCopy_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    QRDetect(Params& params)
    {
      marker_size_ = params.marker_size;
      delta_width_qr_center_ = params.delta_width_qr_center;
      delta_height_qr_center_ = params.delta_height_qr_center;
      delta_width_circles_ = params.delta_width_circles;
      delta_height_circles_ = params.delta_height_circles;
      min_detected_markers_ = params.min_detected_markers;

      // Initialize camera matrix
      cameraMatrix_ = (cv::Mat_<float>(3, 3) << params.fx, 0, params.cx,
                                                0, params.fy, params.cy,
                                                0,         0,        1);

      // Initialize distortion coefficients
      distCoeffs_ = (cv::Mat_<float>(1, 5) << params.k1, params.k2, params.p1, params.p2, 0);

      // Initialize QR dictionary
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    }
```

其余从 `Point2f projectPointDist(...)`（原第 55 行）到文件末尾 **原样保留，不要改动**（包括 `typedef std::shared_ptr<QRDetect> QRDetectPtr;` 和 `#endif`）。

- [ ] **Step 2: 写临时验证程序 `$SCRATCH/task5_test.cpp`**

```cpp
#include "qr_detect.hpp"
#include "vehicle_config_reader.hpp"
#include <opencv2/imgcodecs.hpp>

int main()
{
  Params params = loadSettingsYaml("/home/calib/音乐/code/FAST-Calib2-sany/config/qr_params.yaml");
  bool cam_ok = loadCameraFromVehicleConfig(
      "/home/calib/workspace/project-calib/vehicle_config/L2_data_023", "front_fisheye", params);
  if (!cam_ok) return 1;

  cv::Mat img = cv::imread(
      "/home/calib/音乐/code/FAST-Calib2-sany/calib_data/"
      "Taijia_001_20260630110807641Z8_20260630110810414Z8/camera/"
      "encoding_front_fisheye/1782788887590.png", cv::IMREAD_UNCHANGED);
  if (img.empty()) return 1;
  applyFisheyeUndistortIfNeeded(params, img);

  QRDetect qr(params);
  pcl::PointCloud<pcl::PointXYZ>::Ptr centers(new pcl::PointCloud<pcl::PointXYZ>);
  qr.detect_qr(img, centers);
  std::cout << "qr centers found: " << centers->size() << " (0-4, depends on board geometry match)" << std::endl;
  for (const auto& p : *centers)
  {
    std::cout << "  center: " << p.x << ", " << p.y << ", " << p.z << std::endl;
  }
  return 0;
}
```

- [ ] **Step 3: 编译并运行验证**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
SCRATCH=/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad
g++ -std=c++17 -Iinclude -Isrc \
  $(pkg-config --cflags pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp) \
  $SCRATCH/task5_test.cpp -o $SCRATCH/task5_test \
  $(pkg-config --libs pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp)
$SCRATCH/task5_test
```

Expected: 编译通过（不需要任何 ROS 头文件路径）；运行不 crash，打印 `qr centers found: N`。**N 是 0 还是 4 都算通过**——这一步只验证"去掉 ros::NodeHandle& 之后 QRDetect 还能正常跑鱼眼 undistort 后的图像、不崩溃"，具体能不能测到 4 个标记取决于 `config/qr_params.yaml` 里的板子几何参数是否匹配这批真实数据（阈值调参不在本次改造范围内，若 N=0 或几何校验报警，属已知情况，记录下来即可，不算本任务失败）。

- [ ] **Step 4: 清理临时文件，提交**

```bash
rm -f $SCRATCH/task5_test*
cd /home/calib/音乐/code/FAST-Calib2-sany
git add src/qr_detect.hpp
git commit -m "$(cat <<'EOF'
去ROS化: qr_detect.hpp 去掉 ros::NodeHandle&/ros::Publisher

删掉未使用的 cv_bridge/image_geometry/message_filters/ros/ros.h 头文件，
构造函数从 QRDetect(ros::NodeHandle&, Params&) 简化成 QRDetect(Params&)，
删掉 qr_pub_ 及其 advertise 调用（RViz 发布改到 main.cpp 落盘处理）。
detect_qr 等检测算法本体不变。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 6: `src/lidar_detect.hpp` 去掉 `ros::NodeHandle&`/`ros::Publisher`

**Files:**
- Modify: `src/lidar_detect.hpp:12-15`（includes）、`:1826-1872`（构造函数与 publisher 成员）
- Test: `$SCRATCH/task6_test.cpp`

**Interfaces:**
- Consumes: `Params`（Task 1）、`loadPointCloudInput`（Task 2）
- Produces: `class LidarDetect`，构造函数签名变为 `LidarDetect(Params& params)`（去掉 `ros::NodeHandle&` 参数），`detect_solid_lidar`/`detect_mech_lidar` 及全部 `get*Cloud()` getter 签名不变。

- [ ] **Step 1: 修改 `src/lidar_detect.hpp` 第 12-15 行（includes）**

原：
```cpp
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>
#include <ros/ros.h>
```

改为：
```cpp
#include <Eigen/Dense>
```

- [ ] **Step 2: 修改 `src/lidar_detect.hpp` 第 1826-1872 行（publisher 成员 + 构造函数）**

原（`public:` 到构造函数结束的 `}`，即原第 1826-1872 行）：
```cpp
public:
    ros::Publisher filtered_pub_;
    ros::Publisher plane_pub_;
    ros::Publisher annulus_pub_;
    ros::Publisher boundary_pub_;
    ros::Publisher aligned_pub_;
    ros::Publisher edge_pub_;
    ros::Publisher center_z0_pub_;
    ros::Publisher center_pub_;

    // 构造 LiDAR 检测器并读取参数、初始化调试点云发布器
    LidarDetect(ros::NodeHandle &nh, Params &params)
        : filtered_cloud_(new pcl::PointCloud<Common::Point>),
          plane_cloud_(new pcl::PointCloud<Common::Point>),
          annulus_original_cloud_(new pcl::PointCloud<Common::Point>),
          boundary_original_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          edge_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          center_z0_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        x_min_ = params.x_min;
        x_max_ = params.x_max;
        y_min_ = params.y_min;
        y_max_ = params.y_max;
        z_min_ = params.z_min;
        z_max_ = params.z_max;
        circle_radius_ = params.circle_radius;
        annulus_half_width_ = params.annulus_half_width;
        delta_width_circles_ = params.delta_width_circles;
        delta_height_circles_ = params.delta_height_circles;
        board_width_ = params.board_width;
        board_height_ = params.board_height;
        board_roi_margin_ = params.board_roi_margin;
        board_roi_depth_ = params.board_roi_depth;
        auto_roi_voxel_leaf_ = params.auto_roi_voxel_leaf;
        annulus_voxel_leaf_ = params.annulus_voxel_leaf;
        use_auto_lidar_roi_ = params.use_auto_lidar_roi;

        filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
        plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);
        annulus_pub_ = nh.advertise<sensor_msgs::PointCloud2>("annulus_cloud", 1);
        boundary_pub_ = nh.advertise<sensor_msgs::PointCloud2>("boundary_cloud", 1);
        aligned_pub_ = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
        edge_pub_ = nh.advertise<sensor_msgs::PointCloud2>("edge_cloud", 1);
        center_z0_pub_ = nh.advertise<sensor_msgs::PointCloud2>("center_z0_cloud", 10);
        center_pub_ = nh.advertise<sensor_msgs::PointCloud2>("center_cloud", 10);
    }
```

改为：
```cpp
public:
    // 构造 LiDAR 检测器并读取参数（调试点云改由 main.cpp 落盘，不再发布 RViz topic）
    LidarDetect(Params &params)
        : filtered_cloud_(new pcl::PointCloud<Common::Point>),
          plane_cloud_(new pcl::PointCloud<Common::Point>),
          annulus_original_cloud_(new pcl::PointCloud<Common::Point>),
          boundary_original_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          edge_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          center_z0_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        x_min_ = params.x_min;
        x_max_ = params.x_max;
        y_min_ = params.y_min;
        y_max_ = params.y_max;
        z_min_ = params.z_min;
        z_max_ = params.z_max;
        circle_radius_ = params.circle_radius;
        annulus_half_width_ = params.annulus_half_width;
        delta_width_circles_ = params.delta_width_circles;
        delta_height_circles_ = params.delta_height_circles;
        board_width_ = params.board_width;
        board_height_ = params.board_height;
        board_roi_margin_ = params.board_roi_margin;
        board_roi_depth_ = params.board_roi_depth;
        auto_roi_voxel_leaf_ = params.auto_roi_voxel_leaf;
        annulus_voxel_leaf_ = params.annulus_voxel_leaf;
        use_auto_lidar_roi_ = params.use_auto_lidar_roi;
    }
```

其余（`detect_mech_lidar` 及以后，原第 1874 行开始到文件末尾）**原样保留，不要改动**。

- [ ] **Step 3: 写临时验证程序 `$SCRATCH/task6_test.cpp`**

```cpp
#include "lidar_detect.hpp"
#include "pcd_io.hpp"

int main()
{
  Params params = loadSettingsYaml("/home/calib/音乐/code/FAST-Calib2-sany/config/qr_params.yaml");

  PointCloudLoadResult result;
  bool ok = loadPointCloudInput(
      "/home/calib/音乐/code/FAST-Calib2-sany/calib_data/"
      "Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/1782788887674.pcd",
      "auto", result);
  if (!ok) return 1;
  std::cout << "loaded cloud size=" << result.cloud->size()
            << " type=" << static_cast<int>(result.type) << " (expect 2=Mech)" << std::endl;

  LidarDetect lidar(params);
  pcl::PointCloud<pcl::PointXYZ>::Ptr centers(new pcl::PointCloud<pcl::PointXYZ>);
  lidar.detect_mech_lidar(result.cloud, centers);
  std::cout << "lidar centers found: " << centers->size() << " (0-4, depends on ROI/geometry match)" << std::endl;
  for (const auto& p : *centers)
  {
    std::cout << "  center: " << p.x << ", " << p.y << ", " << p.z << std::endl;
  }
  return 0;
}
```

- [ ] **Step 4: 编译并运行验证**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
SCRATCH=/tmp/claude-1000/-home-calib----code-FAST-Calib2-sany/fc3d2ffa-df5c-42a8-af43-34c53ca876a0/scratchpad
g++ -std=c++17 -Iinclude -Isrc \
  $(pkg-config --cflags pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp) \
  $SCRATCH/task6_test.cpp -o $SCRATCH/task6_test \
  $(pkg-config --libs pcl_common pcl_io pcl_filters pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_sample_consensus opencv4 yaml-cpp)
$SCRATCH/task6_test
```

Expected: 编译通过（不需要 ROS 头文件路径）；`loaded cloud size=115200 type=2`；`lidar centers found: N` 不 crash（N 是 0-4 都算通过，理由同 Task 5 —— ROI/几何阈值调参不在本次范围内）。

- [ ] **Step 5: 清理临时文件，提交**

```bash
rm -f $SCRATCH/task6_test*
cd /home/calib/音乐/code/FAST-Calib2-sany
git add src/lidar_detect.hpp
git commit -m "$(cat <<'EOF'
去ROS化: lidar_detect.hpp 去掉 ros::NodeHandle&/8个ros::Publisher

删掉 sensor_msgs/PointCloud2.h、geometry_msgs/PointStamped.h、ros/ros.h
三个头文件（都只用来支撑 publisher，算法本体不依赖）；构造函数从
LidarDetect(ros::NodeHandle&, Params&) 简化成 LidarDetect(Params&)；
删掉 filtered_pub_/plane_pub_/annulus_pub_/boundary_pub_/aligned_pub_/
edge_pub_/center_z0_pub_/center_pub_ 及其 advertise 调用。solid/mech
两条检测流水线本体一行未动。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 7: 重写 `src/main.cpp` + `CMakeLists.txt` 改纯 CMake（`fast_calib` 首次真实构建）

**Files:**
- Modify: `src/main.cpp`（整个文件重写）
- Modify: `CMakeLists.txt`（整个文件重写，先只含 `fast_calib` 一个 target）
- Delete: `package.xml`
- Test: 真实 `cmake .. && make`，跑样例数据

**Interfaces:**
- Consumes: `DataPreprocess`（Task 4）、`QRDetect`（Task 5）、`LidarDetect`（Task 6）、`loadCameraFromVehicleConfig`/`applyFisheyeUndistortIfNeeded`（Task 3）、`loadSettingsYaml`（Task 1）
- Produces: 可执行文件 `build/fast_calib`，CLI 见下方 Step 1

- [ ] **Step 1: 整体重写 `src/main.cpp`**

```cpp
/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include "vehicle_config_reader.hpp"

#include <sys/stat.h>

namespace
{

void printUsage(const char *argv0)
{
    std::cerr << "Usage: " << argv0
              << " --image <path> --pointcloud <path|folder> --settings <path>"
              << " -c <vehicle_config_dir> --camera <camera_name> -o <output_dir>"
              << " [--lidar-type auto|solid|mech]" << std::endl;
}

bool parseArgs(int argc, char **argv, std::string &image_path, std::string &pointcloud_path,
               std::string &settings_path, std::string &vehicle_config_dir,
               std::string &camera_name, std::string &output_dir, std::string &lidar_type_override)
{
    lidar_type_override = "auto";
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc)
            {
                std::cerr << "[Args] Missing value for " << arg << std::endl;
                std::exit(1);
            }
            return std::string(argv[++i]);
        };

        if (arg == "--image") image_path = next();
        else if (arg == "--pointcloud") pointcloud_path = next();
        else if (arg == "--settings") settings_path = next();
        else if (arg == "-c" || arg == "--vehicle-config") vehicle_config_dir = next();
        else if (arg == "--camera") camera_name = next();
        else if (arg == "-o" || arg == "--output") output_dir = next();
        else if (arg == "--lidar-type") lidar_type_override = next();
        else
        {
            std::cerr << "[Args] Unknown argument: " << arg << std::endl;
            return false;
        }
    }

    if (image_path.empty() || pointcloud_path.empty() || settings_path.empty() ||
        vehicle_config_dir.empty() || camera_name.empty() || output_dir.empty())
    {
        std::cerr << "[Args] Missing required argument(s)." << std::endl;
        return false;
    }
    if (lidar_type_override != "auto" && lidar_type_override != "solid" && lidar_type_override != "mech")
    {
        std::cerr << "[Args] --lidar-type must be auto|solid|mech" << std::endl;
        return false;
    }
    return true;
}

void saveDebugPcd(const std::string &output_dir, const std::string &name,
                  const pcl::PointCloud<Common::Point>::Ptr &cloud)
{
    if (!cloud || cloud->empty()) return;
    pcl::io::savePCDFileBinaryCompressed(output_dir + "/" + name, *cloud);
}

void saveDebugPcd(const std::string &output_dir, const std::string &name,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    if (!cloud || cloud->empty()) return;
    pcl::io::savePCDFileBinaryCompressed(output_dir + "/" + name, *cloud);
}

}  // namespace

int main(int argc, char **argv)
{
    std::string image_path, pointcloud_path, settings_path;
    std::string vehicle_config_dir, camera_name, output_dir, lidar_type_override;

    if (!parseArgs(argc, argv, image_path, pointcloud_path, settings_path,
                   vehicle_config_dir, camera_name, output_dir, lidar_type_override))
    {
        printUsage(argv[0]);
        return 1;
    }

    Params params = loadSettingsYaml(settings_path);
    if (!loadCameraFromVehicleConfig(vehicle_config_dir, camera_name, params))
    {
        return 1;
    }

    while (!output_dir.empty() && output_dir.back() == '/') output_dir.pop_back();
    mkdir(output_dir.c_str(), 0755);
    params.output_path = output_dir;

    DataPreprocess data_preprocess(image_path, pointcloud_path, lidar_type_override);
    if (data_preprocess.img_input_.empty() || data_preprocess.cloud_input_->empty())
    {
        ROS_ERROR("[Main] Failed to load image or point cloud, abort.");
        return 1;
    }

    cv::Mat img_input = data_preprocess.img_input_;
    applyFisheyeUndistortIfNeeded(params, img_input);

    QRDetect qr_detect(params);
    LidarDetect lidar_detect(params);

    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    qr_center_cloud->reserve(4);
    qr_detect.detect_qr(img_input, qr_center_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_center_cloud->reserve(4);
    switch (data_preprocess.lidarType())
    {
        case LiDARType::Solid:
            lidar_detect.detect_solid_lidar(data_preprocess.cloud_input_, lidar_center_cloud);
            break;
        case LiDARType::Mech:
            lidar_detect.detect_mech_lidar(data_preprocess.cloud_input_, lidar_center_cloud);
            break;
        default:
            ROS_ERROR("[Main] Unknown LiDAR type.");
            return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_centers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    sortPatternCenters(qr_center_cloud, qr_centers, "camera");
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

    validateTargetGeometry(qr_centers, params.delta_width_circles, params.delta_height_circles, "QR");
    validateTargetGeometry(lidar_centers, params.delta_width_circles, params.delta_height_circles, "LiDAR");

    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0)
    {
        std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
                  << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = "
              << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(data_preprocess.cloud_input_, transformation,
                             qr_detect.cameraMatrix_, qr_detect.distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qr_detect.imageCopy_);

    if (DEBUG)
    {
        saveDebugPcd(output_dir, "debug_filtered_cloud.pcd", lidar_detect.getFilteredCloud());
        saveDebugPcd(output_dir, "debug_plane_cloud.pcd", lidar_detect.getPlaneCloud());
        saveDebugPcd(output_dir, "debug_annulus_cloud.pcd", lidar_detect.getAnnulusOriginalCloud());
        saveDebugPcd(output_dir, "debug_boundary_cloud.pcd", lidar_detect.getBoundaryOriginalCloud());
        saveDebugPcd(output_dir, "debug_aligned_cloud.pcd", lidar_detect.getAlignedCloud());
        saveDebugPcd(output_dir, "debug_edge_cloud.pcd", lidar_detect.getEdgeCloud());
        saveDebugPcd(output_dir, "debug_center_z0_cloud.pcd", lidar_detect.getCenterZ0Cloud());
        saveDebugPcd(output_dir, "debug_aligned_lidar_centers.pcd", aligned_lidar_centers);
    }

    return 0;
}
```

- [ ] **Step 2: 整体重写 `CMakeLists.txt`（先只含 fast_calib 一个 target）**

```cmake
cmake_minimum_required(VERSION 3.10)
project(fast_calib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_BUILD_TYPE Release)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

add_executable(fast_calib src/main.cpp)
target_link_libraries(fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp::yaml-cpp)
```

- [ ] **Step 3: 删除 `package.xml`**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
git rm package.xml
```

- [ ] **Step 4: 真实构建**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
rm -rf build && mkdir build && cd build && cmake .. && make -j$(nproc)
```

Expected: `cmake ..` 找到 PCL/OpenCV/yaml-cpp 三个包，不报 catkin 相关错误；`make` 编译通过，生成 `build/fast_calib`。

- [ ] **Step 5: 单帧样例跑一次**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
rm -rf /tmp/fast_calib_out_single && mkdir -p /tmp/fast_calib_out_single
./build/fast_calib \
  --image "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/camera/encoding_front_fisheye/1782788887590.png" \
  --pointcloud "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/1782788887674.pcd" \
  --settings config/qr_params.yaml \
  -c /home/calib/workspace/project-calib/vehicle_config/L2_data_023 --camera front_fisheye \
  -o /tmp/fast_calib_out_single
echo "exit code: $?"
ls -la /tmp/fast_calib_out_single
cat /tmp/fast_calib_out_single/single_calib_result.txt
```

Expected: 退出码 0（若 QR/LiDAR 检测数量不足 4 个导致排序/几何校验打印警告，只要程序不崩溃、`single_calib_result.txt` 里 `Rcl`/`Pcl` 不是 NaN，就算通过——检测准确率调参不在本次范围内）；`/tmp/fast_calib_out_single/` 下应出现 `single_calib_result.txt`、`colored_cloud.pcd`、`qr_detect.png`、`circle_center_record.txt`、以及 8 个 `debug_*.pcd`。

- [ ] **Step 6: 文件夹合并样例跑一次**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
rm -rf /tmp/fast_calib_out_folder && mkdir -p /tmp/fast_calib_out_folder
./build/fast_calib \
  --image "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/camera/encoding_front_fisheye/1782788887590.png" \
  --pointcloud "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/" \
  --settings config/qr_params.yaml \
  -c /home/calib/workspace/project-calib/vehicle_config/L2_data_023 --camera front_fisheye \
  -o /tmp/fast_calib_out_folder
echo "exit code: $?"
ls -la /tmp/fast_calib_out_folder
```

Expected: 同 Step 5，且控制台日志里 `[pcd_io] Merged 27 pcd files` 之类信息确认走的是合并分支。

- [ ] **Step 7: 提交**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
git add -A src/main.cpp CMakeLists.txt package.xml
git commit -m "$(cat <<'EOF'
去ROS化: main.cpp 改成纯 CLI，CMakeLists.txt 改纯 CMake（fast_calib）

main.cpp 用手写 argv 解析读 --image/--pointcloud/--settings/-c/--camera/
-o/--lidar-type，不再 ros::init、不再常驻 ros::Rate 主循环，DEBUG 中间
点云改成落盘 debug_*.pcd。CMakeLists.txt 去掉 catkin，直接
find_package(PCL/OpenCV/yaml-cpp)；package.xml 删除（纯 CMake 工程不
需要）。fast_calib 已用真实样例数据（单帧 + 27 帧文件夹）验证跑通。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 8: 重写 `src/multi_scene.cpp` + CMakeLists.txt 加 `multi_fast_calib`

**Files:**
- Modify: `src/multi_scene.cpp:1-13,110-120`
- Modify: `CMakeLists.txt`（追加 `multi_fast_calib` target）
- Test: 真实构建 + 用 Task 7 生成的 `circle_center_record.txt` 跑一次

**Interfaces:**
- Consumes: 无新增（只用 `common_lib.h` 的日志宏，不再需要 `Params`/`data_preprocess.hpp`）
- Produces: 可执行文件 `build/multi_fast_calib`，CLI: `./multi_fast_calib -o <output_dir>`

- [ ] **Step 1: 删掉 `src/multi_scene.cpp` 第 1 行 `#include <ros/ros.h>` 和第 13 行 `#include "data_preprocess.hpp"`**

（两行各自单独删除，不需要替换成别的内容；`data_preprocess.hpp` 在这个文件里从未被实际使用，见 Task 8 之前的验证：`grep -n "DataPreprocess\|LiDARType\|data_preprocess" src/multi_scene.cpp` 只命中这一行 include 本身。）

- [ ] **Step 2: 替换 `src/multi_scene.cpp` 第 110-120 行**

原（`int main` 开头到第二次 `params.output_path` 判断结束）：
```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_fast_calib");
    ros::NodeHandle nh;
    Params params = loadParameters(nh);

    if (params.output_path.back() != '/') params.output_path += '/';
    std::string midtxt_path = params.output_path + "circle_center_record.txt";

    if (params.output_path.back() != '/') params.output_path += '/';
    std::string multi_output_path = params.output_path + "multi_calib_result.txt";
```

改为：
```cpp
int main(int argc, char** argv)
{
    std::string output_dir;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if ((arg == "-o" || arg == "--output") && i + 1 < argc)
        {
            output_dir = argv[++i];
        }
    }
    if (output_dir.empty())
    {
        std::cerr << "Usage: " << argv[0] << " -o <output_dir>" << std::endl;
        return 1;
    }
    while (!output_dir.empty() && output_dir.back() == '/') output_dir.pop_back();

    const std::string midtxt_path = output_dir + "/circle_center_record.txt";
    const std::string multi_output_path = output_dir + "/multi_calib_result.txt";
```

其余代码（读取行、解析 block、`SolveRigidTransformWeighted`、写 `fout`）**原样保留，不要改动**。

- [ ] **Step 3: 在 `CMakeLists.txt` 末尾追加 `multi_fast_calib` target**

```cmake

add_executable(multi_fast_calib src/multi_scene.cpp)
target_link_libraries(multi_fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp::yaml-cpp)
```

- [ ] **Step 4: 真实构建**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany/build
cmake .. && make -j$(nproc)
```

Expected: 编译通过，生成 `build/multi_fast_calib`。

- [ ] **Step 5: 造出 >=3 个 block，跑一次**

`multi_fast_calib` 需要 `circle_center_record.txt` 里至少 3 个 block；用 Task 7 已验证过的 `fast_calib` 对同一份样例数据重复跑 3 次（append-only，重复跑同一份数据完全够用，不需要 3 个不同场景）：

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
rm -rf /tmp/multi_calib_out && mkdir -p /tmp/multi_calib_out
for i in 1 2 3; do
  ./build/fast_calib \
    --image "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/camera/encoding_front_fisheye/1782788887590.png" \
    --pointcloud "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/1782788887674.pcd" \
    --settings config/qr_params.yaml \
    -c /home/calib/workspace/project-calib/vehicle_config/L2_data_023 --camera front_fisheye \
    -o /tmp/multi_calib_out
done
./build/multi_fast_calib -o /tmp/multi_calib_out
echo "exit code: $?"
cat /tmp/multi_calib_out/multi_calib_result.txt
```

Expected: 退出码 0；`multi_calib_result.txt` 里 `Rcl`/`Pcl` 不是 NaN（三次跑的是同一份数据，RMSE 应该接近 0，这是预期行为，不代表真实多场景精度）。

- [ ] **Step 6: 提交**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
git add src/multi_scene.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
去ROS化: multi_scene.cpp 改成纯 CLI（只需 -o），CMakeLists 加 multi_fast_calib

multi_fast_calib 本来就只读 circle_center_record.txt，不需要 Params/
DataPreprocess，argv 解析只留 -o/--output 一个参数；解析 block、加权
SVD、写 multi_calib_result.txt 的逻辑不变。用 fast_calib 重复跑 3 次
同一份样例数据验证跑通。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 9: 重写 `src/lidar_center_test.cpp` + CMakeLists.txt 加 `lidar_center_test`

**Files:**
- Modify: `src/lidar_center_test.cpp`（大范围重写：删掉 `loadCloudFromBag`/`hasField`/rosbag includes，简化 `resolveOutputDirectory`，重写 `main`；调试落盘/质检函数原样保留）
- Modify: `CMakeLists.txt`（追加 `lidar_center_test` target）
- Test: 真实构建，跑单帧 + 文件夹

**Interfaces:**
- Consumes: `loadPointCloudInput`（Task 2）、`loadSettingsYaml`（Task 1）、`LidarDetect`（Task 6）
- Produces: 可执行文件 `build/lidar_center_test`，CLI: `./lidar_center_test --pointcloud <path|folder> --settings <path> -o <output_dir> [--lidar-type auto|solid|mech]`

- [ ] **Step 1: 替换 `src/lidar_center_test.cpp` 第 1-27 行（文件头 + includes）**

原：
```cpp
/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

LiDAR-only batch test entry for target annulus center extraction.
*/

#include "data_preprocess.hpp"
#include "lidar_detect.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sys/stat.h>
```

改为：
```cpp
/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

LiDAR-only batch test entry for target annulus center extraction.
*/

#include "lidar_detect.hpp"
#include "pcd_io.hpp"

#include <algorithm>
#include <array>
#include <cctype>
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
#include <sys/stat.h>
```

- [ ] **Step 2: 删掉 `hasField`/`loadCloudFromBag` 两个函数**

删掉原第 31-146 行（`namespace { ` 内的 `hasField(...)` 和 `loadCloudFromBag(...)` 两个完整函数定义），从 `bool hasField(...)` 开始一直删到 `loadCloudFromBag` 函数结尾的 `}`（原第 146 行），紧接着的 `// 将 LiDAR 类型枚举转换为日志可读字符串` 注释和 `lidarTypeName` 函数（原第 148 行起）保留不动。

- [ ] **Step 3: 简化 `resolveOutputDirectory`（原第 213-224 行）**

原：
```cpp
// 解析测试输出目录，兼容未展开的 ROS launch 变量
std::string resolveOutputDirectory(const Params& params)
{
    std::string output_dir = params.output_path;
    if (output_dir.empty() || output_dir.find("$(") != std::string::npos)
    {
        output_dir = "/home/chunran/02_calib_ws/src/FAST-Calib/output";
    }
    output_dir = trimTrailingSlash(output_dir);
    ensureDirectory(output_dir);
    return output_dir;
}
```

改为：
```cpp
// 解析测试输出目录：CLI 必须显式传 -o，这里只需要去掉尾部斜杠、确保目录存在
std::string resolveOutputDirectory(const Params& params)
{
    std::string output_dir = trimTrailingSlash(params.output_path);
    ensureDirectory(output_dir);
    return output_dir;
}
```

- [ ] **Step 4: 把 `outputPrefixForBag` 改名为 `outputPrefixForPointcloud`（原第 227-231 行）**

原：
```cpp
// 根据 bag 所在目录和文件名生成输出文件前缀
std::string outputPrefixForBag(const std::string& bag_path)
{
    return sanitizeFilePart(pathParentName(bag_path) + "_" +
                            stripExtension(pathBaseName(bag_path)));
}
```

改为：
```cpp
// 根据点云路径（单个 .pcd 文件或文件夹）所在目录和文件名生成输出文件前缀
std::string outputPrefixForPointcloud(const std::string& pointcloud_path)
{
    return sanitizeFilePart(pathParentName(pointcloud_path) + "_" +
                            stripExtension(pathBaseName(pointcloud_path)));
}
```

同步修改本文件内两处调用点（`saveDebugCloud`/`saveCenterCoordinates` 函数体里的 `outputPrefixForBag(bag_path)` 调用，形参名 `bag_path` 不用改，只改函数调用名）：

```cpp
// saveDebugCloud 函数体内：
const std::string output_path = output_dir + "/" + outputPrefixForPointcloud(bag_path) + "_debug_cloud.pcd";

// saveCenterCoordinates 函数体内：
const std::string output_path = output_dir + "/" + outputPrefixForPointcloud(bag_path) + "_centers.txt";
```

- [ ] **Step 5: 重写 `main` 函数（原第 681-751 行）**

```cpp
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

    return centers->size() == TARGET_NUM_CIRCLES ? 0 : 1;
}
```

- [ ] **Step 6: 在 `CMakeLists.txt` 末尾追加 `lidar_center_test` target**

```cmake

add_executable(lidar_center_test src/lidar_center_test.cpp)
target_link_libraries(lidar_center_test ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp::yaml-cpp)
```

- [ ] **Step 7: 真实构建**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany/build
cmake .. && make -j$(nproc)
```

Expected: 编译通过，生成 `build/lidar_center_test`。

- [ ] **Step 8: 单帧 + 文件夹各跑一次**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
rm -rf /tmp/lidar_test_out && mkdir -p /tmp/lidar_test_out
./build/lidar_center_test \
  --pointcloud "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/1782788887674.pcd" \
  --settings config/qr_params.yaml -o /tmp/lidar_test_out
echo "single exit code: $?"

./build/lidar_center_test \
  --pointcloud "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/" \
  --settings config/qr_params.yaml -o /tmp/lidar_test_out
echo "folder exit code: $?"

ls -la /tmp/lidar_test_out
```

Expected: 两次都不 crash，打印 `Run type: mech`；`/tmp/lidar_test_out/` 下出现 `*_centers.txt` 和 `*_debug_cloud.pcd`（单帧和文件夹各一份，文件名前缀不同）。退出码 0 或 1 都算通过（1 代表没凑齐 4 个圆心，属于阈值调参范畴，不代表本任务失败）。

- [ ] **Step 9: 提交**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
git add src/lidar_center_test.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
去ROS化: lidar_center_test.cpp 改用 pcd_io.hpp，CMakeLists 加 lidar_center_test

删掉本文件里重复实现的 hasField/loadCloudFromBag（rosbag 解析），改调
pcd_io.hpp::loadPointCloudInput；resolveOutputDirectory 去掉针对未展开
ROS launch 变量的 fallback（-o 现在是必填 CLI 参数）；outputPrefixForBag
改名 outputPrefixForPointcloud 反映点云输入不再限于 bag。调试落盘/质检
函数本体不变。CLI 从位置参数改成具名 flag，与 fast_calib 风格一致。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 10: 文档收尾 + 全量干净重建验证

**Files:**
- Modify: `CLAUDE.md`（"编译"/"运行"/"去 ROS 化耦合点" 几节需要重写以反映新现实）
- Modify: `readme.md`（按仓库约定追加这次改动的变更记录）
- Test: 全量干净重建 + 三个可执行文件各跑一次

**Interfaces:**
- Consumes: 无（纯文档 + 集成验证）
- Produces: 无新代码接口

- [ ] **Step 1: 重写 `CLAUDE.md` 的"编译"一节**

把原"编译"一节（`catkin_make`/`catkin workspace` 相关内容）替换为：

```markdown
## 编译

纯 CMake 工程，不需要 catkin workspace，不需要 ROS：

\`\`\`bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
\`\`\`

依赖：PCL >= 1.10、OpenCV >= 4.0（用到 `opencv_aruco`）、yaml-cpp（本机走系统包 `libyaml-cpp-dev`，`find_package(yaml-cpp)` 导出 `yaml-cpp::yaml-cpp`）。

`CMakeLists.txt` 里三个可执行目标（`add_executable`），**没有 test target、没有 lint target**：

| 目标 | 源文件 | 作用 |
| --- | --- | --- |
| `fast_calib` | `src/main.cpp` | 单场景标定：QR + LiDAR 检测 → SVD 求外参 |
| `multi_fast_calib` | `src/multi_scene.cpp` | 多场景联合标定：读多次单场景结果做加权 SVD |
| `lidar_center_test` | `src/lidar_center_test.cpp` | LiDAR-only 调参工具，不跑相机侧 |
```

- [ ] **Step 2: 重写 `CLAUDE.md` 的"运行"一节**

把原"运行"一节（`roslaunch`/`rosparam`/`rosrun` 相关内容）替换为：

```markdown
## 运行

参数分三处来源：相机内参从 vehicle_config 的 `camera.yaml` 读（`-c <dir> --camera <name>`）；
标定板几何/LiDAR ROI 等算法参数走项目自带的 settings YAML（`--settings <path>`，即瘦身后的
`config/qr_params.yaml`，缺字段直接报错退出，没有编译期默认路径）；图片/点云/输出目录都是
显式 CLI 参数。

单场景标定：

\`\`\`bash
./build/fast_calib \\
  --image <path/to/image.png> \\
  --pointcloud <path/to/frame.pcd 或 path/to/folder/> \\
  --settings config/qr_params.yaml \\
  -c <vehicle_config_dir> --camera <camera_name> \\
  -o <output_dir> \\
  [--lidar-type auto|solid|mech]
\`\`\`

`--pointcloud` 传单个 `.pcd` 就按单帧处理；传一个目录就非递归扫该目录下所有 `.pcd`、按文件名
升序全部累加合并成一份点云再检测（不做降采样/截断，靠检测算法内部的 `auto_roi_voxel_leaf`/
`annulus_voxel_leaf` 兜底）。跑完后**追加**写入 `<output_dir>/circle_center_record.txt`，并生成
`single_calib_result.txt`（FAST-LIVO2 格式外参 + 相机内参）、`colored_cloud.pcd`、`qr_detect.png`，
`DEBUG=1`（默认开）时还会落盘 8 个 `debug_*.pcd` 中间点云（原来是发布到 RViz，现在是文件）。

攒够 ≥3 个场景（对应标定板摆在车前左/正中/右，`pics/multi-scene.jpg`）后跑联合标定：

\`\`\`bash
./build/multi_fast_calib -o <same_output_dir>
\`\`\`

`multi_scene.cpp` 固定**只取 `circle_center_record.txt` 里最后 3 个 block**做加权 SVD，输出
`multi_calib_result.txt`。

LiDAR-only 调参（不需要图像/相机参数，直接吃点云）：

\`\`\`bash
./build/lidar_center_test \\
  --pointcloud <path/to/frame.pcd 或 folder/> \\
  --settings config/qr_params.yaml \\
  -o <output_dir> \\
  [--lidar-type auto|solid|mech]
\`\`\`

输出 `<prefix>_centers.txt`（4 个环心坐标）和 `<prefix>_debug_cloud.pcd`，比跑整套 `fast_calib`
快得多，调 LiDAR 侧阈值时优先用这个。

`calib_data/`、`output/`、`build/` 都在 `.gitignore` 里，不要把跑出来的数据/结果当代码提交。
```

- [ ] **Step 3: 删掉 `CLAUDE.md` 里的"去 ROS 化耦合点"整节**

这一节列的是"待办耦合点"，本次改造已经全部完成，整节删掉（从 `## 去 ROS 化耦合点` 标题开始，到下一个 `## 编码约定` 标题之前的所有内容）。

- [ ] **Step 4: 更新 `CLAUDE.md` 顶部"对话与工作约定"里关于"当前核心开发方向"的描述**

原：
```
- **本仓库当前的核心开发方向是"去 ROS 化"**：把下面「去 ROS 化耦合点」列出的 ROS/catkin/rosbag 依赖逐步剥离，改造成不依赖 ROS 运行时的纯 CLI 工具（具体目标接口——参数怎么传、点云/图像怎么输入——以用户在对话中给出的方案为准，不要自行假设照抄其他项目的设计）。动手改动这部分之前，先跟用户确认接口设计，不要一次性大改。
```

改为：
```
- **本仓库已完成"去 ROS 化"**：三个可执行文件（`fast_calib`/`multi_fast_calib`/`lidar_center_test`）已改成不依赖 ROS 运行时的纯 CLI 工具，纯 CMake 编译，见下方「编译」「运行」两节。设计文档见 `docs/superpowers/specs/2026-07-01-去ros化-design.md`，实施计划见 `docs/superpowers/plans/2026-07-01-去ros化-implementation.md`。
```

- [ ] **Step 5: 更新 `CLAUDE.md` 里"仓库定位"一节关于 catkin 包的描述**

把提到"本仓库现在仍是完整的 ROS1 catkin 包...没有独立 CMake 编译路径，也没有 `-c`/`vehicle_config` 这类 CLI 参数体系"的那句话，改成：

```
本仓库现在是纯 CMake 工程（不依赖 ROS 运行时），相机内参走 vehicle_config 的 `camera.yaml`（`-c`/`--camera`），标定板几何/LiDAR ROI 走项目自带的 settings YAML（`--settings`）。
```

- [ ] **Step 6: 追加 `readme.md` 变更记录**

按 `CLAUDE.md` 里"每次实质改动都要在 readme.md 按日期追加记录"的约定，在 `readme.md` 里 `## 2026-07-01` 小节末尾追加一条：

```markdown
- 完成"去 ROS 化"实施：三个可执行文件（fast_calib/multi_fast_calib/lidar_center_test）
  全部改成纯 CLI + 纯 CMake，删除 package.xml；新增 `include/pcd_io.hpp`（PCD 单帧/文件夹
  合并 + LiDAR 类型探测）和 `include/vehicle_config_reader.hpp`（vehicle_config camera.yaml
  读取 + 鱼眼转 pinhole）；`config/qr_params.yaml` 瘦身为纯标定板几何/LiDAR ROI 参数。已用
  Taijia_001 样例数据（front_fisheye 图像 + front_jt128 单帧/27帧文件夹点云）实跑验证三个
  可执行文件均可正常编译运行。
```

- [ ] **Step 7: 全量干净重建 + 三个可执行文件各跑一次**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
rm -rf build && mkdir build && cd build && cmake .. && make -j$(nproc)
echo "build exit code: $?"

cd /home/calib/音乐/code/FAST-Calib2-sany
rm -rf /tmp/final_check && mkdir -p /tmp/final_check
./build/fast_calib \
  --image "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/camera/encoding_front_fisheye/1782788887590.png" \
  --pointcloud "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/1782788887674.pcd" \
  --settings config/qr_params.yaml \
  -c /home/calib/workspace/project-calib/vehicle_config/L2_data_023 --camera front_fisheye \
  -o /tmp/final_check
./build/multi_fast_calib -o /tmp/final_check
./build/lidar_center_test \
  --pointcloud "calib_data/Taijia_001_20260630110807641Z8_20260630110810414Z8/lidar/front_jt128/" \
  --settings config/qr_params.yaml -o /tmp/final_check
ls -la /tmp/final_check
```

Expected: `make` 全量重建成功（0 报错、0 catkin 相关引用）；三个可执行文件依次跑完都不 crash（`multi_fast_calib` 此时 `circle_center_record.txt` 只有 1 个 block，报 "Parsed blocks < 3" 退出码 1 是预期行为，不是 bug）。

- [ ] **Step 8: 提交**

```bash
cd /home/calib/音乐/code/FAST-Calib2-sany
git add CLAUDE.md readme.md
git commit -m "$(cat <<'EOF'
去ROS化: 更新 CLAUDE.md 编译/运行文档，readme.md 记录本次改动收尾

CLAUDE.md 的"编译"/"运行"两节改成反映纯 CMake + CLI 的新现实，删掉已经
做完的"去 ROS 化耦合点"待办清单；readme.md 按仓库约定追加这次改动的
变更记录。至此 fast_calib/multi_fast_calib/lidar_center_test 三个可执行
文件均已用真实样例数据验证跑通，去 ROS 化改造完成。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Self-Review Notes

- **Spec 覆盖检查**：spec 的"模块改动清单"→ Task 1-9 逐文件对应；"CLI 接口"→ Task 7/8/9 的 argv 解析；"点云输入"→ Task 2；"相机参数+鱼眼"→ Task 3；"settings YAML"→ Task 1；"日志宏 shim"→ Task 1；"调试点云落盘"→ Task 7；"构建系统"→ Task 7/8/9 的 CMakeLists 增量 + Task 10 文档；"错误处理约定"→ 贯穿各任务的必填参数检查/报错退出；"验证方式"→ 每个任务的 Step N 都是真实编译+跑样例数据。无遗漏。
- **占位符检查**：全文没有 TBD/TODO，所有 Modify 步骤都给出完整前后代码块，没有"其余同上"这类模糊指代（除非明确写"原样保留，不要改动"并指明具体行号范围）。
- **类型一致性检查**：`LiDARType`/`PointCloudLoadResult`/`Params`/`DataPreprocess` 构造函数签名/`loadPointCloudInput`/`loadCameraFromVehicleConfig`/`applyFisheyeUndistortIfNeeded`/`loadSettingsYaml` 在各任务间的调用点用词一致，已核对。
