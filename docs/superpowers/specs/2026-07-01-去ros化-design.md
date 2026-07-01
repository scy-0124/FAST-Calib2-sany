# FAST-Calib2-sany 去 ROS 化设计

日期：2026-07-01

## 背景与目标

本仓库（FAST-Calib2-sany，反光环 annulus 标定板版本）当前是完整的 ROS1 catkin 包：参数走
`rosparam`，点云走 `rosbag`，可视化走 `ros::Publisher` 发 RViz topic，必须放在 catkin
workspace 下用 `catkin_make` 编译。目标是把这一整套 ROS 依赖去掉，改造成不依赖 ROS 运行时、
可以裸 `cmake .. && make` 编译的纯 CLI 工具，图片/点云路径由命令行显式传入。

本次改造范围覆盖三个可执行文件：`fast_calib`（单场景标定）、`multi_fast_calib`（多场景联合
标定）、`lidar_center_test`（LiDAR-only 调参工具），一次性做完，不留 ROS/非 ROS 混合的过渡态。

本次**不改动**标定算法本体：`lidar_detect.hpp` 里 solid/mech 两条检测流水线（ROI裁剪→平面
拟合→强度分割→圆拟合→几何一致性筛选）、`common_lib.h` 里的 `sortPatternCenters` /
`validateTargetGeometry` / `Square` 几何工具、SVD 求外参逻辑，全部原样保留。这次只动"数据怎么
进来、参数怎么传、日志/可视化怎么出去、怎么编译"。

## 样例输入

- 图像：`.../camera/encoding_front_fisheye/1782788887590.png`（front_fisheye 相机，鱼眼镜头）
- 点云单帧：`.../lidar/front_jt128/1782788887674.pcd`（机械式 LiDAR，JT128，PCD 字段
  `x y z intensity ring`，其中 `ring` 是 1 字节 `U` 类型）
- 点云文件夹：`.../lidar/front_jt128/`（27 个按时间戳命名的 `.pcd`，同一段采集的连续帧）
- 相机内参来源：`/home/calib/workspace/project-calib/vehicle_config/L2_data_023/camera.yaml`
  （`camera_name: front_fisheye` 条目，`model_type: fisheye`，畸变系数 `k1..k4`）

## 模块改动清单

```
include/
  common_lib.h              [改] 去掉 pcl_ros/pcl_conversions/tf 头；Params 拆分（相机内参、
                                 路径字段搬出去）；loadParameters(ros::NodeHandle&) 替换成
                                 loadSettingsYaml(path) -> Params（yaml-cpp 读板子几何/ROI/
                                 RANSAC 阈值）；新增 ROS_INFO/ROS_WARN/ROS_ERROR/
                                 ROS_ERROR_STREAM/ROS_WARN_STREAM 非 ROS 宏 shim
  vehicle_config_reader.hpp [新] 读 <vehicle_config_dir>/camera.yaml，按 camera_name 找条目，
                                 解析 fx/fy/cx/cy + 畸变系数 + model_type + width/height；
                                 applyFisheyeUndistortIfNeeded(...)：fisheye/equidistant 时用
                                 cv::fisheye::initUndistortRectifyMap(原K, D, R=I, 原K) +
                                 cv::remap 转成 pinhole 等效图，之后清空畸变系数按 pinhole 走
  pcd_io.hpp                [新] loadPointCloudInput(path, type_override) -> (Common::Point
                                 点云, LiDARType)，统一处理"单个 .pcd" vs "文件夹合并"，替代
                                 原来 rosbag 解析；field-detection 逻辑从 data_preprocess.hpp
                                 挪过来，按字段名读 x/y/z/intensity/ring

src/
  data_preprocess.hpp        [改] 不再依赖 rosbag/livox_ros_driver/sensor_msgs；构造时调用
                                 pcd_io.hpp::loadPointCloudInput + cv::imread，图像与点云
                                 加载解耦成两步，各自独立可失败
  qr_detect.hpp               [改] 构造函数去掉 ros::NodeHandle&，删掉 qr_pub_（ros::Publisher）
  lidar_detect.hpp             [改] 构造函数去掉 ros::NodeHandle&，删掉 7 个 ros::Publisher
                                 成员；内部 ROS_INFO/WARN/ERROR 调用不改（宏 shim 兜底）
  main.cpp                    [改] argv 解析 + vehicle_config/settings 加载 + 鱼眼预处理 +
                                 调用 QRDetect/LidarDetect + 结果落盘；去掉 ros::init/主循环/
                                 RViz 发布，DEBUG 中间点云改成落盘各自 .pcd
  multi_scene.cpp              [改] argv 解析 -o/--output；解析 circle_center_record.txt 的
                                 逻辑不变
  lidar_center_test.cpp        [改] 改用 pcd_io.hpp::loadPointCloudInput，删掉本文件里重复的
                                 loadCloudFromBag；调试落盘逻辑（saveDebugCloud 等）基本不动

CMakeLists.txt / package.xml
  [改] CMakeLists.txt 去掉 find_package(catkin ...) / catkin_package()，改纯 CMake，
      find_package(PCL)/find_package(OpenCV)/find_package(yaml-cpp)，三个可执行文件都链
      yaml-cpp；package.xml 整个删除
```

## CLI 接口

**`fast_calib`**（对应现在的 `roslaunch fast_calib calib.launch`）：
```bash
./fast_calib \
  --image <path/to/image.png> \
  --pointcloud <path/to/frame.pcd 或 path/to/folder/> \
  --settings <path/to/target.yaml> \
  -c <vehicle_config_dir> --camera <camera_name> \
  -o <output_dir> \
  [--lidar-type auto|solid|mech]     # 默认 auto
```

**`multi_fast_calib`**：
```bash
./multi_fast_calib -o <output_dir>
```

**`lidar_center_test`**：
```bash
./lidar_center_test \
  --pointcloud <path/to/frame.pcd 或 folder/> \
  --settings <path/to/target.yaml> \
  -o <output_dir> \
  [--lidar-type auto|solid|mech]
```

必填 flag 缺一个就打印 usage 到 stderr 并 `return 1`；不设默认值静默兜底。

## 点云输入：单帧 / 文件夹合并 + LiDAR 类型判定

`pcd_io.hpp::loadPointCloudInput(path, type_override)`：

1. `stat(path)` 判断目录还是文件：
   - **目录**：非递归扫直接子项，挑后缀为 `.pcd` 的文件，按文件名字符串升序排序（样例文件名
     是时间戳数字，字符串序=数值序），依次读入后累加进同一个 `Common::Point` 点云。不做体素
     降采样/时间窗口截断——和现在 rosbag 把整段时间窗口内所有帧累加的行为等价，降采样交给
     检测算法内部现有的 `auto_roi_voxel_leaf`/`annulus_voxel_leaf`。
   - **单个 `.pcd` 文件**：只读这一帧。
2. 每帧读入后做字段探测：查 `pcl::PCLPointCloud2::fields` 里有没有 `ring`、`intensity`
   （无 `intensity` 时退回 `reflectivity`，都没有则填 0）。字段的具体存储类型/字节数不固定
   （样例 `ring` 是 1 字节 `U`），统一读出数值后 `static_cast` 到 `Common::Point` 对应字段。
3. LiDAR 类型：`type_override == auto` 时，合并范围内**任意一帧**探测到 `ring` 字段就判定
   `Mech`，否则 `Solid`（用"任意一帧"而不是"第一帧"，避免个别帧字段异常导致误判）；
   `type_override` 显式传 `solid`/`mech` 时直接使用，跳过探测。

`DataPreprocess` 构造函数不再是"一次性读图+读点云"，图像（`cv::imread`）与点云
（`loadPointCloudInput`）各自独立可失败，任一失败都打印错误后返回。`lidar_center_test`
直接调 `loadPointCloudInput`，不经过 `DataPreprocess`，不加载图像。

## 相机参数（vehicle_config）+ 鱼眼处理

- `vehicle_config_reader.hpp` 按 `-c <vehicle_config_dir> --camera <camera_name>` 拼出
  `<dir>/camera.yaml`，yaml-cpp 解析成 list，按 `camera_name` 找条目，取
  `camera_model_param` 里的 `fx/fy/cx/cy` + 畸变系数（`model_type: rational/pinhole` 时
  `k1,k2,p1,p2[,k3...]`；`model_type: fisheye/equidistant` 时 `k1,k2,k3,k4`）+
  `width/height`。**不读** `camera_to_body_*`（车体外参）——这次目标只是 `T_cam_lidar`，
  不涉及往 vehicle_config 写车体外参。
- `applyFisheyeUndistortIfNeeded`（`main.cpp` 里紧跟 `DataPreprocess` 之后、`QRDetect`
  构造之前调用）：`model_type` 属于 fisheye/equidistant 族时，用**原始 K**（不重估）+ 4
  系数 D + `R=I` 调 `cv::fisheye::initUndistortRectifyMap` 生成 map，`cv::remap` 得到
  pinhole 等效图；之后 `Params` 畸变系数清零、`camera_model` 标记 pinhole，图像替换成
  undistort 后的版本，尺寸不变。其他 `model_type` 跳过这一步。这一步只影响 2D 投影，
  `T_cam_lidar`（3D 刚体外参）不受影响。
  - `initUndistortRectifyMap` 的 `img_size` 用**实际加载图像**（`cv::imread` 返回的
    `cv::Mat` 的 `cols/rows`）而不是 `camera.yaml` 里的 `width/height` 元数据字段——后者
    只作为信息展示/校验用，真正参与计算的尺寸以实际图像为准，避免元数据与实际图片不一致时
    算出错误的 remap。
  - `model_type: rational`（如样例 vehicle_config 里的 `rear_wide`，8 系数 `k1..k6,p1,p2`）
    目前和去 ROS 化之前一样，只截取 `k1,k2,p1,p2` 喂给 `cv::undistort`，`k3~k6` 不参与计算
    ——这是沿用原有 pinhole+径向切向畸变模型的既有限制，不在本次改造范围内一并修正。本次
    样例用的 `front_fisheye` 是 fisheye 族，不受此限制影响。

## settings YAML（板子几何 / LiDAR ROI）

`config/qr_params.yaml` 瘦身版：去掉相机内参（`fx/fy/cx/cy/k1/k2/p1/p2`）和路径字段
（`image_path/bag_path/lidar_topic/output_path`），只保留：
```yaml
marker_size / delta_width_qr_center / delta_height_qr_center
delta_width_circles / delta_height_circles / circle_radius / annulus_half_width
board_width / board_height / board_roi_margin / board_roi_depth
auto_roi_voxel_leaf / annulus_voxel_leaf / min_detected_markers
use_auto_lidar_roi / x_min / x_max / y_min / y_max / z_min / z_max
```
`loadSettingsYaml(path)` 用 yaml-cpp 按同名 key 读，**缺字段报错退出**（不像原来 `nh.param`
那样静默落到硬编码默认值，避免"传错文件却在跑一套完全不知情的默认阈值"）；不设编译期默认
路径，必须显式 `--settings` 传入。

## 日志宏 shim

`common_lib.h` 新增（替代 `<ros/ros.h>` 提供的宏，保证现有全部调用点不用改一行）：
```cpp
#define ROS_INFO(...)          do { std::printf(__VA_ARGS__); std::printf("\n"); } while(0)
#define ROS_WARN(...)          do { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while(0)
#define ROS_ERROR(...)         do { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while(0)
#define ROS_ERROR_STREAM(x)    do { std::cerr << x << std::endl; } while(0)
#define ROS_WARN_STREAM(x)     do { std::cerr << x << std::endl; } while(0)
```

## 调试点云落盘

替代 `main.cpp` 里 `ros::Publisher` + `ros::Rate` 主循环：`LidarDetect`/`QRDetect` 已有的
getter（`getFilteredCloud()`/`getPlaneCloud()`/`getAnnulusOriginalCloud()`/
`getBoundaryOriginalCloud()`/`getAlignedCloud()`/`getEdgeCloud()`/`getCenterZ0Cloud()`）不
改，`main.cpp` 里原来"发布到 topic"的地方改成挨个 `pcl::io::savePCDFileBinaryCompressed`
落到 `<output_dir>/debug_*.pcd`，写完即退出（不再有常驻循环）：
```
debug_filtered_cloud.pcd, debug_plane_cloud.pcd, debug_annulus_cloud.pcd,
debug_boundary_cloud.pcd, debug_aligned_cloud.pcd, debug_edge_cloud.pcd,
debug_center_z0_cloud.pcd, debug_aligned_lidar_centers.pcd
```
`DEBUG` 宏含义不变（`DEBUG=1` 才落这些盘），已有的 `colored_cloud.pcd`/`qr_detect.png`/
`single_calib_result.txt`（`saveCalibrationResults`）不受影响。

## 构建系统

```cmake
cmake_minimum_required(VERSION 3.10)
project(fast_calib)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
# 三个可执行目标不变：fast_calib / multi_fast_calib / lidar_center_test
# 各自 target_link_libraries(... ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp::yaml-cpp)
```
不再 `find_package(catkin ...)`，不再 `catkin_package()`。仓库不需要放进 `catkin_ws/src`，
直接 `mkdir build && cd build && cmake .. && make -j$(nproc)`。`package.xml` 整个删除
（`test_depend rostest/rosbag` 那两行历史遗留声明也一并消失）。CLAUDE.md "编译" 一节需要
跟着重写。

## 错误处理约定

延续现有风格（打印错误 + 提前 `return`/退出码，不用异常）：
- CLI 必填参数缺失/无法解析 → usage 到 stderr，`return 1`。
- 文件不存在/打不开（图像、pcd、camera.yaml、settings yaml）→ `ROS_ERROR_STREAM` 一行
  说明具体是哪个路径，`return 1`。
- `camera.yaml` 里找不到指定 `camera_name` → 报错退出，列出该文件里实际有哪些
  `camera_name` 供排查。
- settings yaml 缺字段 → 报错退出，点名缺的 key。

## 验证方式

本仓库历来没有 test target，`make` 是唯一编译入口；这次也不新增自动化测试。"测试"手段是
拿样例数据实跑：
```bash
./fast_calib --image .../1782788887590.png --pointcloud .../front_jt128/1782788887674.pcd \
  --settings config/qr_params.yaml -c .../L2_data_023 --camera front_fisheye -o /tmp/out
```
检查：程序不 core dump、退出码 0、`/tmp/out/single_calib_result.txt` 里 `Rcl/Pcl` 数值不
是 NaN、控制台打印的 RMSE 在合理量级、`colored_cloud.pcd` 拖进 CloudCompare 能看到点云跟
图像颜色对上；再用 `--pointcloud` 指向那个 27 帧的文件夹跑一遍确认合并分支也能正常出结果。
`multi_fast_calib`/`lidar_center_test` 同理拿样例数据各跑一次。
