# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 对话与工作约定

- **本仓库内的对话一律使用中文回复**，不要用英文作答（代码注释、commit message 等可按实际需要保留英文/中文混排）。
- **开始新一轮代码改动前，先确认当前 HEAD 已经 push 到 `origin/main`**（`git status` 看是否 ahead/落后，落后就先 `git push`）。也就是说：先把"上一个版本"备份到远端，再在此基础上做下一步修改，不要让多轮未上传的本地改动堆积在一起。
- **本仓库当前的核心开发方向是"去 ROS 化"**：把下面「去 ROS 化耦合点」列出的 ROS/catkin/rosbag 依赖逐步剥离，改造成不依赖 ROS 运行时的纯 CLI 工具（具体目标接口——参数怎么传、点云/图像怎么输入——以用户在对话中给出的方案为准，不要自行假设照抄其他项目的设计）。动手改动这部分之前，先跟用户确认接口设计，不要一次性大改。
- **每次对代码/文档有实质改动后，都要在 `readme.md` 里按日期追加一条记录**（新记录写在最上面还是最下面以 `readme.md` 现有格式为准；没有格式约定时按日期从旧到新追加在文件末尾），简要说明改了什么、为什么改。这是改动日志，不要和 `README.md`（如果以后补充项目介绍）混在一起。

## 仓库定位

这是 HKU MARS [FAST-Calib](https://github.com/hku-mars/FAST-Calib) 的**反光环 annulus 标定板版本（FAST-Calib2）**，`git remote origin` 指向 `github.com/scy-0124/FAST-Calib2-sany`。跟同一台机器上 `/home/calib/code/FAST-Calib2`（已去 ROS 化）是两套独立代码，**不要混用两边的运行约定**——本仓库现在仍是完整的 **ROS1 catkin 包**（`package.xml` 里 `<buildtool_depend>catkin</buildtool_depend>`），一切输入/输出都走 ROS（`rosparam` 传参、`rosbag` 读点云、`roslaunch` 启动），没有独立 CMake 编译路径，也没有 `-c`/`vehicle_config` 这类 CLI 参数体系。

标定板设计：一块板上同时有 4 个反光环（LiDAR 侧检测用）和 4 个 ArUco 视觉标记（相机侧检测用），几何尺寸统一在 `config/qr_params.yaml` 里配置（`delta_width_circles` / `delta_height_circles` 等）。

## 编译

必须放在某个 catkin workspace 的 `src/` 目录下，用 catkin 编译，**不能**像去 ROS 版本那样裸 `cmake .. && make`：

```bash
cd <catkin_ws>
catkin_make          # 或 catkin build
source devel/setup.bash
```

依赖：ROS1（`roscpp` / `sensor_msgs` / `pcl_ros` / `pcl_conversions` / `geometry_msgs` / `livox_ros_driver` 等，见 `package.xml`）、PCL >= 1.10、OpenCV >= 4.0（用到 `opencv_aruco`）。

`CMakeLists.txt` 里只有三个可执行目标（`add_executable`），**没有 test target、没有 lint target**：

| 目标 | 源文件 | 作用 |
| --- | --- | --- |
| `fast_calib` | `src/main.cpp` | 单场景标定：QR + LiDAR 检测 → SVD 求外参 |
| `multi_fast_calib` | `src/multi_scene.cpp` | 多场景联合标定：读多次单场景结果做加权 SVD |
| `lidar_center_test` | `src/lidar_center_test.cpp` | LiDAR-only 调参工具，不跑相机侧 |

`package.xml` 里声明了 `test_depend rostest/rosbag`，但仓库里没有实际测试代码，是历史遗留声明。

## 运行

参数**全部**通过 `rosparam` 从 `config/qr_params.yaml` 加载（`Params loadParameters(ros::NodeHandle&)`，`include/common_lib.h:76`），一份 YAML 里混合了相机内参、标定板几何尺寸、LiDAR ROI、rosbag/图像路径、输出路径——没有分文件、没有 CLI 覆盖机制（`lidar_center_test` 例外，见下）。

单场景标定：

```bash
roslaunch fast_calib calib.launch
```

`launch/calib.launch` 内部就是 `rosparam load config/qr_params.yaml` + 启动 `fast_calib` 节点，跑完后**追加**写入 `<output_path>/circle_center_record.txt`（每次一个 `time / lidar_centers / qr_centers` 三行 block），并生成 `single_calib_result.txt`（FAST-LIVO2 格式外参 + 相机内参）、`colored_cloud.pcd`、`qr_detect.png`。

攒够 ≥3 个场景（对应标定板摆在车前左/正中/右，`pics/multi-scene.jpg`）后跑联合标定：

```bash
roslaunch fast_calib multi_calib.launch
```

`multi_scene.cpp` 固定**只取 `circle_center_record.txt` 里最后 3 个 block**做加权 SVD，输出 `multi_calib_result.txt`。

LiDAR-only 调参（不需要图像，直接吃 bag + topic，命令行传参覆盖 `bag_path`/`lidar_topic`）：

```bash
rosparam load config/qr_params.yaml /
rosrun fast_calib lidar_center_test <bag_path> <lidar_topic> [auto|solid|mech]
```

输出 `<prefix>_centers.txt`（4 个环心坐标）和 `<prefix>_debug_cloud.pcd`（按 intensity 伪彩色的板点云 + 绿色 annulus 点 + 红色边界点 + 白色圆心球标记，可直接拖进 `pcl_viewer`/CloudCompare 看检测效果），比跑整套 `fast_calib` 快得多，调 LiDAR 侧阈值时优先用这个。

`calib_data/`、`output/`、`build/` 都在 `.gitignore` 里，不要把跑出来的数据/结果当代码提交。

## 架构

`src/` 不分子目录，`qr_detect.hpp` / `lidar_detect.hpp` / `data_preprocess.hpp` 是 header-only 算法实现，被三个 `main` 入口直接 include；`include/common_lib.h` 是公共基础设施。

```
main.cpp              单场景入口：DataPreprocess 加载数据 → QRDetect + LidarDetect 各自出 4 个中心
                       → sortPatternCenters 统一排序 → validateTargetGeometry 几何校验
                       → pcl SVD 求 T_cam_lidar → 写 circle_center_record.txt / single_calib_result.txt
multi_scene.cpp        多场景入口：解析 circle_center_record.txt 最后 3 个 block → 自实现加权 SVD（非 pcl）
lidar_center_test.cpp  LiDAR-only 调参入口：直接读 bag，只跑 LidarDetect，另存调试点云/圆心

src/qr_detect.hpp        QRDetect：cv::aruco 检测 4 个标记 → estimatePoseBoard 求板位姿
                          → 反推出相机系下 4 个虚拟环心坐标
src/lidar_detect.hpp      LidarDetect：solid / mech 两条独立检测路径（~2000 行，全仓库最核心的算法文件）
src/data_preprocess.hpp   DataPreprocess：构造即加载——cv::imread 读图像（与 bag 完全无关，需另外单独提供）
                          + rosbag 按 lidar_topic 过滤全部消息、累加成一份点云（非单帧，是整段 bag 时间
                          窗口内的多帧叠加，用于给稀疏点云"加密度"）；顺带按点云是否带 ring 字段判定
                          LiDARType（Solid/Mech）
include/common_lib.h      Common::Point（XYZ+intensity+ring，PCL_NO_PRECOMPILE 自定义点类型）、Params
                          结构体 + rosparam 加载、sortPatternCenters/validateTargetGeometry/computeRMSE/
                          projectPointCloudToImage 等几何与 IO 工具函数
```

### LiDAR 检测：solid vs mech

`LidarDetect` 按 `DataPreprocess` 探测出的 `LiDARType` 走完全独立的两条流水线（`detect_solid_lidar` / `detect_mech_lidar`，`lidar_detect.hpp:1969` / `:1875`），但共享同一套底层工具：

1. **ROI 裁剪**：`use_auto_lidar_roi: true` 时用 `extractAutoBoardRoi` 基于高反光聚类 + 已知板几何自动定位整块板；否则用 `manualPassThroughFilter` 走 YAML 里手动配的 `x/y/z_min/max`。
2. **平面拟合**：RANSAC 拟合标定板所在平面，后续只在平面内点（`planeDistance` 阈值内）里找环。
3. **强度分割**：`computeHighIntensityThreshold` 用 Otsu 直方图自适应算高反光阈值，区分反光环点与板面背景点；mech 路径额外按 `ring` 顺序找强度跳变点做环边界（`extractRingIntensityBoundaryPointsNearPlane`）。
4. **鲁棒圆拟合**：`fitCircleRobust` / `fitFixedRadiiConcentricCenterRobust` 用 Huber 加权最小二乘拟合（固定内外半径）同心圆圆心，对残差大的点自动降权。
5. **几何一致性筛选**：`selectGeometryConsistentCentersByDistances` 按 YAML 里的板真实尺寸（`delta_width_circles`/`delta_height_circles`）从候选聚类里挑出真正对应 4 个环心的组合，剔除误检的高反光物体。

调参时想跳过相机侧、只看 LiDAR 检测效果，用 `lidar_center_test`（见上方"运行"）。

### 输出与坐标系约定

- `saveTargetHoleCenters`（`common_lib.h:244`）以 **追加（`ios::app`）** 方式写 `circle_center_record.txt`——这是单场景与多场景之间唯一的数据交接点，多场景标定不重新跑检测，只读这个文本文件。
- 单场景外参写 `single_calib_result.txt`，多场景写 `multi_calib_result.txt`，两者都是人手写的 **FAST-LIVO2 格式**（`Rcl`/`Pcl` 三行矩阵文本，非 YAML/JSON，改格式时要注意下游是否有解析脚本依赖这个格式）。
- `DEBUG=1`（`common_lib.h:37`）时 `main.cpp` 主循环会持续发布一堆中间点云到 RViz topic（`filtered_cloud`/`plane_cloud`/`annulus_cloud`/...），配 `rviz_cfg/fast_livo2.rviz` 用；关掉这些发布不影响标定结果，纯调试用。

## 去 ROS 化耦合点

去 ROS 化时需要重点处理、当前散落在各文件里的 ROS 依赖：

- **参数输入**：`Params` 只能从 `ros::NodeHandle::param` 读（`common_lib.h:76-111`），没有独立的 YAML 解析或 CLI 解析路径。
- **点云输入**：`data_preprocess.hpp` 和 `lidar_center_test.cpp` 里各自独立实现了一遍 `rosbag::Bag` + `rosbag::View` + `TopicQuery` 解析（Livox `livox_ros_driver::CustomMsg` 和标准 `sensor_msgs::PointCloud2` 两条分支），逻辑重复、且强依赖 `rosbag`/`pcl_conversions`/`sensor_msgs` 几个 ROS 包。
- **日志**：直接用 `ROS_INFO`/`ROS_WARN`/`ROS_ERROR`/`ROS_ERROR_STREAM`（未封装成 shim 宏），全仓库到处都是，替换量大。
- **可视化**：`LidarDetect` 内部持有一堆 `ros::Publisher` 成员，`main.cpp` 主循环靠 `ros::Rate` + `ros::spinOnce` 常驻发布 RViz 消息。
- **消息类型互转**：`pcl::toROSMsg`/`pcl_ros` 依赖贯穿 `main.cpp` 输出逻辑。
- **构建系统**：`CMakeLists.txt` 走 `find_package(catkin REQUIRED COMPONENTS ...)` + `catkin_package()`，整体挂在 catkin 元编译系统下，不是独立 CMake 工程。

## 编码约定

- C++17，`#define PCL_NO_PRECOMPILE` 用自定义 `Common::Point`（XYZ+intensity+ring，`common_lib.h:43-57`）而不是 `pcl::PointXYZI`——所有点云路径都应该用这个类型，annulus 检测全程依赖 `intensity` 字段。
- 没有项目命名空间包裹 `LidarDetect`/`QRDetect`/`DataPreprocess` 这几个核心类；`Common` 命名空间只放点类型。
- 注释以中文为主，算法阈值含义、调参指引写在代码内注释里，不要翻译或删减已有中文注释。
- License：GPLv2（见 `LICENSE`），原作者 Chunran Zheng（HKU MARS）。
