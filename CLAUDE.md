# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 对话与工作约定

- **本仓库内的对话一律使用中文回复**，不要用英文作答（代码注释、commit message 等可按实际需要保留英文/中文混排）。
- **开始新一轮代码改动前，先确认当前 HEAD 已经 push 到 `origin/main`**（`git status` 看是否 ahead/落后，落后就先 `git push`）。也就是说：先把"上一个版本"备份到远端，再在此基础上做下一步修改，不要让多轮未上传的本地改动堆积在一起。
- **本仓库已完成"去 ROS 化"**：三个可执行文件（`fast_calib`/`multi_fast_calib`/`lidar_center_test`）已改成不依赖 ROS 运行时的纯 CLI 工具，纯 CMake 编译，见下方「编译」「运行」两节。设计文档见 `docs/superpowers/specs/2026-07-01-去ros化-design.md`，实施计划见 `docs/superpowers/plans/2026-07-01-去ros化-implementation.md`。
- **每次对代码/文档有实质改动后，都要在 `readme.md` 里按日期追加一条记录**（新记录写在最上面还是最下面以 `readme.md` 现有格式为准；没有格式约定时按日期从旧到新追加在文件末尾），简要说明改了什么、为什么改。这是改动日志，不要和 `README.md`（如果以后补充项目介绍）混在一起。

## 仓库定位

这是 HKU MARS [FAST-Calib](https://github.com/hku-mars/FAST-Calib) 的**反光环 annulus 标定板版本（FAST-Calib2）**，`git remote origin` 指向 `github.com/scy-0124/FAST-Calib2-sany`。跟同一台机器上 `/home/calib/code/FAST-Calib2`（已去 ROS 化）是两套独立代码，**不要混用两边的运行约定**——本仓库现在是纯 CMake 工程（不依赖 ROS 运行时），相机内参走 vehicle_config 的 `camera.yaml`（`-c`/`--camera`），标定板几何/LiDAR ROI 走项目自带的 settings YAML（`--settings`）。

标定板设计：一块板上同时有 4 个反光环（LiDAR 侧检测用）和 4 个 ArUco 视觉标记（相机侧检测用），几何尺寸统一在 `config/qr_params.yaml` 里配置（`delta_width_circles` / `delta_height_circles` 等）。

## 编译

纯 CMake 工程，不需要 catkin workspace，不需要 ROS：

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

依赖：PCL >= 1.10、OpenCV >= 4.0（用到 `opencv_aruco`）、yaml-cpp（本机走系统包 `libyaml-cpp-dev`，`find_package(yaml-cpp)` 导出 `yaml-cpp::yaml-cpp`）。

`CMakeLists.txt` 里三个可执行目标（`add_executable`），**没有 test target、没有 lint target**：

| 目标 | 源文件 | 作用 |
| --- | --- | --- |
| `fast_calib` | `src/main.cpp` | 单场景标定：QR + LiDAR 检测 → SVD 求外参 |
| `multi_fast_calib` | `src/multi_scene.cpp` | 多场景联合标定：读多次单场景结果做加权 SVD |
| `lidar_center_test` | `src/lidar_center_test.cpp` | LiDAR-only 调参工具，不跑相机侧 |

## 运行

参数分三处来源：相机内参从 vehicle_config 的 `camera.yaml` 读（`-c <dir> --camera <name>`）；
标定板几何/LiDAR ROI 等算法参数走项目自带的 settings YAML（`--settings <path>`，即瘦身后的
`config/qr_params.yaml`，缺字段直接报错退出，没有编译期默认路径）；图片/点云/输出目录都是
显式 CLI 参数。

单场景标定：

```bash
./build/fast_calib \
  --image <path/to/image.png> \
  --pointcloud <path/to/frame.pcd 或 path/to/folder/> \
  --settings config/qr_params.yaml \
  -c <vehicle_config_dir> --camera <camera_name> \
  -o <output_dir> \
  [--lidar-type auto|solid|mech]
```

`--pointcloud` 传单个 `.pcd` 就按单帧处理；传一个目录就非递归扫该目录下所有 `.pcd`、按文件名
升序全部累加合并成一份点云再检测（不做降采样/截断，靠检测算法内部的 `auto_roi_voxel_leaf`/
`annulus_voxel_leaf` 兜底）。跑完后**追加**写入 `<output_dir>/circle_center_record.txt`，并生成
`single_calib_result.txt`（FAST-LIVO2 格式外参 + 相机内参）、`colored_cloud.pcd`、`qr_detect.png`，
`DEBUG=1`（默认开）时还会落盘 8 个 `debug_*.pcd` 中间点云（原来是发布到 RViz，现在是文件）。

攒够 ≥3 个场景（对应标定板摆在车前左/正中/右，`pics/multi-scene.jpg`）后跑联合标定：

```bash
./build/multi_fast_calib -o <same_output_dir>
```

`multi_scene.cpp` 固定**只取 `circle_center_record.txt` 里最后 3 个 block**做加权 SVD，输出
`multi_calib_result.txt`。

LiDAR-only 调参（不需要图像/相机参数，直接吃点云）：

```bash
./build/lidar_center_test \
  --pointcloud <path/to/frame.pcd 或 folder/> \
  --settings config/qr_params.yaml \
  -o <output_dir> \
  [--lidar-type auto|solid|mech]
```

输出 `<prefix>_centers.txt`（4 个环心坐标）和 `<prefix>_debug_cloud.pcd`，比跑整套 `fast_calib`
快得多，调 LiDAR 侧阈值时优先用这个。

`calib_data/`、`output/`、`build/` 都在 `.gitignore` 里，不要把跑出来的数据/结果当代码提交。

## 架构

`src/` 不分子目录，`qr_detect.hpp` / `lidar_detect.hpp` / `data_preprocess.hpp` 是 header-only 算法实现，被三个 `main` 入口直接 include；`include/common_lib.h` 是公共基础设施。

```
main.cpp              单场景入口：DataPreprocess 加载数据 → QRDetect + LidarDetect 各自出 4 个中心
                       → sortPatternCenters 统一排序 → validateTargetGeometry 几何校验
                       → pcl SVD 求 T_cam_lidar → 写 circle_center_record.txt / single_calib_result.txt
multi_scene.cpp        多场景入口：解析 circle_center_record.txt 最后 3 个 block → 自实现加权 SVD（非 pcl）
lidar_center_test.cpp  LiDAR-only 调参入口：--pointcloud 读 PCD 单帧/文件夹，只跑 LidarDetect，另存调试点云/圆心

src/qr_detect.hpp        QRDetect：cv::aruco 检测 4 个标记 → estimatePoseBoard 求板位姿
                          → 反推出相机系下 4 个虚拟环心坐标
src/lidar_detect.hpp      LidarDetect：solid / mech 两条独立检测路径（~2000 行，全仓库最核心的算法文件）
src/data_preprocess.hpp   DataPreprocess：构造即加载——cv::imread 读图像 + pcd_io.hpp::loadPointCloudInput
                          读点云（单帧 .pcd / 文件夹合并；后者按文件名升序累加多帧）；顺带按点云是否
                          带 ring 字段判定 LiDARType（Solid/Mech）
include/common_lib.h      Common::Point（XYZ+intensity+ring，PCL_NO_PRECOMPILE 自定义点类型）、Params
                          结构体 + loadSettingsYaml（yaml-cpp）加载、sortPatternCenters/validateTargetGeometry/
                          computeRMSE/projectPointCloudToImage 等几何与 IO 工具函数
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
- `DEBUG=1`（`common_lib.h:38`）时会落盘 8 个 `debug_*.pcd` 中间点云文件（单场景 `fast_calib` 输出），包含 `filtered`/`plane`/`annulus`/`boundary`/`aligned`/`edge`/`center_z0`/`aligned_lidar_centers` 各阶段的点云、边界点、圆心标记，便于调试 LiDAR 检测效果；关掉 `DEBUG` 不影响标定结果，纯调试用。

## 编码约定

- C++17，`#define PCL_NO_PRECOMPILE` 用自定义 `Common::Point`（XYZ+intensity+ring，`common_lib.h:43-57`）而不是 `pcl::PointXYZI`——所有点云路径都应该用这个类型，annulus 检测全程依赖 `intensity` 字段。
- 没有项目命名空间包裹 `LidarDetect`/`QRDetect`/`DataPreprocess` 这几个核心类；`Common` 命名空间只放点类型。
- 注释以中文为主，算法阈值含义、调参指引写在代码内注释里，不要翻译或删减已有中文注释。
- License：GPLv2（见 `LICENSE`），原作者 Chunran Zheng（HKU MARS）。
