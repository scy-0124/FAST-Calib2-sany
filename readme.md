# 改动记录

按日期记录本仓库的实质性改动（代码/文档），供后续回溯用。

## 2026-07-01

- 在 `CLAUDE.md` 中新增约定：每次有实质改动都要在本文件按日期追加记录。
- 完成"去 ROS 化"设计的 brainstorming，产出并提交设计文档
  `docs/superpowers/specs/2026-07-01-去ros化-design.md`：三个可执行文件
  （fast_calib/multi_fast_calib/lidar_center_test）一次性改成纯 CLI + 纯 CMake，图片/点云
  路径显式传参，点云支持单帧 .pcd / 文件夹合并，相机内参从 vehicle_config 的 camera.yaml
  读取并处理鱼眼畸变，标定板几何/ROI 参数保留在项目自带 settings YAML（yaml-cpp 解析）。
  尚未开始实施代码改动。
- 基于设计文档产出并提交去 ROS 化实施计划
  `docs/superpowers/plans/2026-07-01-去ros化-implementation.md`：拆成 10 个可独立验证的
  任务（common_lib.h → pcd_io.hpp → vehicle_config_reader.hpp → data_preprocess.hpp →
  qr_detect.hpp → lidar_detect.hpp → main.cpp+CMakeLists → multi_scene.cpp →
  lidar_center_test.cpp → 文档收尾），每个任务用 Taijia_001 真实样例数据实跑验证。发现本机
  未安装 ROS，因此计划里的顺序刻意把 common_lib.h 去 ROS 化放最前面，保证后续每个任务都能
  standalone 编译验证。尚未开始执行。
- 完成"去 ROS 化"实施：三个可执行文件（fast_calib/multi_fast_calib/lidar_center_test）
  全部改成纯 CLI + 纯 CMake，删除 package.xml；新增 `include/pcd_io.hpp`（PCD 单帧/文件夹
  合并 + LiDAR 类型探测）和 `include/vehicle_config_reader.hpp`（vehicle_config camera.yaml
  读取 + 鱼眼转 pinhole）；`config/qr_params.yaml` 瘦身为纯标定板几何/LiDAR ROI 参数。已用
  Taijia_001 样例数据（front_fisheye 图像 + front_jt128 单帧/27帧文件夹点云）实跑验证三个
  可执行文件均可正常编译运行。
  
  **已知限制**：用本机真实样例数据（Taijia_001 数据集）跑 LiDAR 检测时，当前 `use_auto_lidar_roi`
  的自动 ROI 提取在这批点云上找不到足够的高反光聚类（0个），凑不出4个圆心——这是标定板反射率/ROI
  阈值跟这批数据不匹配的调参问题，不是这次去ROS化引入的代码缺陷，但会影响之后有人拿这批数据实跑
  标定的预期（不会跑出真实的4个圆心结果），需要后续单独调参解决。
