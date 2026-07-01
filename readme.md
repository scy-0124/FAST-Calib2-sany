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
