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
  的自动 ROI 提取在这批点云上找不到足够的高反光聚类（单帧场景为 0 个，27 帧文件夹合并场景为 3 个），凑不出
  4 个圆心——这是标定板反射率/ROI 阈值跟这批数据不匹配的调参问题，不是这次去ROS化引入的代码缺陷，但会影响
  之后有人拿这批数据实跑标定的预期（不会跑出真实的4个圆心结果），需要后续单独调参解决。

## 2026-07-02

- "去 ROS 化"10 个实施任务全部完成并逐个通过 task-scoped code review（每个任务都有实现+审查
  两轮，累计对 5 个任务的审查发现做过修复），随后做了一次最终整支分支 review（`8cc01d6..4ff1219`，
  14 个 commit），发现并修复：
  - **Important**：`fast_calib` 在 QR/LiDAR 检测数量不足 4 个（标定实质失败）时，此前仍会
    `return 0` 并把无意义的外参写进 `single_calib_result.txt`，跟 `lidar_center_test` 早已建立的
    "检测失败用非 0 退出码报告"约定不一致，可能让用 `$?` 判断标定是否成功的下游脚本/CI 把"彻底失败"
    误判为成功。已修复：检测数量不足时打印错误并 `return 1`，不再落盘垃圾结果。
  - 清理了去 ROS 化完成后已经彻底死掉、但仍被 git 跟踪的 ROS 遗留文件：`launch/calib.launch`、
    `launch/multi_calib.launch`、`rviz_cfg/fast_livo2.rviz`、`include/CustomMsg.h`、
    `include/CustomPoint.h`（均已确认无任何源文件引用）。
  - 更新了项目主 `README.md`（原上游英文文档）的 Prerequisites/Run Examples/Standalone LiDAR
    Center Extraction Test 三节，把 `roslaunch`/`rosrun`/`rosparam`/`.bag` 相关的旧运行说明换成
    新的 `./build/fast_calib --image ... --pointcloud ... -c ... -o ...` 等 CLI 用法，与 `CLAUDE.md`
    保持一致。
- PR 建好后用真实生产数据（Taijia_001 另一批场景 + 用户自备的 vehicle_config camera.yaml +
  手动指定的 LiDAR ROI）实测：QR 侧检测完全成功（4/4 标记，几何 RMSE ~1e-5 mm），LiDAR 侧凑到了
  半径完全匹配的环但没凑够 4 个，`fast_calib` 正确报错退出（验证了上面那条 Important 修复在真实
  失败场景下确实生效：退出码 1、不落盘垃圾结果）。测试中发现新问题：`fast_calib` 检测失败时会在
  写 7 个 LiDAR 过程调试点云（`debug_filtered/plane/annulus/boundary/aligned/edge/center_z0_cloud.pcd`）
  **之前**就已经 `return 1` 退出，导致失败时反而看不到任何调试点云——而这几个过程点云本来就只依赖
  `LidarDetect` 自身的检测流水线、不依赖 QR-LiDAR 是否配对成功。已修复：把这 7 个的落盘代码挪到
  检测数量校验之前，不管最终成功与否都会落盘（落盘路径仍是用户传入的 `-o` 目录，没有新增参数）；
  只有依赖 SVD 外参才能算出来的 `debug_aligned_lidar_centers.pcd` 继续保持只在成功时落盘。
  `lidar_center_test` 本来就是"无论成功失败都落盘调试点云"的设计（专门给 LiDAR 侧调参用），
  确认过不需要改。
- 最终 review 里还记录了几条不影响合并、留作后续跟进的 Minor 项：`common_lib.h` 的
  `loadSettingsYaml` 字段类型写错时会 `std::terminate` 而非优雅报错退出；`lidar_center_test` 不校验
  `--lidar-type` 取值合法性；三个可执行文件的参数错误处理风格（退出码/措辞）不完全统一；
  `lidar_center_test.cpp` 里个别函数形参仍叫 `bag_path`（语义上已经不准确，纯命名遗留）。
