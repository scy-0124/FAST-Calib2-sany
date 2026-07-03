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
- `lidar_center_test` 新增完整检测流水线中间点云落盘（`src/lidar_detect.hpp` + `src/lidar_center_test.cpp`）：
  之前只落盘 `plane`/`annulus`/`boundary`/最终圆心合成的一份 `debug_cloud.pcd`，聚类候选和逐个圆拟合
  结果完全看不到。现在无论最终 4 个圆心是否凑齐，都会在 `-o` 输出目录下按点云前缀建一个
  `<prefix>_pipeline/` 子目录（不新增 CLI 参数），把流水线每一步按产生顺序落盘：
  `00_raw_cloud`（原始输入点云）→ `01_roi_filtered`（ROI 裁剪后）→ `02_plane_cloud`（平面 RANSAC 内点）→
  `03_aligned_plane`（对齐到 Z=0 后的板面）→ `04_annulus_original`（原始坐标系高反 annulus/边界点）→
  `05_boundary_original`（仅 solid 路径会有内容，mech 路径始终为空——mech 走的是 04/06 这条线）→
  `06_edge_aligned`（对齐坐标系、送入聚类前的降采样边界点）→ `07_cluster_NN`（每个候选聚类原始点，
  绿色=被采纳为候选圆心/黄色=圆拟合成功但被半径或误差门限剔除/红色=圆拟合本身没收敛）→
  `08_circle_fit_NN`（每个圆拟合成功的聚类叠加拟合圆环可视化：灰=聚类点，青/品红=拟合内外圆环，
  白/橙=圆心，橙表示该候选后来被剔除）→ `09_center_z0`（Z=0 平面上最终选中的圆心，仅成功时有）。
  空点云（比如 mech 数据下的 `05_boundary_original`，或检测在更早步骤就失败导致后续阶段从未产生）
  直接跳过不写文件，不当错误。`lidar_detect.hpp` 侧新增了 `LidarDetect::ClusterFitDebug`
  结构体和 `getLastClusterFitRecords()` getter，纯增量式在三处圆拟合函数
  （`fitAnnulusCentersFromClusters`/`fitConcentricAnnulusCentersFromClusters`/
  `fitSolidBoundaryConcentricCentersFromClusters`）内部记录聚类点+拟合结果，不改动任何检测判定逻辑；
  由于 `detect_mech_lidar`/`detect_solid_lidar` 内部同一批聚类函数可能被调用两次（插值边界 vs
  高反侧边界；单圆 vs 边界同心圆），额外加了一层"每次尝试快照、最后按实际胜出的那次结果覆盖回成员变量"
  的逻辑，确保落盘看到的聚类/圆拟合数据跟最终选用的那次尝试一致，不会张冠李戴。用真实 Taijia_001
  mech 数据（`front_jt128` 文件夹合并 + 手动 ROI）实跑验证：高反侧尝试聚出 2 个候选聚类且都被采纳
  （最终仍因只有 2/4 圆心而按预期返回退出码 1），成功落盘 `00/01/02/03/04/06` + 2 份
  `07_cluster_*`/`08_circle_fit_*`，`05_boundary_original`/`09_center_z0` 正确留空；另用自动 ROI 在
  更早的高反聚类阶段就失败的单帧数据验证了"检测在很早就失败时只落盘 `00_raw_cloud`，不崩溃"。

## 2026-07-03

- 修复 `-o` 输出目录父目录不存在时进程直接 core dump 的问题。用户实跑
  `lidar_center_test --pointcloud ... -o calib_data/Taijia_xxx/output ...`（`calib_data/` 在
  `.gitignore` 里，worktree 中本来就没有）时，检测正常跑完后落盘阶段直接
  `terminate called after throwing an instance of 'pcl::IOException'` 崩溃退出。根因两层：
  （1）全仓库所有 `mkdir()` 都是单层、不检查返回值，父目录缺失就静默创建失败——`lidar_center_test`
  尤其严重，目录校验只在第一次真正存文件时才惰性触发（`saveCenterCoordinates`/`saveDebugCloud`/
  `savePipelineStages` 内部），也就是**检测跑完之后**才发现目录建不起来；
  （2）`pcl::io::savePCDFile*` 系列在目标目录不存在时会 `throw pcl::IOException`（确认自 PCL 1.14
  头文件：空点云只是 `PCL_WARN` 不崩，但 `open()` 失败会抛），全仓库没有任何 `try/catch`，异常一路
  冒到 `std::terminate`。已确认 7 处未加保护的调用点：`src/main.cpp` 的 `saveDebugPcd()` 两个重载、
  `include/common_lib.h` 的 `saveCalibrationResults()`、`src/lidar_center_test.cpp` 的
  `savePipelineCloud`/`saveClusterClouds`/`saveCircleFitVisualizations`/`saveDebugCloud`（后四个是
  昨天新加的过程点云落盘代码贡献的）。
  修复：`include/common_lib.h` 新增共享的 `ensureOutputDirectory()`（基于 `std::filesystem::
  create_directories` 递归创建 + `is_directory`/`access(W_OK)` 校验存在性和可写性），三个可执行文件
  的 `main()` 都在 CLI 解析完成后、检测/加载开始前调用它（`lidar_center_test.cpp` 原来是三者里校验
  最晚的一个，这次把校验提到了最前面；`multi_scene.cpp` 原来完全没有目录创建，顺带补上做行为统一）；
  上述 7 处未加保护的 `pcl::io::savePCDFile*` 调用点逐一补了 `try/catch`，异常统一转成打印错误+按
  原有失败路径返回，不引入新的错误处理风格。用真实场景验证：父目录缺失但可创建时现在会静默递归建好、
  检测正常继续跑、退出码按检测结果是 0/1（不再崩溃）；目标路径被普通文件占用、目录只读两种"真正建不起来"
  的场景验证了干净报错+退出码 1、不 core dump；`fast_calib`/`multi_fast_calib` 用同样的"父目录缺失"
  场景各验证一遍，行为与 `lidar_center_test` 一致；另外重跑了一遍此前验证过的成功/部分成功场景
  （2/4 候选聚类），确认输出文件名、内容、退出码跟修复前完全一致，没有引入回归。

- 按用户要求放宽标定板平面 RANSAC 拟合容差：`lidar_detect.hpp` 的 `fitBoardPlaneFromRoi()` 里
  `fitPlaneRansac` 距离阈值 0.01→0.03m，紧接着 `extractPlaneInliers` 提取 `plane_cloud_` 的阈值
  0.015→0.03m（两处必须一起改，只改前者的话最终进 `plane_cloud_` 的点还是按旧阈值卡，不生效）。
  用 `Taijia_001_20260630110807641Z8` 的 `front_jt128` 数据对比新旧阈值：旧阈值下该场景本来就只能
  拼出 1 个候选聚类（4 环里离散边界点总量不够 `clusterMechanicalAnnulusBoundaryCloud` 的
  `minClusterSize=80` 门槛），新阈值下变成 0 个候选聚类，两者都无法凑够 4 个圆心——确认了这次放宽
  没有让本来能检出的场景变得检不出，该场景的失败另有原因（见下一条）。

- 修复低动态范围（无饱和高反回波）点云下 annulus 检测阈值算冒高、把环点自己滤掉的问题。
  `computeHighIntensityThreshold()` 里 `relative_high = otsu + 0.55*(max_i - otsu)` 假设高反回波
  接近饱和（`max_i` 接近 255）；实测 `front_jt128` 一份场景 `max_i≈91`（无饱和），`relative_high`
  算出 59.748，比环带实际强度（30~60）还高，导致 `adjustHighIntensityThresholdForLabel()` 里
  `Annulus` 标签原有的饱和回退分支（触发条件 `max_i > 200.0f`）根本碰不到，最终该场景 annulus 边界点
  被阈值滤空，`clusterMechanicalAnnulusBoundaryCloud` 找不到任何聚类，圆心检出数从 0/4 到 2/4 不等。
  修复：在 `Annulus` 分支内、饱和回退分支之后新增一条低动态范围回退——`max_i < 150.0f &&
  threshold > high_quantile` 时，阈值回退到 `max(otsu_threshold, high_quantile)`（约等于 p92）；
  两个回退分支的触发条件（`max_i > 200` vs `max_i < 150`）互斥，不影响原饱和分支的行为，
  `computeHighIntensityThreshold()` 本体、ring 边界提取的跳变阈值、非 Annulus 标签的回退逻辑均未改动。
  用 `front_jt128`（`max_i≈91`）实跑验证：新增的 `overshoots on low-dynamic-range cloud` WARN 按预期
  出现，阈值从 59.748 回退到 41.000，两次边界提取尝试（插值/高反侧）都拿到 4 个聚类、4 个候选圆心，
  几何一致性校验通过（max error 9.07mm，RMSE 6.97mm），四环半径校验通过（centerline 最大误差
  2.59mm，半宽最大误差 4.34mm）——从 0/4 圆心恢复到 4/4。饱和场景（`max_i > 200`）行为不变没有实测
  数据可复现（现有 `calib_data/` 几份场景手动 ROI 都是为第一个场景配的，覆盖不到其他场景的标定板），
  按代码逻辑确认：新分支条件 `max_i < 150.0f` 与原饱和分支条件 `max_i > 200.0f` 互斥，`max_i > 200`
  时新分支永远不会触发，原饱和分支的判断和行为原样保留。
