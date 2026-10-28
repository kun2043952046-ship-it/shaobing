# 函数索引（按层 -> 目标文件）

源文件: `test/test_navigation/src/test_navigation.cpp` (7937 行)。「源码行」为起止行，「文档」为含完整源码的详表。

层级：`config/`（只有 yaml） → `tools/`（纯计算） → `io/`（硬件与 ROS/文件） → `tasks/`（业务流程） → `src/`（装配与回调）。

| # | 符号 / 块 | 源码行 | 目标位置 | 文档 |
|---|---|---|---|---|
| 1 | `consts` | 46-55 | `tools/constants.hpp` | 02_tools.md |
| 2 | `Point` | 57-62 | `tools/types.hpp` | 02_tools.md |
| 3 | `LookaheadView` | 70-82 | `tools/types.hpp` | 02_tools.md |
| 4 | `Config` | 84-106 | `tools/types.hpp` | 02_tools.md |
| 5 | `TimedCloud` | 109-113 | `io/timed_cloud.hpp` | 03_io.md |
| 6 | `参数声明区` | 118-604 | `config/params.yaml` | 01_config.md |
| 7 | `constructor_assembly` | 605-815 | `src/realtime_pgm_node.cpp` | 05_node.md |
| 8 | `initDebugLog` | 824-864 | `io/debug_log_file.cpp` | 03_io.md |
| 9 | `saveCloudPcd` | 867-909 | `io/file_pcd.cpp` | 03_io.md |
| 10 | `logCloudFrame` | 917-1042 | `tasks/debug_log.cpp` | 04_tasks.md |
| 11 | `odomCallback` | 1045-1147 | `src/callbacks.cpp` | 05_node.md |
| 12 | `planCallback` | 1150-1174 | `src/callbacks.cpp` | 05_node.md |
| 13 | `localPlanCallback` | 1179-1201 | `src/callbacks.cpp` | 05_node.md |
| 14 | `goalPoseCallback` | 1205-1215 | `src/callbacks.cpp` | 05_node.md |
| 15 | `hasField` | 1217-1227 | `io/ros_pc_utils.hpp` | 03_io.md |
| 16 | `PoseCorrection` | 1233-1238 | `tools/types.hpp` | 02_tools.md |
| 17 | `quaternionToMatrix` | 1240-1265 | `tools/tf_math.hpp (inline)` | 02_tools.md |
| 18 | `matrixToQuaternion` | 1267-1298 | `tools/tf_math.hpp (inline)` | 02_tools.md |
| 19 | `yawFromMatrix` | 1301-1304 | `tools/tf_math.hpp (inline)` | 02_tools.md |
| 20 | `wrapAngle` | 1307-1318 | `tools/tf_math.hpp (inline)` | 02_tools.md |
| 21 | `multiplyMatrix` | 1321-1333 | `tools/tf_math.hpp (inline)` | 02_tools.md |
| 22 | `getPoseCorrection` | 1335-1339 | `tasks/pose_correction.cpp` | 04_tasks.md |
| 23 | `publishPoseCorrection` | 1342-1359 | `io/ros_pose_pub.cpp` | 03_io.md |
| 24 | `setPoseCorrection2D` | 1362-1378 | `tasks/pose_correction.cpp` | 04_tasks.md |
| 25 | `resetPoseCorrection` | 1380-1396 | `tasks/pose_correction.cpp` | 04_tasks.md |
| 26 | `transformPointByCorrection` | 1398-1409 | `tools/tf_math.hpp (inline)` | 02_tools.md |
| 27 | `estimatePoseCorrection` | 1411-1688 | `tasks/pose_correction.cpp (+ tools/registration.hpp 评分内核)` | 04_tasks.md |
| 28 | `score_at` | 1469-1505 | `tools/registration.hpp` | 02_tools.md |
| 29 | `correctCloudForPose` | 1690-1713 | `tasks/pose_correction.cpp` | 04_tasks.md |
| 30 | `cloudCallback` | 1715-1949 | `tasks/cloud_pipeline.cpp` | 04_tasks.md |
| 31 | `pruneHistory` | 1952-1973 | `tasks/cloud_pipeline.cpp` | 04_tasks.md |
| 32 | `splitDynamicObstacles` | 1985-2077 | `tasks/dynamic_split.cpp` | 04_tasks.md |
| 33 | `subtractBaseline` | 2087-2156 | `tasks/dynamic_split.cpp` | 04_tasks.md |
| 34 | `buildStaticBaseline` | 2161-2320 | `tasks/dynamic_split.cpp` | 04_tasks.md |
| 35 | `publishCloudPreview` | 2323-2369 | `io/ros_cloud_pub.cpp` | 03_io.md |
| 36 | `pointCloudStamp` | 2384-2409 | `io/ros_stamp.cpp` | 03_io.md |
| 37 | `publishPointCloud` | 2412-2445 | `io/ros_cloud_pub.cpp` | 03_io.md |
| 38 | `transformToBaseLink` | 2454-2494 | `tools/tf_math.cpp` | 02_tools.md |
| 39 | `publishSelfFilteredCloud` | 2510-2578 | `io/ros_cloud_pub.cpp` | 03_io.md |
| 40 | `parameterCallback` | 2580-3311 | `src/parameter_callback.cpp` | 05_node.md |
| 41 | `clearCostmaps` | 3317-3361 | `io/ros_nav2_client.cpp` | 03_io.md |
| 42 | `renderTimerCallback` | 3363-3425 | `src/callbacks.cpp` | 05_node.md |
| 43 | `publishMotionFrame` | 3432-3474 | `io/ros_motion_pub.cpp` | 03_io.md |
| 44 | `closeCanSocket` | 3478-3484 | `io/can_bus.cpp` | 03_io.md |
| 45 | `openCanSocket` | 3487-3536 | `io/can_bus.cpp` | 03_io.md |
| 46 | `canFrameFromMotion` | 3540-3586 | `io/can_bus.cpp` | 03_io.md |
| 47 | `limitMotionSpeed` | 3590-3601 | `tools/motion_codec.hpp` | 02_tools.md |
| 48 | `effectiveSlowRadius` | 3606-3615 | `tools/motion_codec.hpp` | 02_tools.md |
| 49 | `encodeMotionRaw` | 3619-3628 | `tools/motion_codec.hpp` | 02_tools.md |
| 50 | `createMotionTimer` | 3631-3649 | `src/realtime_pgm_node.cpp` | 05_node.md |
| 51 | `motionTimerCallback` | 3651-4133 | `src/callbacks.cpp (流程段建议拆到 tasks/motion_pipeline)` | 05_node.md |
| 52 | `buildMergedOccupancy` | 4154-4214 | `tasks/astar_planner.cpp` | 04_tasks.md |
| 53 | `planAstarPath` | 4239-4265 | `tasks/astar_planner.cpp` | 04_tasks.md |
| 54 | `planAstarToGoal` | 4273-4602 | `tasks/astar_planner.cpp (+ tools/astar.cpp 内核)` | 04_tasks.md |
| 55 | `getAstarPath` | 4612-4669 | `tasks/astar_planner.cpp` | 04_tasks.md |
| 56 | `publishAstarPlan` | 4672-4694 | `io/ros_path_pub.cpp` | 03_io.md |
| 57 | `HoldObstacleCluster` | 4717-4721 | `tools/types.hpp` | 02_tools.md |
| 58 | `extractHoldObstacles` | 4726-4818 | `tools/grid_cluster.cpp` | 02_tools.md |
| 59 | `computeHoldBlockPose` | 4825-4855 | `tools/grid_geom.cpp` | 02_tools.md |
| 60 | `isHoldPointClear` | 4863-4928 | `tools/grid_clearance.cpp` | 02_tools.md |
| 61 | `isHoldPointClearPath` | 4934-5012 | `tools/grid_clearance.cpp` | 02_tools.md |
| 62 | `computeHoldPointCommand` | 5017-5217 | `tasks/hold_point.cpp` | 04_tasks.md |
| 63 | `publishHoldPointViz` | 5221-5251 | `io/ros_path_pub.cpp` | 03_io.md |
| 64 | `publishHoldPointStatus` | 5254-5296 | `io/ros_path_pub.cpp` | 03_io.md |
| 65 | `findNearestObstacleCluster` | 5318-5347 | `tools/grid_cluster.cpp` | 02_tools.md |
| 66 | `chaseLineClear` | 5352-5367 | `tools/grid_clearance.cpp` | 02_tools.md |
| 67 | `computeChaseCommand` | 5371-5633 | `tasks/chase.cpp` | 04_tasks.md |
| 68 | `recordTargetCloud` | 5638-5671 | `tasks/chase.cpp` | 04_tasks.md |
| 69 | `publishChaseViz` | 5674-5710 | `io/ros_path_pub.cpp` | 03_io.md |
| 70 | `publishChaseStatus` | 5713-5758 | `io/ros_path_pub.cpp` | 03_io.md |
| 71 | `publishChaseTrack` | 5761-5787 | `io/ros_path_pub.cpp` | 03_io.md |
| 72 | `computeAstarFollowCommand` | 5792-5804 | `tasks/motion_control.cpp` | 04_tasks.md |
| 73 | `resolveAstarSpeed` | 5808-5812 | `tools/motion_codec.hpp` | 02_tools.md |
| 74 | `computeFollowCommand` | 5823-6035 | `tasks/motion_control.cpp (+ tools/pure_pursuit.cpp 几何内核)` | 04_tasks.md |
| 75 | `startMotion` | 6038-6120 | `tasks/motion_control.cpp` | 04_tasks.md |
| 76 | `stopMotion` | 6122-6150 | `tasks/motion_control.cpp` | 04_tasks.md |
| 77 | `computeMapSize` | 6153-6191 | `tools/grid_geom.cpp` | 02_tools.md |
| 78 | `FootprintBounds` | 6196-6203 | `tools/types.hpp` | 02_tools.md |
| 79 | `computeVehicleFootprint` | 6210-6253 | `tools/grid_geom.cpp` | 02_tools.md |
| 80 | `clipCloudToMapBounds` | 6262-6294 | `tools/grid_geom.cpp` | 02_tools.md |
| 81 | `filterVehicleFootprintPoints` | 6301-6327 | `tools/grid_geom.cpp` | 02_tools.md |
| 82 | `clearVehicleFootprint` | 6337-6390 | `tools/grid.cpp` | 02_tools.md |
| 83 | `accumulateCloud` | 6395-6459 | `tools/grid.cpp` | 02_tools.md |
| 84 | `applyDilate` | 6462-6519 | `tools/grid.cpp` | 02_tools.md |
| 85 | `occupancyToPixels` | 6522-6552 | `tools/grid.cpp` | 02_tools.md |
| 86 | `renderPgm` | 6554-6655 | `tasks/map_render.cpp` | 04_tasks.md |
| 87 | `renderHistoryPgm` | 6659-6827 | `tasks/map_render.cpp` | 04_tasks.md |
| 88 | `writeHistoryCloudsBin` | 6839-6892 | `io/file_bin.cpp` | 03_io.md |
| 89 | `readHistoryCloudsBin` | 6895-6975 | `io/file_bin.cpp` | 03_io.md |
| 90 | `writeMapYaml` | 6985-7021 | `io/file_yaml.cpp` | 03_io.md |
| 91 | `mergeImportedMap` | 7026-7043 | `tasks/map_io.cpp` | 04_tasks.md |
| 92 | `exportHistoryMap` | 7046-7155 | `tasks/map_io.cpp` | 04_tasks.md |
| 93 | `importHistoryMap` | 7158-7261 | `tasks/map_io.cpp` | 04_tasks.md |
| 94 | `renderDynamicObstaclePgm` | 7279-7322 | `tasks/map_render.cpp` | 04_tasks.md |
| 95 | `publishDynamicObstacleCloud` | 7346-7418 | `io/ros_cloud_pub.cpp` | 03_io.md |
| 96 | `publishClearRingCloud` | 7432-7469 | `io/ros_cloud_pub.cpp` | 03_io.md |
| 97 | `printConfig` | 7471-7567 | `src/realtime_pgm_node.cpp` | 05_node.md |
| 98 | `成员变量: 点云基础 / 渲染周期` | 7570-7576 | `tasks/cloud_pipeline.cpp（cloud_mutex_ / latest_cloud_）；话题名进 params.yaml` | 05_node.md |
| 99 | `成员变量: 调试日志 / 异常快照 / 跳变检测` | 7578-7611 | `tasks/debug_log.cpp → DebugLogger（阈值键来自 params.yaml）` | 05_node.md |
| 100 | `成员变量: 参数与配置快照` | 7613-7615 | `值 → config/params.yaml；结构 → tools/types.hpp；锁与快照 → src/ 节点` | 05_node.md |
| 101 | `成员变量: 订阅 / 定时器 / 单帧图发布` | 7617-7630 | `订阅与定时器 → src/ 节点；Image 发布器 → io/ros_image_pub` | 05_node.md |
| 102 | `成员变量: 历史图缓冲` | 7632-7638 | `tasks/cloud_pipeline.cpp（history_ 队列）` | 05_node.md |
| 103 | `成员变量: 地图导出 / 导入` | 7640-7655 | `tasks/map_io.cpp → MapIo（atomic 请求位）；落盘 → io/file_*` | 05_node.md |
| 104 | `成员变量: 动态障碍 / 位姿修正` | 7657-7704 | `tasks/dynamic_split.cpp + tasks/pose_correction.cpp` | 05_node.md |
| 105 | `成员变量: 自过滤点云发布` | 7706-7715 | `io/ros_cloud_pub.cpp → RosCloudPublisher` | 05_node.md |
| 106 | `成员变量: 清除射线` | 7717-7725 | `io/ros_cloud_pub.cpp → RosCloudPublisher` | 05_node.md |
| 107 | `成员变量: 清空 Nav2 代价地图` | 7727-7735 | `io/ros_nav2_client.cpp；开关键 `clear_costmap_on_history_reset` 进 params.yaml` | 05_node.md |
| 108 | `成员变量: 运动帧框架 / 输入话题` | 7737-7749 | `话题名与 scale → config/params.yaml；打包与下发 → io/ros_motion_pub.cpp` | 05_node.md |
| 109 | `成员变量: 路径跟随参数` | 7751-7775 | `config/params.yaml（motion_* 14 项）` | 05_node.md |
| 110 | `成员变量: A* 兜底参数` | 7777-7788 | `config/params.yaml（astar_* 8 项）+ tasks/astar_planner.cpp（缓存）` | 05_node.md |
| 111 | `成员变量: 占点` | 7790-7816 | `config/params.yaml（hold_point_*）+ tasks/hold_point.cpp（状态）` | 05_node.md |
| 112 | `成员变量: 追击` | 7818-7846 | `config/params.yaml（chase_*）+ tasks/chase.cpp（状态与轨迹）` | 05_node.md |
| 113 | `成员变量: 轨迹 + A* 缓存 + 目标点缓存` | 7847-7867 | `tasks/chase.cpp（chase_track_）+ tasks/astar_planner.cpp（astar_cache_*）` | 05_node.md |
| 114 | `ChaseTrackPoint` | 7848-7854 | `tools/types.hpp` | 02_tools.md |
| 115 | `成员变量: 运动状态 / 里程计 / 路径缓冲` | 7869-7903 | `tasks/motion_control.cpp（状态）+ src/callbacks.cpp（写入）` | 05_node.md |
| 116 | `成员变量: 运动请求 + 可视化快照` | 7905-7911 | `tasks/motion_control.cpp（lookahead_view_ 的写入上移到此）` | 05_node.md |
| 117 | `成员变量: SocketCAN 直发` | 7913-7921 | `io/can_bus.cpp → CanBus；`can_*` 键进 params.yaml` | 05_node.md |
| 118 | `成员变量: 参数回调句柄` | 7923-7924 | `src/parameter_callback.cpp` | 05_node.md |
| 119 | `main` | 7927-7937 | `src/main.cpp` | 05_node.md |

合计 **119** 个条目。
