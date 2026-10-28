# test_navigation 拆分总览（五层结构）

## 0. 本文档集

分析对象：`test/test_navigation/src/test_navigation.cpp`（**7937 行**，单类 `RealtimePgmNode`，**122 个 ROS 参数**、**96 个成员函数 + 1 个 main**）。

目标结构（**config 只放 yaml 调参；io 与 tools 彻底分离**）：

| 文档 | 内容 | 条目数 |
|---|---|---|
| `INDEX.md` | 全量索引：符号 / 源码行 / 目标文件 / 所属文档 | 119 |
| `01_config.md` | **只有 yaml**：122 个参数表（含类型与默认值）+ `params.yaml` 骨架 + 成员变量去向表 | — |
| `02_tools.md` | 纯计算：类型与常量、几何变换、编解码、栅格、聚类/间距（含完整源码） | 35 |
| `03_io.md` | 硬件与外部交互：ROS 发布、时间戳、服务调用、SocketCAN、文件 IO（含完整源码） | 24 |
| `04_tasks.md` | 业务流程：流水线、位姿修正、渲染、导入导出、A*、占点、追击、运动控制（含完整源码） | 28 |
| `05_node.md` | 装配与回调：构造装配区、main、6 个回调、参数回调 + 成员变量源码附录 | 11 + 20 块 |
| `06_other_files.md` | launch / scripts / 构建文件 / 日志 / 顶层目录的归属，含三库 CMake | — |
| `BY_FILE.md` | 文件视角：每个目标文件装哪些符号 + 合并源码 | — |
| `gen_function_docs.py` | 生成脚本（元数据 + 目标路径覆写表） | — |

重新生成：

```bash
cd /home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation
python3 docs/split/gen_function_docs.py
```

## 1. 目标结构与依赖方向

```
Navigation/
├── config/                  # ★ 只有 yaml，专用于调参
│   ├── params.yaml          122 个可调参数（键名/默认值见 01_config.md）
│   └── nav2_params.yaml     Nav2 栈参数
│
├── tools/                   # ★ 纯计算：不 include ROS、不碰 socket/文件、不加锁
│   ├── types.hpp            Point / LookaheadView / Config / PoseCorrection / HoldObstacleCluster / FootprintBounds / ChaseTrackPoint
│   ├── constants.hpp        kMotionSpeedScale / kMotionMaxSpeed / kSlowRadiusTime / kMotionRawLimit
│   ├── tf_math.*            四元数↔矩阵、yaw、wrapAngle、矩阵乘、点变换、map→base_link
│   ├── ros_utils.*          hasField（消息字段查询，只读入参）
│   ├── registration.*       scorePose（配准评分内核）
│   ├── motion_codec.hpp     限速 / 减速半径 / 12 位量化 / A* 速度解析
│   ├── grid.*               accumulateCloud / applyDilate / occupancyToPixels / clearVehicleFootprint
│   ├── grid_geom.*          computeMapSize / 车体 AABB / 裁剪 / 点级自过滤 / 站位点
│   ├── grid_cluster.*       extractHoldObstacles / findNearestObstacleCluster
│   ├── grid_clearance.*     isHoldPointClear(Path) / chaseLineClear
│   ├── astar.*              （二次抽取）膨胀 / 代价场 / 八连通搜索 / 回溯 / 弧长抽稀
│   └── pure_pursuit.*       （二次抽取）baseFromLidar / nearestOnPlan / trapezoidSpeed / lookaheadByArc / projectToBody
│
├── io/                      # ★ 唯一接触外部世界的地方
│   ├── timed_cloud.hpp      TimedCloud（含 rclcpp::Time，故不放 tools）
│   ├── can_bus.*            openCanSocket / closeCanSocket / canFrameFromMotion
│   ├── ros_cloud_pub.*      preview / self_filtered / dynamic / clear_ring / 通用点云发布
│   ├── ros_path_pub.*       A* 路径 / 占点 / 追击的可视化、状态、轨迹
│   ├── ros_motion_pub.*     8 字节运动帧打包与下发
│   ├── ros_pose_pub.*       map->odom 修正量发布
│   ├── ros_image_pub.*      PGM / 历史图 / 动态障碍层 Image 发布
│   ├── ros_stamp.*          pointCloudStamp（odom 戳 - 0.1 s，防 TF 丢弃）
│   ├── ros_nav2_client.*    ClearEntireCostmap 服务客户端
│   ├── file_pcd.* file_bin.* file_yaml.* file_pgm.*   落盘/读盘
│   └── debug_log_file.*     调试日志目录与 CSV 初始化
│
├── tasks/                   # ★ 业务流程：编排 io 与 tools，持有状态机
│   ├── cloud_pipeline.*     cloudCallback / pruneHistory
│   ├── dynamic_split.*      splitDynamicObstacles / subtractBaseline / buildStaticBaseline
│   ├── pose_correction.*    estimate / set / reset / get / correctCloudForPose
│   ├── map_render.*         renderPgm / renderHistoryPgm / renderDynamicObstaclePgm
│   ├── map_io.*             export / import / mergeImportedMap
│   ├── astar_planner.*      buildMergedOccupancy / planAstarPath / planAstarToGoal / getAstarPath
│   ├── hold_point.*         computeHoldPointCommand
│   ├── chase.*              computeChaseCommand / recordTargetCloud
│   ├── motion_control.*     start/stop / computeFollowCommand / computeAstarFollowCommand
│   └── debug_log.*          logCloudFrame（统计与异常判定）
│
└── src/                     # ★ 只做 ROS 装配
    ├── main.cpp
    ├── realtime_pgm_node.hpp / .cpp   构造装配区 + createMotionTimer + printConfig
    ├── callbacks.cpp                  odom / plan / localPlan / goalPose / renderTimer / motionTimer
    └── parameter_callback.cpp         732 行参数热更新（键来自 params.yaml）
```

依赖方向（单向，禁止反向）：

```
config(yaml)  →  src 读取参数
tools  ←  io  ←  tasks  ←  src
```

**已完成的搬运（2026-10-28）**：`test/test_navigation/config/nav2_params.yaml` → `Navigation/config/nav2_params.yaml`；
`navigate_to_pose_w_replanning_5hz.xml` → `Navigation/launch/behavior_trees/`；
`realtime_pgm.launch.py` 的 `nav2_params_file` 默认值改为绝对路径，并新增 `bt_xml_file` 参数。

判定规则（搬代码时照着问）：

| 问题 | 是 → 放哪 |
|---|---|
| 能在没有 ROS、没有网络、没有磁盘的情况下编译并跑单测？ | `tools/` |
| 需要 rclcpp 类型 / 话题 / 服务 / socket / 文件系统？ | `io/` |
| 需要多步决策、状态机、跨帧状态、互斥量？ | `tasks/` |
| 只是 `create_*` 与回调转发？ | `src/` |
| 只是一个数值/字符串默认值？ | `config/params.yaml` |

## 2. 迁移顺序（每阶段可编译）

1. **建 tools 的类型层**：`types.hpp` / `constants.hpp`（`02_tools.md` 第 1 节），原文件保留 `using` 别名即可过编译。
2. **抽零依赖纯函数**（12 个）：`accumulateCloud`、`applyDilate`、`occupancyToPixels`、`clearVehicleFootprint`、`limitMotionSpeed`、`effectiveSlowRadius`、`encodeMotionRaw`、`wrapAngle`、`yawFromMatrix`、`multiplyMatrix`、`quaternionToMatrix`、`matrixToQuaternion`。
3. **抽参数化后即纯的函数**：`computeMapSize`、`computeVehicleFootprint`、`clipCloudToMapBounds`、`filterVehicleFootprintPoints`、`transformToBaseLink`、`isHoldPointClear(Path)`、`computeHoldBlockPose`、`chaseLineClear`、`extractHoldObstacles`、`findNearestObstacleCluster`（把"读成员"改成"传入 Config + 栅格 + mask"）。
4. **抽 io**：先把 5 个 `publish*` 与 `pointCloudStamp` 挪进 `io/ros_*`，再把文件落盘 4 个函数挪进 `io/file_*`，最后把 CAN 三个函数挪进 `io/can_bus`。
5. **抽 tasks**：按 `04_tasks.md` 的 8 个小节逐类搬，节点类只留 `shared_ptr<Task>`。
6. **拆大回调**：`motionTimerCallback`（1483 行）与 `parameterCallback`（732 行）放最后。
7. **参数外置**：把 `118-604` 的 122 个 `declare_parameter` 默认值抄进 `config/params.yaml`，用 `01_config.md` 的骨架直接生成。

## 3. 二次抽取：大函数内部的纯内核（→ tools）

| 内核 | 出处（源码行） | 建议签名 |
|---|---|---|
| 障碍膨胀为 blocked | 4328-4351 | `inflateGrid(occ, w, h, cells)` |
| 起点强制可通行 / 终点就近可行格 | 4355-4409 | `forceFree` / `findNearestFreeCell` |
| 障碍邻近代价场（多源 BFS） | 4414-4475 | `buildObstacleCostField(occ, w, h, reach, penalty)` |
| A\* 八连通搜索（防穿墙角、惰性删除） | 4477-4562 | `astarSearch(blocked, cost, w, h, s, g)` |
| 回溯 / 栅格→世界 / 弧长抽稀 | 4564-4601 | `backtrackCells` / `cellsToWorld` / `resampleByArcLength` |
| 配准评分 lambda | 1469-1505 | `scorePose(samples, mask, w, h, x, y, yaw, score_min)` |
| 跟随几何五段 | 5844-6017 | `baseFromLidar` / `nearestOnPlan` / `trapezoidSpeed` / `lookaheadByArc` / `projectToBody` |
| 运动帧打包（8 字节协议） | 3432-3474 内 | `packMotionFrame(control, vx, vy, sx, sy)` → 放 `tools/motion_codec`，`send` 留 `io/can_bus` |
| PGM 写盘 | `renderPgm` / `renderHistoryPgm` 内 | `writePgmFile(path, w, h, pixels)` → `io/file_pgm` |
| 聚类桶 | 4751-4756 | `clusterOccupancy(occ, w, h, step, merge_r, center, reach)` |

## 4. motionTimerCallback 拆段（1483 行）

| 段 | 源码行 | 迁移目标 |
|---|---|---|
| 消费 start/stop 按钮 | 3653-3661 | `src/callbacks.cpp` |
| 追击优先接管 | 3669-3722 | `tasks/chase.cpp :: ChaseTask::step()` |
| 占点接管 | 3731-3788 | `tasks/hold_point.cpp :: HoldPointTask::step()` |
| 取状态与路径（局部优先） | 3790-3844 | `tasks/motion_control.cpp :: selectPlan()` |
| 无运动 / 无里程计 | 3847-3855 | 同上 |
| 判可用路径 + A\* 兜底 | 3870-3954 | `tasks/astar_planner.cpp :: fallbackStep()` |
| 正常跟随 | 3957-3967 | `tasks/motion_control.cpp :: follow()` |
| 到点校验 + A\* 重导航 | 3978-4119 | `tasks/motion_control.cpp :: goalCheckStep()` |
| 发帧 + 节流日志 | 4121-4132 | `io/ros_motion_pub.cpp` |

## 5. 拆分风险点

1. **发布器移层后别让 io 反向依赖 tasks**：`publish*` 只接受"点列 / 路径 / 数值资产"，不允许接收 `Target`、`ChaseState` 这类 tasks 结构。状态序列化（JSON 拼串）留在 `io`，语义由 tasks 决定。
2. **重复建图**：占点、追击、timer 各自调 `buildMergedOccupancy`，一帧可能建 3 次。改造时在 timer 建一次、作为入参下传。
3. **四把锁**：`cloudCallback` 同时碰 cloud/dynamic/config/motion 四把锁。建议"进锁取 `Config` + mask 快照，出锁再算"。
4. **`TimedCloud` 带 `rclcpp::Time`**：所以它进 `io/`；若想让 `tools/io_bin` 也能用，时间戳改 `int64_t` 纳秒再下沉。
5. **`computeFollowCommand` 一身两职**：算速度 + 写 `lookahead_view_`。tools 版只返回 `{vx, vy, 前视点, dist_end}`，可视化写入留在 tasks。
6. **A\* 缓存状态**：`astar_cache_valid_` / `astar_cached_path_` / `astar_last_plan_*` 只在 `getAstarPath`、`startMotion` 写，收进 `AstarPlanner` 私有成员。
7. **时间戳一致性**：`/cloud_clear_ring`、`/cloud_dynamic_obstacles` 若用 `get_clock()->now()` 打戳，会被 Nav2 TF 过滤器整帧丢弃（`cmd.txt` 已记录）。统一走 `io/ros_stamp.cpp` 的 `pointCloudStamp()`。
8. **参数校验集中段**（2887-3126）不要拆散：影子变量 + 校验表 + 一次提交，是热更新不炸的关键。

## 6. 落地提醒

顶层 `Navigation/{config,tools,io,tasks,src}` 目前**不是 ROS 包**（`colcon` 只认含 `package.xml` 的目录）。要能编译，需在 `Navigation/` 根放 `package.xml` + `CMakeLists.txt`，或建一个 `nav_core` 包内含这五个子目录。推荐三库一可执行：

- `nav_tools`（STATIC，无 ROS）→ `nav_io`（STATIC，需要 rclcpp/消息）→ `nav_tasks`（STATIC）→ `test_navigation_node`（可执行）。

具体 CMake 写法见 `06_other_files.md` 第 3 节。
