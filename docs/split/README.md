# test_navigation 拆分总览

## 0. 本文档集是什么

分析对象：`test/test_navigation/src/test_navigation.cpp`（**7937 行**，单个类 `RealtimePgmNode`，**104 个 ROS 参数**、**96 个成员函数 + 1 个 main**）。

目标：按"多类 + 纯算法库"重构，把代码拆到工作区顶层的 `src/`、`tasks/`、`tools/`、`config/`，由你自行逐块搬运。

| 文档 | 内容 | 条目数 |
|---|---|---|
| `INDEX.md` | 全量索引：函数、源码行、目标位置 | 119 |
| `01_config.md` | 常量、9 个数据结构、参数清单（104 个）、20 组成员变量 | 30 |
| `02_tools.md` | 纯计算 / 坐标变换 / 栅格 / 序列化函数（含完整源码） | 31 |
| `03_tasks.md` | 业务流程函数（含完整源码） | 43 |
| `04_node.md` | 构造装配、回调、参数回调、CAN、main（含完整源码） | 15 |
| `05_other_files.md` | launch / config / scripts / 构建文件 / 日志等非 C++ 文件的归属 | — |
| `gen_function_docs.py` | 文档生成脚本（改元数据即可重新生成） | — |

重新生成：

```bash
cd /home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation
python3 docs/split/gen_function_docs.py
```

> 每个条目都是"源码行 + 目标位置 + 职责 + 依赖"，下面紧跟原样源码，可直接复制到你新建的文件里。

## 1. 目标目录与依赖方向

```
Navigation/
├── config/          # 纯头文件/纯数据：常量、struct、参数加载器、yaml
│   ├── constants.hpp      kMotionSpeedScale / kMotionMaxSpeed / kSlowRadiusTime / kMotionRawLimit
│   ├── types.hpp          Point / TimedCloud / PoseCorrection / LookaheadView / HoldObstacleCluster / FootprintBounds / ChaseTrackPoint
│   ├── config.hpp         MappingConfig / MotionConfig / AstarConfig / HoldPointConfig / ChaseConfig / IoConfig
│   └── config_loader.*    declare_parameter -> AppConfig
├── tools/           # 零 ROS 依赖的纯函数库（只 include config/ + 标准库 + msg 类型）
│   ├── tf_math.*          quaternionToMatrix / matrixToQuaternion / yawFromMatrix / wrapAngle / multiplyMatrix / transformPointByCorrection / transformToBaseLink
│   ├── motion_codec.*     limitMotionSpeed / effectiveSlowRadius / encodeMotionRaw / resolveAstarSpeed / packMotionFrame
│   ├── grid.*             accumulateCloud / applyDilate / occupancyToPixels / clearVehicleFootprint
│   ├── grid_geom.*        computeMapSize / computeVehicleFootprint / clipCloudToMapBounds / filterVehicleFootprintPoints / computeHoldBlockPose
│   ├── grid_cluster.*     extractHoldObstacles / findNearestObstacleCluster
│   ├── grid_clearance.*   isHoldPointClear / isHoldPointClearPath / chaseLineClear
│   ├── astar.*            inflateGrid / findNearestFreeCell / buildObstacleCostField / astarSearch / backtrackCells / cellsToWorld / resampleByArcLength
│   ├── pure_pursuit.*     baseFromLidar / nearestOnPlan / trapezoidSpeed / lookaheadByArc / projectToBody
│   ├── registration.*     scorePose（estimatePoseCorrection 的评分 lambda）
│   ├── io_pcd.* io_bin.* io_yaml.* io_pgm.*
│   └── ros_utils.*        hasField 等消息层小工具
├── tasks/           # 业务/行为层：持有 ROS 通信与状态，调用 tools
│   ├── cloud_pipeline.*   cloudCallback / pruneHistory
│   ├── dynamic_split.*    splitDynamicObstacles / subtractBaseline / buildStaticBaseline
│   ├── pose_correction.*  estimatePoseCorrection 外壳 + get/set/reset/publish
│   ├── cloud_publish.*    preview / self_filtered / dynamic / clear_ring
│   ├── map_render.*       renderPgm / renderHistoryPgm / renderDynamicObstaclePgm
│   ├── map_io.*           export / import / mergeImportedMap
│   ├── astar_planner.*    buildMergedOccupancy / planAstarPath / getAstarPath / publishAstarPlan
│   ├── hold_point.*       computeHoldPointCommand + viz/status
│   ├── chase.*            computeChaseCommand + recordTargetCloud + viz/status/track
│   ├── motion_control.*   startMotion / stopMotion / computeFollowCommand / computeAstarFollowCommand
│   ├── motion_frame.*     publishMotionFrame
│   ├── nav2_service.*     clearCostmaps
│   └── debug_log.*        initDebugLog / logCloudFrame / saveCloudPcd
└── src/             # 节点装配层：只做 ROS 胶水
    ├── main.cpp
    ├── realtime_pgm_node.hpp / .cpp   构造装配 + printConfig + createMotionTimer
    ├── callbacks.cpp                  odom/plan/localPlan/goalPose/renderTimer/motionTimer
    ├── parameter_callback.cpp         732 行参数热更新
    └── can_socket.cpp                 open/close/canFrameFromMotion
```

依赖方向（单向，不允许反向）：

```
config  <-  tools  <-  tasks  <-  src
```

`tools/` 只依赖 `config/` + 标准库 + 必要的 msg 头；`tasks/` 依赖 `tools/` 并可持有 publisher/mutex/参数快照；`src/` 依赖 `tasks/`，是唯一做 `create_subscription/create_publisher/create_wall_timer` 的地方。

## 2. 建议的迁移顺序（每阶段都能编译）

1. **建 config**：搬 `constants.hpp` / `types.hpp`（`01_config.md` 第 1 节）。此时把 `Point` 等改为全局 `namespace nav::config`，原文件保留 `using` 别名即可编译。
2. **抽 tools 的纯函数**：先搬"零成员访问"的 12 个——`accumulateCloud`、`applyDilate`、`occupancyToPixels`、`limitMotionSpeed`、`effectiveSlowRadius`、`encodeMotionRaw`、`wrapAngle`、`yawFromMatrix`、`multiplyMatrix`、`quaternionToMatrix`、`matrixToQuaternion`、`transformPointByCorrection`。这类改动零风险（纯替换调用点）。
3. **抽工具+改造签名**：`computeMapSize`、`clipCloudToMapBounds`、`computeVehicleFootprint`、`filterVehicleFootprintPoints`、`clearVehicleFootprint`、`transformToBaseLink`、`isHoldPointClear(Path)`、`computeHoldBlockPose`、`chaseLineClear`、`extractHoldObstacles`、`findNearestObstacleCluster`。把"读成员"改成"传入 `Config` + 栅格 + mask"。
4. **抽序列化/IO**：`saveCloudPcd`、`writeHistoryCloudsBin`、`readHistoryCloudsBin`、`writeMapYaml`（以及 `renderPgm` 内的 PGM 写盘段）→ `tools/io_*`。
5. **抽 tasks 类**：按 `03_tasks.md` 的 9 个小节逐类搬，节点类只留 `shared_ptr<Task>` 成员与回调转调。
6. **拆大回调**：`motionTimerCallback`（1483 行）与 `parameterCallback`（732 行）最后拆，因为它们牵动的状态最多。
7. **参数外置**：`118-604` 行的 104 个 `declare_parameter` → `config/config_loader` + `config/params.yaml`。

## 3. 二次抽取：大函数内部还能拆出的纯内核

| 内核 | 出处（源码行） | 建议 tools 签名 |
|---|---|---|
| 障碍膨胀为 blocked 网格 | 4328-4351 | `inflateGrid(occ, w, h, cells) -> vector<uint8_t>` |
| 起点强制可通行 / 终点就近可行格 | 4355-4409 | `forceFree(blocked)` / `findNearestFreeCell(blocked, w, h, gx, gy)` |
| 障碍邻近代价场（多源 BFS + 线性衰减） | 4414-4475 | `buildObstacleCostField(occ, w, h, reach, penalty)` |
| A\* 搜索本体（八连通 + 防穿墙角 + 惰性删除） | 4477-4562 | `astarSearch(blocked, cost, w, h, start, goal)` |
| 回溯 + 栅格→世界 + 弧长抽稀 | 4564-4601 | `backtrackCells` / `cellsToWorld` / `resampleByArcLength` |
| 配准评分 lambda | 1469-1505 | `scorePose(samples, mask, w, h, x, y, yaw, score_min)` |
| 跟随几何五段 | 5844-6017 | `baseFromLidar` / `nearestOnPlan` / `trapezoidSpeed` / `lookaheadByArc` / `projectToBody` |
| 运动帧打包（8 字节协议） | 3432-3474 内 | `packMotionFrame(control, vx, vy, vx_scale, vy_scale) -> array<uint8_t,8>` |
| 聚类桶 | 4751-4756（局部 struct Bucket） | `clusterOccupancy(occ, w, h, step, merge_r, center, reach)` |
| PGM 写盘 | 6554-6655 / 6659-6827 内的写文件段 | `writePgmFile(path, w, h, pixels)` |

## 4. motionTimerCallback 的天然拆段（1483 行）

| 段 | 源码行 | 迁移目标 |
|---|---|---|
| 消费 start/stop 按钮请求 | 3653-3661 | `src/callbacks.cpp` 保留 |
| 追击优先接管 | 3669-3722 | `tasks/chase.cpp :: ChaseTask::step()` |
| 占点接管 | 3731-3788 | `tasks/hold_point.cpp :: HoldPointTask::step()` |
| 取运动状态与路径（局部优先） | 3790-3844 | `tasks/motion_control.cpp :: MotionController::selectPlan()` |
| 无运动 / 无里程计分支 | 3847-3855 | 同上 |
| 判可用路径 + A\* 兜底 | 3870-3954 | `AstarPlanner::fallbackStep()` |
| 正常跟随 | 3957-3967 | `MotionController::follow()` |
| 到点位置校验 + A\* 重导航 | 3978-4119 | `MotionController::goalCheckStep()` |
| 发帧 + 节流日志 | 4121-4132 | `MotionFramePublisher` |

## 5. 拆分风险点（务必先看）

1. **重复建图**：`computeHoldPointCommand`、`computeChaseCommand`、`motionTimerCallback` 各自调用 `buildMergedOccupancy`，一帧可能建 3 次。改造时建议在 timer 里建一次 occupancy 并作为入参下传。
2. **四把锁**：`cloudCallback` 同时碰 `cloud_mutex_` / `dynamic_mutex_` / `config_mutex_` / `motion_mutex_`。建议改为"进入即拷贝 `Config` + mask 快照"，出锁后再算。
3. **`TimedCloud` 带 `rclcpp::Time`**：要让 `tools/io_bin` 彻底无 ROS 依赖，把时间戳换成 `int64_t` 纳秒。
4. **`computeFollowCommand` 一身两职**：既算速度又写 `lookahead_view_`（加锁）。tools 版只返回 `{vx, vy, 前视点, dist_end}`，可视化写入上移到 tasks。
5. **A\* 缓存状态**：`astar_cache_valid_` / `astar_cached_path_` / `astar_last_plan_*` 只在 `getAstarPath` 与 `startMotion` 写，建议收进 `AstarPlanner` 私有成员，避免散落。
6. **时间戳一致性**：`cmd.txt` 里已记录的问题——`/cloud_clear_ring`、`/cloud_dynamic_obstacles` 若用 `get_clock()->now()` 打戳会被 Nav2 TF 过滤器整帧丢弃。`pointCloudStamp()` 已给出正确做法（odom 戳 − 0.1 s），拆分时不要退回 `now()`。
7. **参数校验集中段**（2887-3126）不要拆散：影子变量 + 校验表 + 一次提交，是热更新不炸的关键。

## 6. 落地提醒

顶层 `Navigation/{src,tasks,tools,config}` 目前**不是 ROS 包**（`colcon` 只认含 `package.xml` 的目录）。搬运后要能编译，需要以下之一：

- 在 `Navigation/` 根放 `package.xml` + `CMakeLists.txt`（一个包，含 `src/tasks/tools/config` 子目录），或
- 在 `Navigation/src/` 下建包（`nav_core`），`tasks/tools/config` 作为该包内的子目录 / 独立 `INTERFACE` 库。

`tools/` 建议做成 `add_library(nav_tools STATIC ...)`，`tasks/` 做成 `add_library(nav_tasks STATIC ...)`，`src/` 的可执行文件只链接这两个库——这样 tools 还能单独写单元测试。
