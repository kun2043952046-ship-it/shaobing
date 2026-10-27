# 01 配置与数据类型（config/）

来源: `test/test_navigation/src/test_navigation.cpp`。本文覆盖常量、数据结构、参数声明、成员变量四部分。
迁入目录: 工作区顶层 `Navigation/config/`；若设为纯头文件目录，`tools/` 与 `tasks/` 都可直接 include。

## 参数清单（自动扫描 118-604 行的 declare_parameter）

共 **104** 个参数。

| 主题 | 参数 | 类型 | 声明行 |
|---|---|---|---|
| 基础建图 (写入 Config) | `resolution` | `double` | 121 |
| 基础建图 (写入 Config) | `zmin` | `double` | 123 |
| 基础建图 (写入 Config) | `zmax` | `double` | 125 |
| 基础建图 (写入 Config) | `dilate` | `int` | 127 |
| 基础建图 (写入 Config) | `baseline_dilate` | `int` | 129 |
| 基础建图 (写入 Config) | `self_filter_size` | `double` | 132 |
| 基础建图 (写入 Config) | `xmin` | `double` | 135 |
| 基础建图 (写入 Config) | `ymin` | `double` | 137 |
| 基础建图 (写入 Config) | `xmax` | `double` | 139 |
| 基础建图 (写入 Config) | `ymax` | `double` | 141 |
| 基础建图 (写入 Config) | `history_duration` | `double` | 151 |
| 基础建图 (写入 Config) | `history_max_points_per_frame` | `int` | 153 |
| 基础建图 (写入 Config) | `history_min_hits` | `int` | 155 |
| 调试日志 / 异常快照 | `debug_log_enabled` | `bool` | 169 |
| 调试日志 / 异常快照 | `debug_pcd_enabled` | `bool` | 174 |
| 调试日志 / 异常快照 | `debug_pcd_max_files` | `int` | 177 |
| 调试日志 / 异常快照 | `debug_cloud_count_jump_ratio` | `double` | 180 |
| 调试日志 / 异常快照 | `debug_cloud_count_jump_delta` | `int` | 182 |
| 调试日志 / 异常快照 | `debug_centroid_jump_m` | `double` | 185 |
| 调试日志 / 异常快照 | `debug_odom_jump_m` | `double` | 188 |
| 调试日志 / 异常快照 | `debug_odom_jump_yaw_deg` | `double` | 190 |
| 调试日志 / 异常快照 | `pose_jump_reject_sec` | `double` | 194 |
| 调试日志 / 异常快照 | `debug_occ_ratio_jump` | `double` | 197 |
| 渲染 / 话题名 | `render_period_ms` | `int` | 200 |
| 渲染 / 话题名 | `clear_history` | `bool` | 203 |
| 渲染 / 话题名 | `cloud_preview_max_points` | `int` | 227 |
| 渲染 / 话题名 | `image_topic` | `std::string` | 347 |
| 地图导出 / 导入 | `export_map_dir` | `std::string` | 213 |
| 地图导出 / 导入 | `export_map_name` | `std::string` | 220 |
| 地图导出 / 导入 | `export_map_request` | `bool` | 214 |
| 地图导出 / 导入 | `import_map_dir` | `std::string` | 216 |
| 地图导出 / 导入 | `import_map_request` | `bool` | 217 |
| 动态障碍模式 | `dynamic_obstacle_enabled` | `bool` | 241 |
| 位姿校正 (map->odom) | `pose_correction_topic` | `std::string` | 255 |
| 位姿校正 (map->odom) | `pose_correction_global_period_sec` | `double` | 257 |
| 位姿校正 (map->odom) | `pose_correction_global_range_m` | `double` | 259 |
| 位姿校正 (map->odom) | `pose_correction_local_range_m` | `double` | 261 |
| 位姿校正 (map->odom) | `pose_correction_score_min` | `double` | 263 |
| 位姿校正 (map->odom) | `pose_correction_max_delta_m` | `double` | 265 |
| 位姿校正 (map->odom) | `pose_correction_ema_alpha` | `double` | 269 |
| 位姿校正 (map->odom) | `pose_correction_score_gain` | `double` | 271 |
| 位姿校正 (map->odom) | `pose_correction_global_trigger_fails` | `int` | 274 |
| 位姿校正 (map->odom) | `apply_pose_correction` | `bool` | 282 |
| 自过滤点云 (Nav2 clearing) | `self_filtered_publish_rate` | `double` | 301 |
| 自过滤点云 (Nav2 clearing) | `self_filtered_max_points` | `int` | 304 |
| 清除射线环 | `clear_ring_points` | `int` | 315 |
| 清除射线环 | `clear_ring_radius` | `double` | 318 |
| 清空 Nav2 代价图 | `clear_costmap_on_history_reset` | `bool` | 327 |
| 运动帧 / 输入话题 | `odom_topic` | `std::string` | 374 |
| 运动帧 / 输入话题 | `global_plan_topic` | `std::string` | 376 |
| 运动帧 / 输入话题 | `local_plan_topic` | `std::string` | 380 |
| 运动帧 / 输入话题 | `local_plan_timeout` | `double` | 382 |
| 运动帧 / 输入话题 | `motion_control` | `int` | 385 |
| 运动帧 / 输入话题 | `motion_vx_scale` | `double` | 387 |
| 运动帧 / 输入话题 | `motion_vy_scale` | `double` | 389 |
| 运动帧 / 输入话题 | `motion_rate_hz` | `double` | 391 |
| 路径跟随 | `motion_target_speed` | `double` | 395 |
| 路径跟随 | `motion_lookahead` | `double` | 397 |
| 路径跟随 | `motion_lookahead_speed_gain` | `double` | 402 |
| 路径跟随 | `motion_lookahead_max` | `double` | 405 |
| 路径跟随 | `motion_yaw_kp` | `double` | 407 |
| 路径跟随 | `motion_goal_tolerance` | `double` | 409 |
| 路径跟随 | `motion_goal_check_tolerance` | `double` | 416 |
| 路径跟随 | `motion_goal_check_max_retry` | `int` | 419 |
| 路径跟随 | `motion_max_lateral` | `double` | 421 |
| 路径跟随 | `motion_plan_timeout` | `double` | 427 |
| 路径跟随 | `motion_slow_radius` | `double` | 431 |
| 路径跟随 | `motion_min_speed` | `double` | 433 |
| 路径跟随 | `motion_base_to_lidar_x` | `double` | 435 |
| 路径跟随 | `motion_base_to_lidar_y` | `double` | 437 |
| A* 兜底 | `astar_enabled` | `bool` | 447 |
| A* 兜底 | `astar_speed` | `double` | 451 |
| A* 兜底 | `astar_inflate_radius` | `double` | 456 |
| A* 兜底 | `astar_obstacle_penalty` | `double` | 460 |
| A* 兜底 | `astar_cost_reach` | `double` | 465 |
| A* 兜底 | `astar_path_spacing` | `double` | 469 |
| A* 兜底 | `astar_replan_interval` | `double` | 479 |
| A* 兜底 | `astar_replan_move_dist` | `double` | 482 |
| A* 兜底 | `goal_pose_topic` | `std::string` | 486 |
| A* 兜底 | `astar_plan_topic` | `std::string` | 490 |
| 网页一次性触发 | `start_motion` | `bool` | 493 |
| 网页一次性触发 | `stop_motion` | `bool` | 494 |
| 占点 | `hold_point_enabled` | `bool` | 505 |
| 占点 | `hold_point_x` | `double` | 507 |
| 占点 | `hold_point_y` | `double` | 508 |
| 占点 | `hold_point_radius` | `double` | 511 |
| 占点 | `hold_point_speed` | `double` | 514 |
| 占点 | `hold_point_slow_radius` | `double` | 521 |
| 占点 | `hold_point_standoff_scale` | `double` | 525 |
| 占点 | `hold_point_recenter_deadband` | `double` | 528 |
| 追击 | `chase_enabled` | `bool` | 550 |
| 追击 | `chase_pick_x` | `double` | 552 |
| 追击 | `chase_pick_y` | `double` | 553 |
| 追击 | `chase_radius` | `double` | 555 |
| 追击 | `chase_deadband` | `double` | 557 |
| 追击 | `chase_speed` | `double` | 559 |
| 追击 | `chase_lock_radius` | `double` | 561 |
| 追击 | `chase_track_radius` | `double` | 565 |
| 追击 | `chase_track_history_sec` | `double` | 568 |
| 追击 | `chase_track_record_interval` | `double` | 570 |
| SocketCAN 直发 | `can_enabled` | `bool` | 597 |
| SocketCAN 直发 | `can_interface` | `std::string` | 599 |
| SocketCAN 直发 | `can_id` | `int` | 601 |
| SocketCAN 直发 | `can_send_period_ms` | `int` | 603 |

## 1. 常量与数据结构（config/）

全部为类内 struct/constexpr, 建议搬到 config/ 下的纯头文件, 供 tools 与 tasks 共用。

### `consts`

- **源码行**: `46-55` (10 行)
- **目标位置**: `config/constants.hpp`
- **职责**: 运动帧协议常量: kMotionSpeedScale(量化 200) / kMotionMaxSpeed(±2 m/s) / kSlowRadiusTime(减速时间常数) / kMotionRawLimit(12 位饱和 2040)
- **依赖 / 备注**: block(46-55)

```cpp
  static constexpr double kMotionSpeedScale = 200.0;   // 量化系数: raw = v * scale
  // 调试用单轴速度上限: 协议量程本可到 ±10.2 m/s, 这里收紧到 ±2 m/s。
  // 超限时对 (vx, vy) 做等比缩放 (保方向), 而不是逐轴独立截断。
  static constexpr double kMotionMaxSpeed = 2.0;       // 单轴速度上限 (m/s)
  // 减速起始距离由目标速度决定: slow_radius = kSlowRadiusTime * 目标速度 (m),
  // 即距终点还剩 kSlowRadiusTime 秒的行程时开始线性减速 (速度越大越早减速)。
  // 参数 motion_slow_radius > 0 时用手动值覆盖该自动计算 (见 effectiveSlowRadius)。
  static constexpr double kSlowRadiusTime = 1.0;       // 减速时间常数 (s)
  // 12 位有符号量程对应的原始量上限 (±2040/200 = ±10.2 m/s), 仅用作饱和兜底
  static constexpr long kMotionRawLimit = 2040;
```

### `Point`

- **源码行**: `57-62` (6 行)
- **目标位置**: `config/types.hpp`
- **职责**: 点云基本单元 float x/y/z (被 tools 与 tasks 共用, 建议放最底层头文件)
- **依赖 / 备注**: type; 应保持无 ROS 依赖

```cpp
  struct Point
  {
    float x;
    float y;
    float z;
  };
```

### `LookaheadView`

- **源码行**: `70-82` (13 行)
- **目标位置**: `config/types.hpp`
- **职责**: 路径跟随可视化快照: valid/base_x/base_y/yaw/target_x/target_y/vx/vy/lookahead/dist_end
- **依赖 / 备注**: type

```cpp
  struct LookaheadView
  {
    bool valid{false};
    double base_x{0.0};
    double base_y{0.0};
    double yaw{0.0};
    double target_x{0.0};
    double target_y{0.0};
    double vx{0.0};
    double vy{0.0};
    double lookahead{0.0};
    double dist_end{0.0};
  };
```

### `Config`

- **源码行**: `84-106` (23 行)
- **目标位置**: `config/config.hpp`
- **职责**: 建图配置: resolution/zmin/zmax/dilate/baseline_dilate/self_filter_size/地图范围/输出路径/历史时长/每帧点数上限/投票阈值
- **依赖 / 备注**: type; 纯 POD, 建议按主题拆成 MappingConfig 等

```cpp
  struct Config
  {
    double resolution;
    double zmin;
    double zmax;
    int dilate;
    // 静态基准图膨胀量: 只在动态障碍模式用于生成 static_obstacle_mask_。
    //   <0 表示沿用 dilate; 正值可单独放大基准, 避免"障碍贴住静态物表面"
    //   时被误判为静态 (表现为近距离绿色消失)。
    int baseline_dilate;
    // 车体自身自过滤正方形边长 (m): 以车体中心为中心, 该边长正方形内的
    //   已占据格一律视为车体本身并清除。<=0 关闭。
    double self_filter_size;
    double xmin;
    double ymin;
    double xmax;
    double ymax;
    std::string output;
    std::string history_output;
    double history_duration;
    int history_max_points_per_frame;
    int history_min_hits;
  };
```

### `TimedCloud`

- **源码行**: `109-113` (5 行)
- **目标位置**: `config/types.hpp`
- **职责**: 带时间戳的点云缓存(滑动窗元素): rclcpp::Time stamp + shared_ptr<const vector<Point>>
- **依赖 / 备注**: type; 若要 tools 无 ROS 依赖, stamp 可改 int64_t 纳秒

```cpp
  struct TimedCloud
  {
    rclcpp::Time stamp;
    std::shared_ptr<const std::vector<Point>> points;
  };
```

### `PoseCorrection`

- **源码行**: `1233-1238` (6 行)
- **目标位置**: `config/types.hpp`
- **职责**: map->odom 刚体变换: double r[9] 行主序 + double t[3] + bool valid
- **依赖 / 备注**: type

```cpp
  struct PoseCorrection
  {
    double r[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};   // 行主序
    double t[3] = {0.0, 0.0, 0.0};
    bool valid = false;
  };
```

### `HoldObstacleCluster`

- **源码行**: `4717-4721` (5 行)
- **目标位置**: `config/types.hpp`
- **职责**: 障碍簇世界系质心 x/y (占点与追击共用)
- **依赖 / 备注**: type

```cpp
  struct HoldObstacleCluster
  {
    double x{0.0};
    double y{0.0};
  };
```

### `FootprintBounds`

- **源码行**: `6196-6203` (8 行)
- **目标位置**: `config/types.hpp`
- **职责**: 车体自过滤 AABB: valid / xmin / xmax / ymin / ymax
- **依赖 / 备注**: type

```cpp
  struct FootprintBounds
  {
    bool valid{false};      // false: 无里程计或已关闭自过滤, 不做任何过滤
    double xmin{0.0};
    double xmax{0.0};
    double ymin{0.0};
    double ymax{0.0};
  };
```

### `ChaseTrackPoint`

- **源码行**: `7848-7854` (7 行)
- **目标位置**: `config/types.hpp`
- **职责**: 追击目标轨迹条目: stamp / x / y / 该时刻簇点云
- **依赖 / 备注**: type; 嵌在成员区声明

```cpp
  struct ChaseTrackPoint
  {
    rclcpp::Time stamp;
    double x{0.0};
    double y{0.0};
    std::vector<Point> cloud;   // 记录该时刻目标簇的 3D 点云 (map 系)
  };
```

## 2. 参数声明区（构造函数前半段, 建议替换为 config 加载器）

这 487 行是 86 个 `declare_parameter`。拆分后建议改为：`config/params.yaml` + `config/config_loader.cpp` 一次性读入 `AppConfig` 结构体，构造函数只保留装配。

### `参数声明区 (constructor 118-604)`

- **源码行**: `118-604` (487 行)
- **目标位置**: `config/config_loader.cpp`
- **职责**: 逐项 declare_parameter 并写入成员/Config
- **依赖 / 备注**: block(118-604)

```cpp
    Config initial_config;

    initial_config.resolution =
      declare_parameter<double>("resolution", 0.03);
    initial_config.zmin =
      declare_parameter<double>("zmin", -2.25);
    initial_config.zmax =
      declare_parameter<double>("zmax", 0.22);
    initial_config.dilate =
      declare_parameter<int>("dilate", 1);
    initial_config.baseline_dilate =
      declare_parameter<int>("baseline_dilate", -1);
    // 车体自身自过滤正方形边长 (m): 边长 1.0 m 的正方形内已占据格视为车体
    initial_config.self_filter_size =
      declare_parameter<double>("self_filter_size", 1.0);

    initial_config.xmin =
      declare_parameter<double>("xmin", -20.0);
    initial_config.ymin =
      declare_parameter<double>("ymin", -20.0);
    initial_config.xmax =
      declare_parameter<double>("xmax", 20.0);
    initial_config.ymax =
      declare_parameter<double>("ymax", 20.0);

    initial_config.output =
      declare_parameter<std::string>(
        "output", "/tmp/realtime_map.pgm");

    initial_config.history_output =
      declare_parameter<std::string>(
        "history_output", "/tmp/realtime_map_history.pgm");
    initial_config.history_duration =
      declare_parameter<double>("history_duration", 300.0);
    initial_config.history_max_points_per_frame =
      declare_parameter<int>("history_max_points_per_frame", 5000);
    initial_config.history_min_hits =
      declare_parameter<int>("history_min_hits", 2);

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_ = initial_config;
    }

    cloud_topic_ =
      declare_parameter<std::string>(
        "cloud_topic", "/cloud_registered");

    // ---- 调试日志 / 异常快照 ----
    // 逐帧把点云统计 (点数/质心/包围盒/自过滤数) 写入 CSV, 供"雷达突然全是
    //   障碍"等异常事后溯源; 异常帧 (点数突增/质心跳变) 额外存一份 PCD 快照。
    debug_log_enabled_ = declare_parameter<bool>("debug_log_enabled", true);
    debug_log_dir_ = declare_parameter<std::string>(
      "debug_log_dir",
      "log");
    // 异常帧是否自动落盘 PCD 快照 (供离线复现)
    debug_pcd_enabled_ = declare_parameter<bool>("debug_pcd_enabled", true);
    // PCD 快照落盘上限: 超过后不再保存, 避免磁盘写满
    debug_pcd_max_files_ =
      declare_parameter<int>("debug_pcd_max_files", 200);
    // 点数突增判定: 本帧点数 > 上帧点数 * 该倍数 且 差值 > 阈值 -> 判为异常
    debug_cloud_count_jump_ratio_ =
      declare_parameter<double>("debug_cloud_count_jump_ratio", 3.0);
    debug_cloud_count_jump_delta_ =
      declare_parameter<int>("debug_cloud_count_jump_delta", 20000);
    // 质心跳变判定: 本帧质心与上帧质心距离 > 该值 (m) -> 判为异常
    debug_centroid_jump_m_ =
      declare_parameter<double>("debug_centroid_jump_m", 1.5);
    // 里程计位姿跳变判定: 单帧位移/偏航超阈值 -> WARN (疑似 LIO 发散/重定位)
    debug_odom_jump_m_ =
      declare_parameter<double>("debug_odom_jump_m", 0.5);
    debug_odom_jump_yaw_ =
      declare_parameter<double>("debug_odom_jump_yaw_deg", 30.0) * M_PI / 180.0;
    // 位姿跳变"拒绝窗口"时长 (s): 检出跳变后, 该时长内的所有点云帧一律丢弃
    //   (不入历史缓冲/不发 Nav2), 覆盖跳变后的位姿整定过程。<=0 关闭该保护。
    pose_jump_reject_sec_ =
      declare_parameter<double>("pose_jump_reject_sec", 0.8);
    // 历史图占据比例单帧涨幅超此值 -> WARN
    debug_occ_ratio_jump_ =
      declare_parameter<double>("debug_occ_ratio_jump", 0.15);

    render_period_ms_ =
      declare_parameter<int>("render_period_ms", 150);

    // 一次性触发参数: 网页"刷新累计图"按钮写入 true 时清空历史缓冲并重建
    declare_parameter<bool>("clear_history", false);

    // ---- 历史累计图导出 / 导入 ----
    // 导出: 网页"导出地图"按钮写入 export_map_dir (服务器生成的临时目录) 并置位
    //   export_map_request=true, 本节点把历史图写成 map.pgm + map.yaml,
    //   把原始历史点云队列写成 map_clouds.bin, 由网页端打包成 zip 下载。
    // 导入: 网页上传 zip 解包后写入 import_map_dir 并置位 import_map_request=true,
    //   本节点读取 map_clouds.bin 重建历史点云队列, 读取 map.pgm/map.yaml 作为
    //   导入底图 (用于显示与静态基准)。
    export_map_dir_ =
      declare_parameter<std::string>("export_map_dir", "/tmp/pgm_export");
    declare_parameter<bool>("export_map_request", false);
    import_map_dir_ =
      declare_parameter<std::string>("import_map_dir", "/tmp/pgm_import");
    declare_parameter<bool>("import_map_request", false);
    // 导出 zip 文件名 (网页下载时使用), 缺失时默认 map_export.zip
    export_map_name_ =
      declare_parameter<std::string>("export_map_name", "map_export.zip");

    cloud_preview_topic_ =
      declare_parameter<std::string>(
        "cloud_preview_topic", "/realtime_pgm/cloud");

    cloud_preview_max_points_ =
      declare_parameter<int>("cloud_preview_max_points", 15000);

    history_image_topic_ =
      declare_parameter<std::string>(
        "history_image_topic", "/realtime_pgm/image_history");

    // ---- 动态障碍 (可变障碍) 模式 ----
    // 网页"锁定历史累计图"打开后:
    //   1) 历史累计图停止吸收新点云 (冻结为"静态环境"基准);
    //   2) 之后新出现的点云若落在"锁定时刻历史图未占据"的格子上, 视为
    //      临时出现的障碍 (人/车/纸箱等), 单独渲染成绿色并发布, 不写入历史图;
    //   3) 动态障碍图/点云同时供 Nav2 obstacle_layer 使用 -> 既能避障,
    //      又不会把临时障碍永久烧进静态地图 (避免"假障碍红线"永久化)。
    dynamic_obstacle_enabled_ =
      declare_parameter<bool>("dynamic_obstacle_enabled", false);
    // 动态障碍点云 (世界系, frame=map), 供 Nav2 obstacle_layer 订阅
    dynamic_obstacle_topic_ =
      declare_parameter<std::string>(
        "dynamic_obstacle_topic", "/cloud_dynamic_obstacles");
    // 动态障碍栅格图 (mono8: 0=动态障碍, 205=非动态障碍), 供网页绿色叠加
    dynamic_obstacle_image_topic_ =
      declare_parameter<std::string>(
        "dynamic_obstacle_image_topic", "/realtime_pgm/image_dynamic");

    // ---- 锁定历史图后的位姿校正 (map->odom) ----
    // 锁定后以冻结历史图为基准做二维配准, 输出 map->odom 修正量并发布
    //   /pose_correction, 由 nav2_bridge.py 动态发布 map->odom。
    pose_correction_topic_ =
      declare_parameter<std::string>("pose_correction_topic", "/pose_correction");
    pose_correction_global_period_sec_ =
      declare_parameter<double>("pose_correction_global_period_sec", 1.0);
    pose_correction_global_range_m_ =
      declare_parameter<double>("pose_correction_global_range_m", 15.0);
    pose_correction_local_range_m_ =
      declare_parameter<double>("pose_correction_local_range_m", 1.0);
    pose_correction_score_min_ =
      declare_parameter<double>("pose_correction_score_min", 0.12);
    pose_correction_max_delta_m_ =
      declare_parameter<double>("pose_correction_max_delta_m", 0.3);
    // 时序滤波: EMA 平滑系数 (1 = 不平滑) 与滞回增益阈值
    //   (新解评分须比当前解高出该值才接受, 否则保持当前修正量)
    pose_correction_ema_alpha_ =
      declare_parameter<double>("pose_correction_ema_alpha", 0.5);
    pose_correction_score_gain_ =
      declare_parameter<double>("pose_correction_score_gain", 0.05);
    // 全局兜底搜索: 局部搜索连续失败达到该帧数才触发 ±15 m/±180° 全搜
    pose_correction_global_trigger_fails_ =
      declare_parameter<int>("pose_correction_global_trigger_fails", 5);
    // 修正来源开关:
    //   true  = 本节点自带的 2D 栅格配准: 自己估计 /pose_correction 并发布,
    //           同时施加到整帧点云;
    //   false = 修正交由外部重定位节点 (relocalization_3d GICP) 承担:
    //           本节点不估计、不发布, 仅订阅外部 /pose_correction 并施加到
    //           点云 (动态障碍检测/绿色层/Nav2 点云均基于修正后点云)。
    apply_pose_correction_ =
      declare_parameter<bool>("apply_pose_correction", false);

    // ---- 自过滤点云 (供 Nav2 obstacle_layer 的"清除射线") ----
    // nav2_params.yaml 里 pointcloud 观测源一直订阅 /cloud_self_filtered, 但本节点
    // 此前从未发布它 -> 该源空转, obstacle_layer 少了"用当前可见点云反推空闲空间"
    // 的能力, 导致 dynamic_obstacles 的 raytrace 清除覆盖不全 (旧红色区域残留)。
    // 这里把"车体自过滤后的完整当前帧点云"以 base_link 帧发布到该话题:
    //   - frame=base_link (而非 map): ObservationBuffer 对 PointCloud2 取传感器原点为
    //     点云坐标系原点, 只有 base_link 帧才能让清除射线从"雷达当前位置"射出。
    //     若发 map 帧, 射线起点退化成地图原点, 只能清掉原点->当前位置这条走廊。
    //   - 点坐标必须真实变换到 base_link 系 (见 publishSelfFilteredCloud)。绝不能把
    //     map 系数值谎报成 base_link 帧: Nav2 会再叠一次 base_link->global_frame 的 TF,
    //     相当于机器人位姿施加两次, 整张点云被搬到车体上并随航向旋转。
    self_filtered_topic_ =
      declare_parameter<std::string>(
        "self_filtered_topic", "/cloud_self_filtered");
    // 自过滤点云发布频率 (Hz): 限频避免每帧 2 万点全量发布压垮 DDS/Nav2 变换。
    // <=0 表示不限频 (每收到一帧点云就发一帧)。
    self_filtered_publish_rate_ =
      declare_parameter<double>("self_filtered_publish_rate", 10.0);
    // 每帧最大点数 (等间隔抽稀; <=0 表示不抽稀)
    self_filtered_max_points_ =
      declare_parameter<int>("self_filtered_max_points", 6000);

    // ---- 清除射线点云 (让红色代价区域实时跟随绿色障碍消失) ----
    // nav2_params.yaml 里的 clear_ring 源只开 clearing / 不开 marking, 本节点
    // 每帧向它发一圈 base_link 系的稀疏点, 让 obstacle_layer 从车体原点向四周
    // 射出射线, 把绿色障碍消失后残留的旧红格清成空闲 (详见 publishClearRingCloud)。
    clear_ring_topic_ =
      declare_parameter<std::string>(
        "clear_ring_topic", "/cloud_clear_ring");
    // 环上点数 (每 360/N 度一个): <=0 关闭该机制
    clear_ring_points_ =
      declare_parameter<int>("clear_ring_points", 72);
    // 环半径 (m): 决定清除射线覆盖到车周多远; <=0 关闭
    clear_ring_radius_ =
      declare_parameter<double>("clear_ring_radius", 1.2);

    // ---- 刷新历史累计图时整张清空 Nav2 红色代价区 ----
    // 网页"刷新累计图"只清空本节点的历史点云缓冲, 而红色代价区是 Nav2
    // obstacle_layer 自己维护的累积栅格 —— 两者不联动, 清完历史图后红区仍会
    // 残留旧障碍。这里在刷新历史图的同时, 调用 Nav2 的 ClearEntireCostmap 服务
    // 把局部/全局代价地图整张清空, 之后随新历史图从零重建 (与"刷新=重新建图"
    // 语义一致)。为 true 时生效。
    clear_costmap_on_history_reset_ =
      declare_parameter<bool>("clear_costmap_on_history_reset", true);
    // Nav2 代价地图清空服务名 (与nav2_params.yaml/行为树里的服务名一致)
    clear_local_costmap_service_ =
      declare_parameter<std::string>(
        "clear_local_costmap_service",
        "/local_costmap/clear_entirely_local_costmap");
    clear_global_costmap_service_ =
      declare_parameter<std::string>(
        "clear_global_costmap_service",
        "/global_costmap/clear_entirely_global_costmap");

    parameter_callback_handle_ =
      add_on_set_parameters_callback(
      std::bind(
        &RealtimePgmNode::parameterCallback,
        this,
        std::placeholders::_1));
    
    // 渲染结果同步发布为 mono8 图像, 供查看窗口显示
    image_topic_ =
      declare_parameter<std::string>("image_topic", "/realtime_pgm/image");

    image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(image_topic_, 1);

    // ---------------- 导航运动控制 (网页"开始运动"后按 Nav2 全局路径下发) ----------------
    // 下发话题: 每包 8 字节 uint8 数组, 与接收方 CAN_ID=0xFF 报文 Data[0..7] 一一对应
    //   [0] bit0 = control 运动使能 (0=停止 / 1=运动); bit1 = shoot 发射 (导航恒 0)
    //   [1] tar_yaw   高 8 位 (云台 yaw, 导航恒为 0)
    //   [2] tar_yaw   低 8 位
    //   [3] tar_pitch 高 8 位 (云台 pitch, 导航恒为 0)
    //   [4] tar_pitch 低 8 位
    //   [5] vx 高 8 位 | [6] 高 4 位 = vx 低 4 位
    //   [6] 低 4 位 = vy 高 4 位 | [7] = vy 低 8 位
    //   速度均为 12 位有符号量, 单位 1/200 m/s (接收方 /200.0f 还原)。
    //   vx: 车头前方为正; vy: 车辆左手边为正 (与网页绘制的橙色车体系一致)。
  //   速度矢量 (vx, vy) 的车体系定义为: x 轴 = 车辆前方, 由前方轴逆时针
  //   旋转 90 度得到 y 轴 (车辆左手边)。网页可视化坐标系时按同一约定绘制。
    //   !!! 注意: 雷达"头朝地面"倒装 (绕自身 X 轴转 180 度) 会把雷达的 Y 轴
    //   翻转到车辆右手边, 因此从雷达里程计投影出的横向分量是"右手边为正",
    //   下发前必须取反 (见 computeFollowCommand 步骤 5 的 left = -veh_right)。
    motion_topic_ =
      declare_parameter<std::string>(
        "motion_topic", "/nav_motion_cmd");

    // 运动控制输入源: 雷达位姿里程计 + Nav2 路径
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/Odometry");
    global_plan_topic_ =
      declare_parameter<std::string>("global_plan_topic", "/plan");
    // Nav2 局部路径 (controller_server 输出, 已含动态避障)。
    // 新鲜时优先跟随它, 超时则回退到全局路径 (见 motionTimerCallback)。
    local_plan_topic_ =
      declare_parameter<std::string>("local_plan_topic", "/local_plan");
    local_plan_timeout_ =
      declare_parameter<double>("local_plan_timeout", 0.5);

    motion_control_ =
      declare_parameter<int>("motion_control", 1);
    motion_vx_scale_ =
      declare_parameter<double>("motion_vx_scale", kMotionSpeedScale);
    motion_vy_scale_ =
      declare_parameter<double>("motion_vy_scale", kMotionSpeedScale);
    motion_rate_hz_ =
      declare_parameter<double>("motion_rate_hz", 100.0);

    // 路径跟随参数
    motion_target_speed_ =
      declare_parameter<double>("motion_target_speed", 1.00);
    motion_lookahead_ =
      declare_parameter<double>("motion_lookahead", 0.45);
    // 前视距离速度增益: L = L_min + k * V, 使 L/V 近似恒定 -> 横向误差一阶
    // 收敛时间常数 tau = L/V 恒定, 避免高速 tau 过小导致过冲 (S 型振荡)、
    // 低速 tau 过大导致贴线差。0 表示关闭自适应 (退回固定前视距离)。
    motion_lookahead_speed_gain_ =
      declare_parameter<double>("motion_lookahead_speed_gain", 0.52);
    // 前视距离上限 (m): <= 0 表示不限制。防止高速时前视点过远而过弯切内弯。
    motion_lookahead_max_ =
      declare_parameter<double>("motion_lookahead_max", 1.50);
    motion_yaw_kp_ =
      declare_parameter<double>("motion_yaw_kp", 1.20);
    motion_goal_tolerance_ =
      declare_parameter<double>("motion_goal_tolerance", 0.25);
    // 到达后的位置校验阈值 (m): 判定"到达"时, 除路径末端距离外, 还要复核
    //   "车体中心 -> 用户目标点 (goal_x_/goal_y_, 来自 /nav_goal_pose)" 的距离。
    //   超过该阈值说明只是到了 Nav2 路径末端 (Nav2 tolerance 会在目标 0.5 m 内
    //   就近找可行格, 末点可能偏离用户点的目标), 视为"校验不通过", 需用自建 A*
    //   从当前位置重新导航到目标。<= 0 表示关闭该校验 (仅看路径末端距离)。
    motion_goal_check_tolerance_ =
      declare_parameter<double>("motion_goal_check_tolerance", 0.35);
    // 到达后位置校验失败时, 用自建 A* 重新导航的最大重试次数 (0 = 不重试)。
    motion_goal_check_max_retry_ =
      declare_parameter<int>("motion_goal_check_max_retry", 2);
    motion_max_lateral_ =
      declare_parameter<double>("motion_max_lateral", 0.0);
    // 全局路径新鲜度上限 (s): /plan 超过该时间没有更新, 就判定 Nav2 已停止
    // 重规划 (导航完成/中止/被取消), 立即停车, 不再按过期路径盲跑。
    // Nav2 正常导航时行为树会以 5 Hz 重规划, 因此 1.0 s 足够宽松;
    // <= 0 表示关闭该监督 (仅调试用)。
    motion_plan_timeout_ =
      declare_parameter<double>("motion_plan_timeout", 1.0);
    // 减速起始距离 (m): <= 0 表示自动 —— 由目标速度决定 (= kSlowRadiusTime * 速度);
    // > 0 时为手动覆盖值
    motion_slow_radius_ =
      declare_parameter<double>("motion_slow_radius", 0.0);
    motion_min_speed_ =
      declare_parameter<double>("motion_min_speed", 0.08);
    motion_base_to_lidar_x_ =
      declare_parameter<double>("motion_base_to_lidar_x", -0.239);
    motion_base_to_lidar_y_ =
      declare_parameter<double>("motion_base_to_lidar_y", 0.158);

    // ---- 自建 A* 兜底规划器 ----
    // 动机: Nav2 的 /plan 在"车体起点落在障碍膨胀区内"时会规划失败并停止刷新,
    //   此时用节点内自建的 A* 重算一条 "从当前车体位置 -> Web 页面选定目标点"
    //   的路径 (含障碍邻近代价惩罚, 使路径远离障碍), 以 astar_speed 低速跟随它
    //   逃出障碍并继续前往目标。A* 失败时停车并保持重试 (每帧仍会重新规划)。
    //   Nav2 的 /plan 一旦恢复刷新, 立即切回跟随 Nav2 (Nav2 仍负责局部动态避障)。
    //   注意: 仅在 /plan 不可用 (过期/缺失) 时启用; 每帧在运动线程内重算。
    astar_enabled_ =
      declare_parameter<bool>("astar_enabled", true);
    // A* 兜底路径的跟随速度 (m/s): <= 0 表示跟随 motion_target_speed (与 Nav2
    //   正常巡航速度保持一致, 避免"兜底时突然变慢"的顿挫); > 0 时为手动指定。
    astar_speed_ =
      declare_parameter<double>("astar_speed", -1.0);
    // A* 路径硬性安全间距 (m): 障碍向外膨胀该半径后再规划, 这 2 段内被标为
    //   blocked, A* 绝不进入 -> "车体旋转轴中心 -> 障碍" 的硬性最小间距。
    //   与 Nav2 的 robot_radius 口径一致 (均为"车体中心 -> 障碍"), 统一取 0.50。
    astar_inflate_radius_ =
      declare_parameter<double>("astar_inflate_radius", 0.50);
    // 障碍邻近代价惩罚权重: 栅格越靠近障碍, 代价越高, 引导 A* 走"远离障碍"的
    //   宽阔通路而非贴边。0 表示纯最短路 (只避障不惩罚)。对应"A* + 靠障碍代价惩罚"。
    astar_obstacle_penalty_ =
      declare_parameter<double>("astar_obstacle_penalty", 0.8);
    // 障碍邻近代价影响范围 (m): 距障碍该范围内的栅格按线性衰减加代价 (越近越高),
    //   即"软性避让范围"。与 Nav2 的 inflation_radius 口径一致, 统一取 0.60。
    //   <= 0 表示不产生邻近代价 (等价于纯最短路)。
    astar_cost_reach_ =
      declare_parameter<double>("astar_cost_reach", 0.60);
    // A* 路径点抽稀间距 (m): 规划出的栅格路径按弧长抽稀成较稀疏的路径点,
    //   便于 computeFollowCommand 的前视点推进; <= 0 表示不抽稀 (逐格保留)。
    astar_path_spacing_ =
      declare_parameter<double>("astar_path_spacing", 0.15);
    // A* 重算最小间隔 (s): 运动线程以 motion_rate_hz (默认 100 Hz) 运行, 但整张
    //   栅格重建 + 膨胀 + 代价场 + A* 搜索在 40m×40m/0.05m (64 万格) 下成本很高,
    //   100 Hz 每帧重算会阻塞运动线程 -> 运动帧停发 (车会保持最后指令) -> 表现为
    //   失控。故这里做"限频 + 变化触发": 距上次规划不足该间隔时复用上一条路径,
    //   但满足下列任一条件强制立刻重算:
    //     (a) 车体中心较上次规划点移动超过 astar_replan_move_dist (m);
    //     (b) 距上次规划已超过该间隔 (时间兜底, 保证障碍更新能被吸收)。
    //   本参数即设为 (b) 的时间上限, <= 0 表示不限频 (真·每帧重算, 有卡顿风险)。
    astar_replan_interval_ =
      declare_parameter<double>("astar_replan_interval", 0.20);
    // 触发 A* 重算的车体位移阈值 (m): 车移动超过该距离就立即重算 (与时间兜底二选一)。
    astar_replan_move_dist_ =
      declare_parameter<double>("astar_replan_move_dist", 0.05);
    // Web 目标点话题 (geometry_msgs/PoseStamped, map 系): 由网页点击时发布,
    //   供 A* 兜底规划器作为终点。为空表示不订阅。
    goal_pose_topic_ =
      declare_parameter<std::string>("goal_pose_topic", "/nav_goal_pose");
    // 自建 A* 路径发布话题 (nav_msgs/Path, map 系): 供网页用黄色叠加显示,
    //   便于确认兜底路径算得对不对。为空表示不发布。
    astar_plan_topic_ =
      declare_parameter<std::string>("astar_plan_topic", "/astar_plan");

    // 一次性触发参数: 网页"开始运动"/"停止运动"按钮写入
    declare_parameter<bool>("start_motion", false);
    declare_parameter<bool>("stop_motion", false);

    // ---------------- 占点 (hold point) 行为 ----------------
    // 网页: 先点"选择占点位置"按钮进入选点态, 再点历史累计图上的目标位置,
    //   该点即"占点中心"。车辆自动前往并驻守该中心, 实时计算"障碍 -> 占点中心"
    //   的行进路线, 抢先用车体挡住该路线 (物理阻挡), 全程保持碰撞安全间距。
    //
    // 安全口径: 与当前导航完全一致 —— 车体旋转轴中心 -> 障碍的硬性最小间距
    //   复用 astar_inflate_radius_ (默认 0.50 m, 与 Nav2 robot_radius 同口径),
    //   不再另设一套参数, 保证"占点不撞"与"导航不撞"是同一个判据。
    hold_point_enabled_ =
      declare_parameter<bool>("hold_point_enabled", false);
    // 占点中心 (map 系, m): 网页点击后写入
    hold_point_x_ = declare_parameter<double>("hold_point_x", 0.0);
    hold_point_y_ = declare_parameter<double>("hold_point_y", 0.0);
    // 占点范围 (m): 半径内的障碍才算威胁, 范围外忽略。网页可动态调整。
    //   同时用于历史累计图上绘制占点圈, 便于确认作用范围。
    hold_point_radius_ = declare_parameter<double>("hold_point_radius", 1.50);
    // 占点模式下的机动速度 (m/s): 拦截/回位均以该速度行进。
    //   <= 0 表示跟随 motion_target_speed (与正常巡航速度一致); > 0 时为手动指定。
    hold_point_speed_ = declare_parameter<double>("hold_point_speed", -1.0);
    // 占点机动"接近目标点"的梯形减速起始距离 (m):
    //   < 0 -> 自动: 取常规导航减速距离的一半 (更短的减速段, 响应更快, 避免因
    //          控制/地图更新频率低而"刹不住"冲过站位点);
    //   = 0 -> 完全关闭减速 (全程巡航速度, 不推荐: 易冲过);
    //   > 0 -> 手动指定 (建议取一个远小于常规减速距离的值, 且 > 到点阈值)。
    hold_point_slow_radius_ =
      declare_parameter<double>("hold_point_slow_radius", -1.0);
    // 拦截点站位间距系数: 车体中心停在"障碍 -> 占点中心"连线上距障碍
    //   k * astar_inflate_radius_ 处 (>=1 保证硬性间距有余量; 越接近 1 越贴近敌人)
    hold_point_standoff_scale_ =
      declare_parameter<double>("hold_point_standoff_scale", 1.10);
    // 回位死区 (m): 车体中心距占点中心小于该值且无威胁时不再微调, 避免抖动
    hold_point_recenter_deadband_ =
      declare_parameter<double>("hold_point_recenter_deadband", 0.08);
    // 占点状态发布话题 JSON 格式 (std_msgs/String), 供网页轮询/展示
    hold_point_status_topic_ =
      declare_parameter<std::string>(
      "hold_point_status_topic", "/hold_point_status");
    // 占点可视化话题 (nav_msgs/Path, map 系): 第 1 点=占点中心, 第 2 点=拦截点
    //   (无威胁时只有占点中心一个点), 供网页画圈/连线
    hold_point_viz_topic_ =
      declare_parameter<std::string>(
      "hold_point_viz_topic", "/hold_point_viz");

    // ---------------- 追击 (chase) 模式 ----------------
    // 网页: 锁定历史累计图后, 点"选择追击目标"进入选点态, 再点历史图上的目标
    //   物体 -> 锁定距该点最近的障碍簇作为追击目标。此后每帧:
    //     1) 用最新点云重新定位目标 (最近障碍簇), 记录其 3D 点云/轨迹;
    //     2) 维持"车体中心到目标中心约 chase_radius (默认 1.0 m)"的环形跟随
    //        (太近则退、太远则进); 若车体与目标连线被障碍挡住, 则用 A* 绕开;
    //     3) 目标丢失 (搜索半径内找不到) 时直接停车, 等待其重新出现。
    //        (丢失后的重寻/搜索点逻辑已移除, 以后再实现)
    //   控制复用 computeFollowCommand (与占点同一套纯方向跟踪)。
    //   安全口径与导航/占点一致: 硬性最小间距 = astar_inflate_radius_。
    chase_enabled_ =
      declare_parameter<bool>("chase_enabled", false);
    // 目标点击点 (map 系, m): 网页点击后写入, 用于首次锁定目标簇
    chase_pick_x_ = declare_parameter<double>("chase_pick_x", 0.0);
    chase_pick_y_ = declare_parameter<double>("chase_pick_y", 0.0);
    // 环形跟随半径 (m): 保持车体中心到目标中心约该距离 (太近退、太远进)
    chase_radius_ = declare_parameter<double>("chase_radius", 1.0);
    // 环形死区 (m): 距目标 |d - chase_radius_| 小于该值时不前后微调, 避免抖动
    chase_deadband_ = declare_parameter<double>("chase_deadband", 0.15);
    // 追击机动速度 (m/s): <= 0 时跟随 motion_target_speed_ (与正常巡航一致)
    chase_speed_ = declare_parameter<double>("chase_speed", -1.0);
    // 目标锁定半径 (m): 点选处该半径内最近障碍簇才算命中, 避免误锁远处障碍
    chase_lock_radius_ = declare_parameter<double>("chase_lock_radius", 1.0);
    // 目标重定位搜索半径 (m): 以上一帧目标位置为中心, 在该半径内找最近障碍簇
    //   作为本帧目标 (跟踪同一物体, 防止被周围障碍抢占)
    //   默认 5.0 m, 目标离开后重新进入该范围即可被重新捕获
    chase_track_radius_ = declare_parameter<double>("chase_track_radius", 5.0);
    // 目标轨迹记录时长 (s) 与记录间隔 (s): 供回溯/网页展示
    chase_track_history_sec_ =
      declare_parameter<double>("chase_track_history_sec", 30.0);
    chase_track_record_interval_ =
      declare_parameter<double>("chase_track_record_interval", 0.2);
    // 追击状态发布话题 JSON (std_msgs/String), 供网页展示目标/状态
    chase_status_topic_ =
      declare_parameter<std::string>(
      "chase_status_topic", "/chase_status");
    // 追击可视化话题 (nav_msgs/Path, map 系): 第 1 点=当前目标, 第 2 点=期望站位点
    chase_viz_topic_ =
      declare_parameter<std::string>(
      "chase_viz_topic", "/chase_viz");
    // 目标轨迹发布话题 (nav_msgs/Path, map 系): 记录的历史轨迹, 供网页画线
    chase_track_topic_ =
      declare_parameter<std::string>(
      "chase_track_topic", "/chase_track");

    motion_publisher_ =
      create_publisher<std_msgs::msg::UInt8MultiArray>(
      motion_topic_, rclcpp::QoS(10));

    // ---------------- SocketCAN 直发 (运动中把同一帧写进 CAN 总线) ----------------
    // 打开后不再依赖任何外部桥接节点/脚本:
    //   运动线程算出 8 字节运动帧 -> 除发布 motion_topic 外, 同时 write() 到 can0。
    // 接口需先拉起:
    //   sudo ip link set can0 down
    //   sudo ip link set can0 type can bitrate 1000000
    //   sudo ip link set can0 up
    // 用 candump -t d can0,0FF:7FF 可看到本节点发出的报文。
    can_enabled_ =
      declare_parameter<bool>("can_enabled", false);
    can_interface_ =
      declare_parameter<std::string>("can_interface", "can0");
    can_id_ = static_cast<canid_t>(
      declare_parameter<int>("can_id", 0xFF) & 0x7FF);
    can_send_period_ms_ =
      declare_parameter<int>("can_send_period_ms", 10);

```

## 3. 成员变量区（private, 7569-7924）

按主题切块，便于你逐块搬到 `config/` 结构体或各 tasks 类。

### `成员变量: 点云基础 / 渲染周期`

- **源码行**: `7570-7576` (7 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7570-7576)

```cpp
  std::string cloud_topic_;
  int render_period_ms_{200};

  std::mutex cloud_mutex_;
  std::vector<Point> latest_cloud_;
  bool received_cloud_{false};
  std::size_t cloud_frames_{0};
```

### `成员变量: 调试日志 / 异常快照 / 跳变检测`

- **源码行**: `7578-7611` (34 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7578-7611)

```cpp
  // ---------------- 调试日志 / 异常快照 ----------------
  bool debug_log_enabled_{true};
  std::string debug_log_dir_{"log"};
  bool debug_pcd_enabled_{true};
  int debug_pcd_max_files_{200};
  double debug_cloud_count_jump_ratio_{3.0};
  int debug_cloud_count_jump_delta_{20000};
  double debug_centroid_jump_m_{1.5};
  std::mutex debug_log_mutex_;
  std::ofstream debug_cloud_log_;        // 逐帧点云统计 CSV
  std::size_t debug_pcd_saved_{0};       // 已保存的 PCD 快照数
  std::size_t debug_frame_index_{0};     // 调试日志帧号 (独立于 cloud_frames_)
  bool debug_prev_valid_{false};         // 上帧统计是否有效 (用于突增判定)
  // 里程计位姿跳变检测 (仅在 odom 回调线程访问, 无需加锁)
  bool debug_prev_odom_valid_{false};
  double debug_prev_odom_x_{0.0};
  double debug_prev_odom_y_{0.0};
  double debug_prev_odom_yaw_{0.0};
  double debug_odom_jump_m_{0.5};        // 单帧位移超此值判为跳变 (m)
  double debug_odom_jump_yaw_{0.52};     // 单帧偏航超此值判为跳变 (rad, ~30°)
  // 位姿跳变"拒绝窗口": 检出跳变后, 截止时刻之前到达的点云帧一律丢弃。
  //   在 odom 回调线程写、cloud 回调线程读, 故均用原子量保证跨线程可见性。
  //   截止时刻用 header.stamp 的秒数 (本机时钟, 与点云同源) 表示。
  double pose_jump_reject_sec_{0.8};     // 拒绝窗口时长 (s), <=0 关闭
  std::atomic<bool> pose_jump_has_until_{false};
  std::atomic<double> pose_jump_until_sec_{0.0};
  // 历史图占据比例暴涨检测 (仅在渲染线程访问, 无需加锁)
  bool debug_prev_occ_valid_{false};
  double debug_prev_occ_ratio_{0.0};
  double debug_occ_ratio_jump_{0.15};    // 单帧占比涨幅超此值 -> WARN
  std::size_t debug_prev_points_{0};     // 上帧点数
  double debug_prev_cx_{0.0};            // 上帧质心
  double debug_prev_cy_{0.0};
  double debug_prev_cz_{0.0};
```

### `成员变量: 参数与配置快照`

- **源码行**: `7613-7615` (3 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7613-7615)

```cpp
  std::mutex config_mutex_;
  Config config_{};
  bool config_changed_{false};
```

### `成员变量: 订阅 / 定时器 / 单帧图发布`

- **源码行**: `7617-7630` (14 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7617-7630)

```cpp
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_subscription_;

  rclcpp::TimerBase::SharedPtr render_timer_;
  rclcpp::TimerBase::SharedPtr warn_timer_;

  std::string image_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    image_publisher_;

  std::string cloud_preview_topic_;
  int cloud_preview_max_points_{15000};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_preview_publisher_;
```

### `成员变量: 历史图缓冲`

- **源码行**: `7632-7638` (7 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7632-7638)

```cpp
  std::deque<TimedCloud> history_;
  // 网页"刷新累计图"请求标志, 由参数回调置位, 渲染线程消费
  std::atomic<bool> history_reset_requested_{false};

  std::string history_image_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    history_image_publisher_;
```

### `成员变量: 地图导出 / 导入`

- **源码行**: `7640-7655` (16 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7640-7655)

```cpp
  // ---------- 历史累计图导出 / 导入 ----------
  // 导出目录 (由网页端写入 export_map_dir 参数指定), 本节点在该目录生成
  //   map.pgm / map.yaml / map_clouds.bin 三个中间文件, 网页端打包 zip 下载。
  std::string export_map_dir_;
  std::string export_map_name_;
  std::atomic<bool> export_map_requested_{false};
  // 导入目录 (网页端解包 zip 后写入), 本节点从该目录读取上述三个文件恢复。
  std::string import_map_dir_;
  std::atomic<bool> import_map_requested_{false};
  // 导入的成品历史图 (mono8: 0=占据, 205=未知): 非空时作为网页底图优先发布,
  //   并作为动态障碍基准 (static_obstacle_mask_) 的来源。
  std::mutex imported_map_mutex_;
  std::vector<uint8_t> imported_map_pixels_;
  int imported_map_width_{0};
  int imported_map_height_{0};
  bool imported_map_valid_{false};
```

### `成员变量: 动态障碍 / 位姿修正`

- **源码行**: `7657-7704` (48 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7657-7704)

```cpp
  // ---------- 动态障碍 (可变障碍): 锁定历史累计图后, 新出现的障碍单独成层 ----------
  // 静态基准栅格: 锁定那一刻由当前历史累积图投票生成, 作为"什么是静态环境"的依据
  std::mutex dynamic_mutex_;
  std::vector<uint8_t> static_obstacle_mask_;
  // 配准评分容差掩码: static_obstacle_mask_ 再膨胀 1 格 (buildStaticBaseline 生成)。
  //   搜索步长 (0.2 m/1.0 m) 远大于栅格边长, 若按"恰好落在占据格"评分, 候选
  //   分数退化为噪声 -> 修正量乱跳。预膨胀后评分仍是单次查表。
  std::vector<uint8_t> static_obstacle_tolerance_;
  // 本帧绿色层点云 (= 自过滤整帧 − 基准占据格), 供渲染线程发布绿色栅格图
  std::vector<Point> latest_dynamic_obstacles_;
  std::atomic<bool> dynamic_obstacle_enabled_{false};
  std::string pose_correction_topic_{"/pose_correction"};
  double pose_correction_global_period_sec_{1.0};
  double pose_correction_global_range_m_{15.0};
  double pose_correction_local_range_m_{1.0};
  double pose_correction_score_min_{0.12};
  double pose_correction_max_delta_m_{0.3};
  // 时序滤波: EMA 平滑系数 (1 = 不平滑) 与滞回增益阈值
  //   (新解评分须比当前解高出该值才接受, 否则保持当前修正量不动)
  double pose_correction_ema_alpha_{0.5};
  double pose_correction_score_gain_{0.05};
  // 全局兜底搜索触发条件: 局部搜索连续失败达到该帧数才做 ±15 m/±180° 全搜
  int pose_correction_global_trigger_fails_{5};
  // 修正来源: true=本节点 2D 配准 (估计+发布); false=订阅外部 /pose_correction
  bool apply_pose_correction_{false};
  std::atomic<int> pose_correction_local_fail_count_{0};
  std::atomic<double> pose_correction_last_global_sec_{
    -std::numeric_limits<double>::infinity()};
  std::mutex pose_correction_mutex_;
  double baseline_pose_x_{0.0};
  double baseline_pose_y_{0.0};
  double baseline_pose_yaw_{0.0};
  bool baseline_pose_valid_{false};
  // 位姿修正量 (map->odom, SE(3)): 由本节点自带 2D 栅格配准提供
  PoseCorrection pose_correction_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_correction_publisher_;
  // 外部修正输入订阅 (apply_pose_correction=false 时消费; 内部模式下忽略回环)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_correction_sub_;
  std::string dynamic_obstacle_topic_;
  std::string dynamic_obstacle_image_topic_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    dynamic_obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    dynamic_obstacle_image_publisher_;
  // 上次是否已发布过动态障碍图: 用于解锁后补发一张空图, 让网页绿色层消失
  bool dynamic_image_published_{false};
```

### `成员变量: 自过滤点云发布`

- **源码行**: `7706-7715` (10 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7706-7715)

```cpp
  // ---------- 自过滤点云 (供 Nav2 obstacle_layer 的清除射线) ----------
  // 车体自过滤后的完整当前帧点云 (静态+动态), 以 base_link 帧发布。
  // 作用: obstacle_layer 的 pointcloud 源此前因无人发布该话题而空转, 补上之后
  // 清除射线从车体当前位置射出, 可把旧红色残留清成空闲 (详见 publishSelfFilteredCloud)。
  std::string self_filtered_topic_;
  double self_filtered_publish_rate_{10.0};
  int self_filtered_max_points_{6000};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    self_filtered_publisher_;
  rclcpp::Time self_filtered_last_publish_{0, 0, RCL_ROS_TIME};
```

### `成员变量: 清除射线`

- **源码行**: `7717-7725` (9 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7717-7725)

```cpp
  // ---------- 清除射线点云 (让红色代价区域实时跟随绿色障碍消失) ----------
  // 每帧向 nav2_params.yaml 的 clear_ring 源 (marking:false / clearing:true)
  // 发一圈 base_link 系的稀疏点, 让 obstacle_layer 从车体原点向四周射线清除
  // 旧红格。见 publishClearRingCloud。
  std::string clear_ring_topic_;
  int clear_ring_points_{72};
  double clear_ring_radius_{1.2};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    clear_ring_publisher_;
```

### `成员变量: 清空 Nav2 代价地图`

- **源码行**: `7727-7735` (9 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7727-7735)

```cpp
  // ---------- 刷新历史图时整张清空 Nav2 红色代价区 ----------
  // 见 clearCostmaps: 异步调 ClearEntireCostmap 清空局部/全局代价地图。
  bool clear_costmap_on_history_reset_{true};
  std::string clear_local_costmap_service_;
  std::string clear_global_costmap_service_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
    clear_local_costmap_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
    clear_global_costmap_client_;
```

### `成员变量: 运动帧框架 / 输入话题`

- **源码行**: `7737-7749` (13 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7737-7749)

```cpp
  // ---------- 导航运动控制 (Nav2 全局路径 -> 8 字节运动帧) ----------
  // 运动帧话题: uint8[8], 对应接收方 0xFF 报文 Data[0..7]:
  //   [0] bit0=control bit1=shoot, [1-2] tar_yaw(H/L), [3-4] tar_pitch(H/L),
  //   [5-7] vx/vy 各 12 位有符号量 (单位 1/scale m/s)
  std::string motion_topic_;
  int motion_control_{1};          // control 位取值 (0=停止 / 1=运动)
  double motion_vx_scale_{kMotionSpeedScale};  // vx 量化系数: raw = vx(m/s) * scale
  double motion_vy_scale_{kMotionSpeedScale};  // vy 量化系数: raw = vy(m/s) * scale
  double motion_rate_hz_{100.0};   // 运动帧下发频率 (100 Hz => 10 ms 一帧)

  std::string odom_topic_;
  std::string global_plan_topic_;
  std::string local_plan_topic_;
```

### `成员变量: 路径跟随参数`

- **源码行**: `7751-7775` (25 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7751-7775)

```cpp
  // 路径跟随参数 (速度以车体系为参考: vx=车头前方, vy=车辆左手边)
  double motion_target_speed_{1.00};
  double motion_lookahead_{0.45};
  double motion_lookahead_speed_gain_{0.52};
  double motion_lookahead_max_{1.50};
  double motion_yaw_kp_{1.20};
  double motion_goal_tolerance_{0.25};
  // 到达后位置校验阈值 (m): 车体中心 -> 用户目标点; 超过则用自建 A* 重新导航
  double motion_goal_check_tolerance_{0.35};
  // 位置校验失败时用自建 A* 重新导航的最大重试次数
  int motion_goal_check_max_retry_{2};
  // 位置校验失败重试计数 (每次新导航/新目标点清零, 达到上限则彻底停车)
  int motion_goal_check_retry_{0};
  // "校验失败已改用 A* 重导航"标志: true 时暂时无视 Nav2 路径, 由 A* 接管直到
  //   车体驶离到达半径 (再交还 Nav2), 避免旧 Nav2 末点让每帧都重触发校验
  bool motion_goal_check_replan_{false};
  double motion_max_lateral_{0.0};
  double motion_slow_radius_{0.0};
  double motion_min_speed_{0.08};
  // "车体中心 -> 雷达原点" 的平移, 表达在车体系, 数值为 lidar_to_base
  // (雷达系表达) 的反向量。使用时必须做减法反推车体中心:
  //   base = lidar_odom - R(yaw) * motion_base_to_lidar   (勿写成加法!)
  // 详见 computeFollowCommand 步骤 0 处的说明。
  double motion_base_to_lidar_x_{-0.239};
  double motion_base_to_lidar_y_{0.158};
```

### `成员变量: A* 兜底参数`

- **源码行**: `7777-7788` (12 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7777-7788)

```cpp
  // ---- 自建 A* 兜底规划器 (详见 planAstarPath) ----
  bool astar_enabled_{true};           // 是否启用 A* 兜底 (/plan 不可用时)
  double astar_speed_{-1.0};           // 兜底跟随速度 (m/s), <=0 跟随 motion_target_speed
  double astar_inflate_radius_{0.50};  // 硬性最小间距 (m): 车体中心 -> 障碍
  double astar_cost_reach_{0.60};      // 软性避让范围 (m): 障碍邻近代价影响半径
  double astar_obstacle_penalty_{0.8}; // 靠障碍代价惩罚权重 (越大越远离障碍)
  double astar_path_spacing_{0.15};    // A* 路径抽稀间距 (m), <=0 不抽稀
  // A* 重算限频 (详见参数声明处): 时间上限 + 位移触发, 防止 100 Hz 重算阻塞运动线程
  double astar_replan_interval_{0.20};
  double astar_replan_move_dist_{0.05};
  std::string goal_pose_topic_;        // Web 目标点话题 (PoseStamped, map 系)
  std::string astar_plan_topic_;       // A* 路径发布话题 (Path, map 系)
```

### `成员变量: 占点`

- **源码行**: `7790-7816` (27 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7790-7816)

```cpp
  // ---------- 占点 (hold point) 行为 ----------
  // 行为: 车辆前往"占点中心"驻守, 实时计算"威胁障碍 -> 占点中心"的行进路线,
  //   抢先机动到该路线上用车体阻挡。无威胁时回到占点中心。
  // 安全: 硬性间距复用 astar_inflate_radius_ (0.50 m, 与 Nav2 robot_radius 同口径),
  //   任何时刻车体中心到障碍都不得小于该值 —— 若机动会导致越界则拒绝该机动。
  mutable std::mutex hold_point_mutex_;
  bool hold_point_enabled_{false};     // 占点模式开关 (网页按钮)
  double hold_point_x_{0.0};           // 占点中心 (map 系, m)
  double hold_point_y_{0.0};
  double hold_point_radius_{1.50};     // 占点范围 (m): 半径内的障碍才算威胁
  double hold_point_speed_{-1.0};      // 占点机动速度 (m/s), <=0 跟随 motion_target_speed
  double hold_point_slow_radius_{-1.0};  // 占点减速起始距离 (m), <0 自动取常规的一半
  double hold_point_standoff_scale_{1.10};  // 拦截点距障碍 = k * 硬性间距
  double hold_point_recenter_deadband_{0.08};    // 回位死区 (m)
  std::string hold_point_status_topic_;
  std::string hold_point_viz_topic_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr hold_point_viz_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    hold_point_status_publisher_;

  // 当前仲裁出的威胁 (供网页展示): 位置/到中心直线距离/是否已就位阻挡
  bool hold_point_threat_valid_{false};
  double hold_point_threat_x_{0.0};
  double hold_point_threat_y_{0.0};
  double hold_point_threat_dist_{0.0};   // 威胁障碍到占点中心的直线距离 (m)
  double hold_point_block_x_{0.0};       // 计算出的阻挡站位点
  double hold_point_block_y_{0.0};
```

### `成员变量: 追击`

- **源码行**: `7818-7846` (29 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7818-7846)

```cpp
  // ---------- 追击 (chase) 模式 ----------
  // 行为: 网页锁定历史图后点"选择追击目标"并点图 -> 锁定最近障碍簇为目标;
  //   每帧重新定位目标并维持约 chase_radius_ 的环形跟随 (太近退/太远进);
  //   车体与目标连线被障碍挡住时用 A* 绕行; 目标丢失时直接停车等待重新出现。
  //   控制复用 computeFollowCommand。
  mutable std::mutex chase_mutex_;
  bool chase_enabled_{false};        // 追击模式开关 (网页按钮)
  double chase_pick_x_{0.0};         // 目标点选点 (map 系, m), 用于首次锁定目标簇
  double chase_pick_y_{0.0};
  double chase_radius_{1.0};         // 环形跟随半径 (m)
  double chase_deadband_{0.15};      // 环形死区 (m)
  double chase_speed_{-1.0};         // 追击速度 (m/s), <=0 跟随 motion_target_speed_
  double chase_lock_radius_{1.0};    // 首次锁定命中半径 (m)
  double chase_track_radius_{5.0};   // 每帧重定位搜索半径 (m)
  double chase_track_history_sec_{30.0};     // 目标轨迹记录时长 (s)
  double chase_track_record_interval_{0.2};  // 目标轨迹记录间隔 (s)
  std::string chase_status_topic_;
  std::string chase_viz_topic_;
  std::string chase_track_topic_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr chase_viz_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr chase_track_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chase_status_publisher_;

  // 目标锁定/跟踪状态 (供追击主逻辑与网页展示)
  bool chase_target_valid_{false};      // 是否已锁定目标
  double chase_target_x_{0.0};          // 当前目标位置 (map 系, m)
  double chase_target_y_{0.0};
  double chase_goal_x_{0.0};            // 本帧期望到达点 (站位点)
  double chase_goal_y_{0.0};
```

### `成员变量: 轨迹 + A* 缓存 + 目标点缓存`

- **源码行**: `7847-7867` (21 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7847-7867)

```cpp
  // 目标历史轨迹 (map 系): 记录目标位置与时间, 供丢失后回溯与网页绘制
  struct ChaseTrackPoint
  {
    rclcpp::Time stamp;
    double x{0.0};
    double y{0.0};
    std::vector<Point> cloud;   // 记录该时刻目标簇的 3D 点云 (map 系)
  };
  std::deque<ChaseTrackPoint> chase_track_;
  rclcpp::Time chase_last_record_time_{0, 0, RCL_ROS_TIME};
  // A* 路径缓存 (供限频复用): 上次规划结果 + 规划时刻 + 规划时车体中心
  std::vector<Point> astar_cached_path_;
  rclcpp::Time astar_last_plan_time_{0, 0, RCL_ROS_TIME};
  double astar_last_plan_x_{0.0};
  double astar_last_plan_y_{0.0};
  bool astar_cache_valid_{false};
  // Web 目标点缓存 (由 goalPoseCallback 写入, 供 A* 作为终点)
  float goal_x_{0.0F};
  float goal_y_{0.0F};
  bool goal_received_{false};
  rclcpp::Time goal_recv_time_{0, 0, RCL_ROS_TIME};
```

### `成员变量: 运动状态 / 里程计 / 路径缓冲`

- **源码行**: `7869-7903` (35 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7869-7903)

```cpp
  std::mutex motion_mutex_;
  bool motion_active_{false};        // 是否正在按路径运动
  // "等待路径"态: /plan 过期/缺失时不再真正停车, 而是保持运动态、下发 0 速等待
  // Nav2 重规划; 新路径一到立即恢复跟随。自建 A* 兜底也只在等待态下进行。
  bool motion_waiting_{false};       // 是否处于"无有效路径, 等待重规划"状态
  double odom_x_{0.0};               // 最新雷达位姿 (map 系)
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool odom_received_{false};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<Point> motion_plan_;   // Nav2 全局路径 (map 系)
  bool motion_plan_received_{false};
  rclcpp::Time motion_plan_stamp_{0, 0, RCL_ROS_TIME};
  // 全局路径新鲜度监督: /plan 超过 motion_plan_timeout_ 秒未更新 -> 判定
  // Nav2 已停止重规划(导航完成/中止/取消), 立即停车, 不再按过期路径盲跑。
  // recv_time 用本机时钟记录 (见 planCallback)。
  double motion_plan_timeout_{1.0};
  rclcpp::Time motion_plan_recv_time_{0, 0, RCL_ROS_TIME};
  std::vector<Point> local_plan_;    // Nav2 局部路径 (map 系, 含避障)
  bool local_plan_received_{false};
  rclcpp::Time local_plan_stamp_{0, 0, RCL_ROS_TIME};
  double local_plan_timeout_{0.5};   // 局部路径超时 (s), 超时回退全局路径

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
    local_plan_subscription_;
  // Web 目标点订阅 (自建 A* 兜底规划器的终点) 与 A* 路径发布 (网页黄色显示)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_pose_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr astar_plan_publisher_;
  rclcpp::TimerBase::SharedPtr motion_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
    motion_publisher_;
```

### `成员变量: 运动请求 + 可视化快照`

- **源码行**: `7905-7911` (7 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7905-7911)

```cpp
  // 网页"开始运动"/"停止运动"请求标志, 由参数回调置位, 运动线程消费
  std::atomic<bool> motion_start_requested_{false};
  std::atomic<bool> motion_stop_requested_{false};

  // 路径跟随可视化快照 (前视点/车体中心/速度矢量/车头航向), 供网页读取
  std::mutex lookahead_mutex_;
  LookaheadView lookahead_view_{};
```

### `成员变量: SocketCAN 直发`

- **源码行**: `7913-7921` (9 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7913-7921)

```cpp
  // ---------- SocketCAN 直发 (把运动帧原样写进 CAN 总线) ----------
  // 开启后无需任何桥接节点: 运动线程产出的 8 字节帧同时发布话题并 write() 到 can0
  std::atomic<bool> can_enabled_{false};
  std::string can_interface_{"can0"};
  canid_t can_id_{0xFF};
  int can_send_period_ms_{10};       // 直发限频周期 (10 ms => 100 Hz 上限)
  int can_socket_fd_{-1};
  std::atomic<uint64_t> can_sent_frames_{0};
  std::chrono::steady_clock::time_point can_last_send_{};
```

### `成员变量: 参数回调句柄`

- **源码行**: `7923-7924` (2 行)
- **目标位置**: `config/config.hpp + 各 tasks 类的私有成员`
- **职责**: （变量清单见源码, 拆分时按主题分配到对应类）
- **依赖 / 备注**: block(7923-7924)

```cpp
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;
```
