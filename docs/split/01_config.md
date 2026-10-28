# 01 参数配置（config/，只有 yaml）

**约定**：`Navigation/config/` 只放 yaml，专用于调参；不放头文件、不放源码、不放数据结构。

| 文件 | 内容 |
|---|---|
| `config/params.yaml` | 本节点全部可调参数（下表即其全部键） |
| `config/nav2_params.yaml` | Nav2 栈参数（人工维护） |

Nav2 行为树 XML 不是 yaml：`launch/behavior_trees/navigate_to_pose_w_replanning_5hz.xml`，launch 用 `RewrittenYaml` 注入绝对路径。

**数据类型与常量不在 config**：`Point` / `Config` / `LookaheadView` / `PoseCorrection` / `HoldObstacleCluster` / `FootprintBounds` / `ChaseTrackPoint` → `tools/types.hpp`；运动帧协议常量 → `tools/constants.hpp`；带 `rclcpp::Time` 的 `TimedCloud` → `io/timed_cloud.hpp`。源码见 `02_tools.md` 第 1 节、`03_io.md` 第 7 节。

## 参数清单（自动扫描 118-604 行的 declare_parameter）

共 **122** 个参数（这是 `config/params.yaml` 的全部键）。

| 主题 | 参数 | 类型 | 默认值 | 声明行 |
|---|---|---|---|---|
| 基础建图 (写入 Config) | `resolution` | `double` | `0.03` | 121 |
| 基础建图 (写入 Config) | `zmin` | `double` | `-2.25` | 123 |
| 基础建图 (写入 Config) | `zmax` | `double` | `0.22` | 125 |
| 基础建图 (写入 Config) | `dilate` | `int` | `1` | 127 |
| 基础建图 (写入 Config) | `baseline_dilate` | `int` | `-1` | 129 |
| 基础建图 (写入 Config) | `self_filter_size` | `double` | `1.0` | 132 |
| 基础建图 (写入 Config) | `xmin` | `double` | `-20.0` | 135 |
| 基础建图 (写入 Config) | `ymin` | `double` | `-20.0` | 137 |
| 基础建图 (写入 Config) | `xmax` | `double` | `20.0` | 139 |
| 基础建图 (写入 Config) | `ymax` | `double` | `20.0` | 141 |
| 基础建图 (写入 Config) | `output` | `std::string` | `"/tmp/realtime_map.pgm"` | 145 |
| 基础建图 (写入 Config) | `history_output` | `std::string` | `"/tmp/realtime_map_history.pgm"` | 149 |
| 基础建图 (写入 Config) | `history_duration` | `double` | `300.0` | 151 |
| 基础建图 (写入 Config) | `history_max_points_per_frame` | `int` | `5000` | 153 |
| 基础建图 (写入 Config) | `history_min_hits` | `int` | `2` | 155 |
| 基础建图 (写入 Config) | `cloud_topic` | `std::string` | `"/cloud_registered"` | 164 |
| 调试日志 / 异常快照 | `debug_log_enabled` | `bool` | `true` | 169 |
| 调试日志 / 异常快照 | `debug_log_dir` | `std::string` | `"log"` | 171 |
| 调试日志 / 异常快照 | `debug_pcd_enabled` | `bool` | `true` | 174 |
| 调试日志 / 异常快照 | `debug_pcd_max_files` | `int` | `200` | 177 |
| 调试日志 / 异常快照 | `debug_cloud_count_jump_ratio` | `double` | `3.0` | 180 |
| 调试日志 / 异常快照 | `debug_cloud_count_jump_delta` | `int` | `20000` | 182 |
| 调试日志 / 异常快照 | `debug_centroid_jump_m` | `double` | `1.5` | 185 |
| 调试日志 / 异常快照 | `debug_odom_jump_m` | `double` | `0.5` | 188 |
| 调试日志 / 异常快照 | `debug_odom_jump_yaw_deg` | `double` | `30.0` | 190 |
| 调试日志 / 异常快照 | `pose_jump_reject_sec` | `double` | `0.8` | 194 |
| 调试日志 / 异常快照 | `debug_occ_ratio_jump` | `double` | `0.15` | 197 |
| 渲染 / 话题名 | `render_period_ms` | `int` | `150` | 200 |
| 渲染 / 话题名 | `clear_history` | `bool` | `false` | 203 |
| 渲染 / 话题名 | `cloud_preview_topic` | `std::string` | `"/realtime_pgm/cloud"` | 224 |
| 渲染 / 话题名 | `cloud_preview_max_points` | `int` | `15000` | 227 |
| 渲染 / 话题名 | `history_image_topic` | `std::string` | `"/realtime_pgm/image_history"` | 231 |
| 渲染 / 话题名 | `image_topic` | `std::string` | `"/realtime_pgm/image"` | 347 |
| 地图导出 / 导入 | `export_map_dir` | `std::string` | `"/tmp/pgm_export"` | 213 |
| 地图导出 / 导入 | `export_map_name` | `std::string` | `"map_export.zip"` | 220 |
| 地图导出 / 导入 | `export_map_request` | `bool` | `false` | 214 |
| 地图导出 / 导入 | `import_map_dir` | `std::string` | `"/tmp/pgm_import"` | 216 |
| 地图导出 / 导入 | `import_map_request` | `bool` | `false` | 217 |
| 动态障碍模式 | `dynamic_obstacle_enabled` | `bool` | `false` | 241 |
| 动态障碍模式 | `dynamic_obstacle_topic` | `std::string` | `"/cloud_dynamic_obstacles"` | 245 |
| 动态障碍模式 | `dynamic_obstacle_image_topic` | `std::string` | `"/realtime_pgm/image_dynamic"` | 249 |
| 位姿校正 (map->odom) | `pose_correction_topic` | `std::string` | `"/pose_correction"` | 255 |
| 位姿校正 (map->odom) | `pose_correction_global_period_sec` | `double` | `1.0` | 257 |
| 位姿校正 (map->odom) | `pose_correction_global_range_m` | `double` | `15.0` | 259 |
| 位姿校正 (map->odom) | `pose_correction_local_range_m` | `double` | `1.0` | 261 |
| 位姿校正 (map->odom) | `pose_correction_score_min` | `double` | `0.12` | 263 |
| 位姿校正 (map->odom) | `pose_correction_max_delta_m` | `double` | `0.3` | 265 |
| 位姿校正 (map->odom) | `pose_correction_ema_alpha` | `double` | `0.5` | 269 |
| 位姿校正 (map->odom) | `pose_correction_score_gain` | `double` | `0.05` | 271 |
| 位姿校正 (map->odom) | `pose_correction_global_trigger_fails` | `int` | `5` | 274 |
| 位姿校正 (map->odom) | `apply_pose_correction` | `bool` | `false` | 282 |
| 自过滤点云 (Nav2 clearing) | `self_filtered_topic` | `std::string` | `"/cloud_self_filtered"` | 297 |
| 自过滤点云 (Nav2 clearing) | `self_filtered_publish_rate` | `double` | `10.0` | 301 |
| 自过滤点云 (Nav2 clearing) | `self_filtered_max_points` | `int` | `6000` | 304 |
| 清除射线环 | `clear_ring_topic` | `std::string` | `"/cloud_clear_ring"` | 312 |
| 清除射线环 | `clear_ring_points` | `int` | `72` | 315 |
| 清除射线环 | `clear_ring_radius` | `double` | `1.2` | 318 |
| 清空 Nav2 代价图 | `clear_costmap_on_history_reset` | `bool` | `true` | 327 |
| 清空 Nav2 代价图 | `clear_local_costmap_service` | `std::string` | `"/local_costmap/clear_entirely_local_costmap"` | 331 |
| 清空 Nav2 代价图 | `clear_global_costmap_service` | `std::string` | `"/global_costmap/clear_entirely_global_costmap"` | 335 |
| 运动帧 / 输入话题 | `motion_topic` | `std::string` | `"/nav_motion_cmd"` | 370 |
| 运动帧 / 输入话题 | `odom_topic` | `std::string` | `"/Odometry"` | 374 |
| 运动帧 / 输入话题 | `global_plan_topic` | `std::string` | `"/plan"` | 376 |
| 运动帧 / 输入话题 | `local_plan_topic` | `std::string` | `"/local_plan"` | 380 |
| 运动帧 / 输入话题 | `local_plan_timeout` | `double` | `0.5` | 382 |
| 运动帧 / 输入话题 | `motion_control` | `int` | `1` | 385 |
| 运动帧 / 输入话题 | `motion_vx_scale` | `double` | `kMotionSpeedScale` | 387 |
| 运动帧 / 输入话题 | `motion_vy_scale` | `double` | `kMotionSpeedScale` | 389 |
| 运动帧 / 输入话题 | `motion_rate_hz` | `double` | `100.0` | 391 |
| 路径跟随 | `motion_target_speed` | `double` | `1.00` | 395 |
| 路径跟随 | `motion_lookahead` | `double` | `0.45` | 397 |
| 路径跟随 | `motion_lookahead_speed_gain` | `double` | `0.52` | 402 |
| 路径跟随 | `motion_lookahead_max` | `double` | `1.50` | 405 |
| 路径跟随 | `motion_yaw_kp` | `double` | `1.20` | 407 |
| 路径跟随 | `motion_goal_tolerance` | `double` | `0.25` | 409 |
| 路径跟随 | `motion_goal_check_tolerance` | `double` | `0.35` | 416 |
| 路径跟随 | `motion_goal_check_max_retry` | `int` | `2` | 419 |
| 路径跟随 | `motion_max_lateral` | `double` | `0.0` | 421 |
| 路径跟随 | `motion_plan_timeout` | `double` | `1.0` | 427 |
| 路径跟随 | `motion_slow_radius` | `double` | `0.0` | 431 |
| 路径跟随 | `motion_min_speed` | `double` | `0.08` | 433 |
| 路径跟随 | `motion_base_to_lidar_x` | `double` | `-0.239` | 435 |
| 路径跟随 | `motion_base_to_lidar_y` | `double` | `0.158` | 437 |
| A* 兜底 | `astar_enabled` | `bool` | `true` | 447 |
| A* 兜底 | `astar_speed` | `double` | `-1.0` | 451 |
| A* 兜底 | `astar_inflate_radius` | `double` | `0.50` | 456 |
| A* 兜底 | `astar_obstacle_penalty` | `double` | `0.8` | 460 |
| A* 兜底 | `astar_cost_reach` | `double` | `0.60` | 465 |
| A* 兜底 | `astar_path_spacing` | `double` | `0.15` | 469 |
| A* 兜底 | `astar_replan_interval` | `double` | `0.20` | 479 |
| A* 兜底 | `astar_replan_move_dist` | `double` | `0.05` | 482 |
| A* 兜底 | `goal_pose_topic` | `std::string` | `"/nav_goal_pose"` | 486 |
| A* 兜底 | `astar_plan_topic` | `std::string` | `"/astar_plan"` | 490 |
| 网页一次性触发 | `start_motion` | `bool` | `false` | 493 |
| 网页一次性触发 | `stop_motion` | `bool` | `false` | 494 |
| 占点 | `hold_point_enabled` | `bool` | `false` | 505 |
| 占点 | `hold_point_x` | `double` | `0.0` | 507 |
| 占点 | `hold_point_y` | `double` | `0.0` | 508 |
| 占点 | `hold_point_radius` | `double` | `1.50` | 511 |
| 占点 | `hold_point_speed` | `double` | `-1.0` | 514 |
| 占点 | `hold_point_slow_radius` | `double` | `-1.0` | 521 |
| 占点 | `hold_point_standoff_scale` | `double` | `1.10` | 525 |
| 占点 | `hold_point_recenter_deadband` | `double` | `0.08` | 528 |
| 占点 | `hold_point_status_topic` | `std::string` | `"/hold_point_status"` | 532 |
| 占点 | `hold_point_viz_topic` | `std::string` | `"/hold_point_viz"` | 537 |
| 追击 | `chase_enabled` | `bool` | `false` | 550 |
| 追击 | `chase_pick_x` | `double` | `0.0` | 552 |
| 追击 | `chase_pick_y` | `double` | `0.0` | 553 |
| 追击 | `chase_radius` | `double` | `1.0` | 555 |
| 追击 | `chase_deadband` | `double` | `0.15` | 557 |
| 追击 | `chase_speed` | `double` | `-1.0` | 559 |
| 追击 | `chase_lock_radius` | `double` | `1.0` | 561 |
| 追击 | `chase_track_radius` | `double` | `5.0` | 565 |
| 追击 | `chase_track_history_sec` | `double` | `30.0` | 568 |
| 追击 | `chase_track_record_interval` | `double` | `0.2` | 570 |
| 追击 | `chase_status_topic` | `std::string` | `"/chase_status"` | 574 |
| 追击 | `chase_viz_topic` | `std::string` | `"/chase_viz"` | 578 |
| 追击 | `chase_track_topic` | `std::string` | `"/chase_track"` | 582 |
| SocketCAN 直发 | `can_enabled` | `bool` | `false` | 597 |
| SocketCAN 直发 | `can_interface` | `std::string` | `"can0"` | 599 |
| SocketCAN 直发 | `can_id` | `int` | `0xFF` | 601 |
| SocketCAN 直发 | `can_send_period_ms` | `int` | `10` | 603 |

## params.yaml 骨架

键名与默认值均由源码自动提取，可直接落成 `config/params.yaml`。

```yaml
# Navigation 可调参数（由 test_navigation.cpp 的 declare_parameter 提炼）
# 用法: 由 src/parameter_callback.cpp 通过 declare_parameter 默认值覆盖读取

# ==================== 基础建图 (写入 Config) ====================
resolution: 0.03                       # double, 默认 0.03
zmin: -2.25                            # double, 默认 -2.25
zmax: 0.22                             # double, 默认 0.22
dilate: 1                              # int, 默认 1
baseline_dilate: -1                    # int, 默认 -1
self_filter_size: 1.0                  # double, 默认 1.0
xmin: -20.0                            # double, 默认 -20.0
ymin: -20.0                            # double, 默认 -20.0
xmax: 20.0                             # double, 默认 20.0
ymax: 20.0                             # double, 默认 20.0
output: "/tmp/realtime_map.pgm"        # std::string, 默认 "/tmp/realtime_map.pgm"
history_output: "/tmp/realtime_map_history.pgm" # std::string, 默认 "/tmp/realtime_map_history.pgm"
history_duration: 300.0                # double, 默认 300.0
history_max_points_per_frame: 5000     # int, 默认 5000
history_min_hits: 2                    # int, 默认 2
cloud_topic: "/cloud_registered"       # std::string, 默认 "/cloud_registered"

# ==================== 调试日志 / 异常快照 ====================
debug_log_enabled: true                # bool, 默认 true
debug_log_dir: "log"                   # std::string, 默认 "log"
debug_pcd_enabled: true                # bool, 默认 true
debug_pcd_max_files: 200               # int, 默认 200
debug_cloud_count_jump_ratio: 3.0      # double, 默认 3.0
debug_cloud_count_jump_delta: 20000    # int, 默认 20000
debug_centroid_jump_m: 1.5             # double, 默认 1.5
debug_odom_jump_m: 0.5                 # double, 默认 0.5
debug_odom_jump_yaw_deg: 30.0          # double, 默认 30.0
pose_jump_reject_sec: 0.8              # double, 默认 0.8
debug_occ_ratio_jump: 0.15             # double, 默认 0.15

# ==================== 渲染 / 话题名 ====================
render_period_ms: 150                  # int, 默认 150
clear_history: false                   # bool, 默认 false
cloud_preview_topic: "/realtime_pgm/cloud" # std::string, 默认 "/realtime_pgm/cloud"
cloud_preview_max_points: 15000        # int, 默认 15000
history_image_topic: "/realtime_pgm/image_history" # std::string, 默认 "/realtime_pgm/image_history"
image_topic: "/realtime_pgm/image"     # std::string, 默认 "/realtime_pgm/image"

# ==================== 地图导出 / 导入 ====================
export_map_dir: "/tmp/pgm_export"      # std::string, 默认 "/tmp/pgm_export"
export_map_name: "map_export.zip"      # std::string, 默认 "map_export.zip"
export_map_request: false              # bool, 默认 false
import_map_dir: "/tmp/pgm_import"      # std::string, 默认 "/tmp/pgm_import"
import_map_request: false              # bool, 默认 false

# ==================== 动态障碍模式 ====================
dynamic_obstacle_enabled: false        # bool, 默认 false
dynamic_obstacle_topic: "/cloud_dynamic_obstacles" # std::string, 默认 "/cloud_dynamic_obstacles"
dynamic_obstacle_image_topic: "/realtime_pgm/image_dynamic" # std::string, 默认 "/realtime_pgm/image_dynamic"

# ==================== 位姿校正 (map->odom) ====================
pose_correction_topic: "/pose_correction" # std::string, 默认 "/pose_correction"
pose_correction_global_period_sec: 1.0 # double, 默认 1.0
pose_correction_global_range_m: 15.0   # double, 默认 15.0
pose_correction_local_range_m: 1.0     # double, 默认 1.0
pose_correction_score_min: 0.12        # double, 默认 0.12
pose_correction_max_delta_m: 0.3       # double, 默认 0.3
pose_correction_ema_alpha: 0.5         # double, 默认 0.5
pose_correction_score_gain: 0.05       # double, 默认 0.05
pose_correction_global_trigger_fails: 5 # int, 默认 5
apply_pose_correction: false           # bool, 默认 false

# ==================== 自过滤点云 (Nav2 clearing) ====================
self_filtered_topic: "/cloud_self_filtered" # std::string, 默认 "/cloud_self_filtered"
self_filtered_publish_rate: 10.0       # double, 默认 10.0
self_filtered_max_points: 6000         # int, 默认 6000

# ==================== 清除射线环 ====================
clear_ring_topic: "/cloud_clear_ring"  # std::string, 默认 "/cloud_clear_ring"
clear_ring_points: 72                  # int, 默认 72
clear_ring_radius: 1.2                 # double, 默认 1.2

# ==================== 清空 Nav2 代价图 ====================
clear_costmap_on_history_reset: true   # bool, 默认 true
clear_local_costmap_service: "/local_costmap/clear_entirely_local_costmap" # std::string, 默认 "/local_costmap/clear_entirely_local_costmap"
clear_global_costmap_service: "/global_costmap/clear_entirely_global_costmap" # std::string, 默认 "/global_costmap/clear_entirely_global_costmap"

# ==================== 运动帧 / 输入话题 ====================
motion_topic: "/nav_motion_cmd"        # std::string, 默认 "/nav_motion_cmd"
odom_topic: "/Odometry"                # std::string, 默认 "/Odometry"
global_plan_topic: "/plan"             # std::string, 默认 "/plan"
local_plan_topic: "/local_plan"        # std::string, 默认 "/local_plan"
local_plan_timeout: 0.5                # double, 默认 0.5
motion_control: 1                      # int, 默认 1
motion_vx_scale: kMotionSpeedScale     # double, 默认 kMotionSpeedScale
motion_vy_scale: kMotionSpeedScale     # double, 默认 kMotionSpeedScale
motion_rate_hz: 100.0                  # double, 默认 100.0

# ==================== 路径跟随 ====================
motion_target_speed: 1.00              # double, 默认 1.00
motion_lookahead: 0.45                 # double, 默认 0.45
motion_lookahead_speed_gain: 0.52      # double, 默认 0.52
motion_lookahead_max: 1.50             # double, 默认 1.50
motion_yaw_kp: 1.20                    # double, 默认 1.20
motion_goal_tolerance: 0.25            # double, 默认 0.25
motion_goal_check_tolerance: 0.35      # double, 默认 0.35
motion_goal_check_max_retry: 2         # int, 默认 2
motion_max_lateral: 0.0                # double, 默认 0.0
motion_plan_timeout: 1.0               # double, 默认 1.0
motion_slow_radius: 0.0                # double, 默认 0.0
motion_min_speed: 0.08                 # double, 默认 0.08
motion_base_to_lidar_x: -0.239         # double, 默认 -0.239
motion_base_to_lidar_y: 0.158          # double, 默认 0.158

# ==================== A* 兜底 ====================
astar_enabled: true                    # bool, 默认 true
astar_speed: -1.0                      # double, 默认 -1.0
astar_inflate_radius: 0.50             # double, 默认 0.50
astar_obstacle_penalty: 0.8            # double, 默认 0.8
astar_cost_reach: 0.60                 # double, 默认 0.60
astar_path_spacing: 0.15               # double, 默认 0.15
astar_replan_interval: 0.20            # double, 默认 0.20
astar_replan_move_dist: 0.05           # double, 默认 0.05
goal_pose_topic: "/nav_goal_pose"      # std::string, 默认 "/nav_goal_pose"
astar_plan_topic: "/astar_plan"        # std::string, 默认 "/astar_plan"

# ==================== 网页一次性触发 ====================
start_motion: false                    # bool, 默认 false
stop_motion: false                     # bool, 默认 false

# ==================== 占点 ====================
hold_point_enabled: false              # bool, 默认 false
hold_point_x: 0.0                      # double, 默认 0.0
hold_point_y: 0.0                      # double, 默认 0.0
hold_point_radius: 1.50                # double, 默认 1.50
hold_point_speed: -1.0                 # double, 默认 -1.0
hold_point_slow_radius: -1.0           # double, 默认 -1.0
hold_point_standoff_scale: 1.10        # double, 默认 1.10
hold_point_recenter_deadband: 0.08     # double, 默认 0.08
hold_point_status_topic: "/hold_point_status" # std::string, 默认 "/hold_point_status"
hold_point_viz_topic: "/hold_point_viz" # std::string, 默认 "/hold_point_viz"

# ==================== 追击 ====================
chase_enabled: false                   # bool, 默认 false
chase_pick_x: 0.0                      # double, 默认 0.0
chase_pick_y: 0.0                      # double, 默认 0.0
chase_radius: 1.0                      # double, 默认 1.0
chase_deadband: 0.15                   # double, 默认 0.15
chase_speed: -1.0                      # double, 默认 -1.0
chase_lock_radius: 1.0                 # double, 默认 1.0
chase_track_radius: 5.0                # double, 默认 5.0
chase_track_history_sec: 30.0          # double, 默认 30.0
chase_track_record_interval: 0.2       # double, 默认 0.2
chase_status_topic: "/chase_status"    # std::string, 默认 "/chase_status"
chase_viz_topic: "/chase_viz"          # std::string, 默认 "/chase_viz"
chase_track_topic: "/chase_track"      # std::string, 默认 "/chase_track"

# ==================== SocketCAN 直发 ====================
can_enabled: false                     # bool, 默认 false
can_interface: "can0"                  # std::string, 默认 "can0"
can_id: 0xFF                           # int, 默认 0xFF
can_send_period_ms: 10                 # int, 默认 10
```

## 附录：成员变量（20 组）去向

`config/` 只放 yaml，成员变量分三种去向：① 可调参数 → `params.yaml` 的键；② 纯数据结构 → `tools/types.hpp`；③ 运行期状态（mutex/队列/缓存/发布器）→ `io/` 或 `tasks/` 类的私有成员。逐组源码见 `05_node.md` 附录。

| 成员变量组 | 源码行 | 去向 |
|---|---|---|
| 点云基础 / 渲染周期 | 7570-7576 | tasks/cloud_pipeline.cpp（cloud_mutex_ / latest_cloud_）；话题名进 params.yaml |
| 调试日志 / 异常快照 / 跳变检测 | 7578-7611 | tasks/debug_log.cpp → DebugLogger（阈值键来自 params.yaml） |
| 参数与配置快照 | 7613-7615 | 值 → config/params.yaml；结构 → tools/types.hpp；锁与快照 → src/ 节点 |
| 订阅 / 定时器 / 单帧图发布 | 7617-7630 | 订阅与定时器 → src/ 节点；Image 发布器 → io/ros_image_pub |
| 历史图缓冲 | 7632-7638 | tasks/cloud_pipeline.cpp（history_ 队列） |
| 地图导出 / 导入 | 7640-7655 | tasks/map_io.cpp → MapIo（atomic 请求位）；落盘 → io/file_* |
| 动态障碍 / 位姿修正 | 7657-7704 | tasks/dynamic_split.cpp + tasks/pose_correction.cpp |
| 自过滤点云发布 | 7706-7715 | io/ros_cloud_pub.cpp → RosCloudPublisher |
| 清除射线 | 7717-7725 | io/ros_cloud_pub.cpp → RosCloudPublisher |
| 清空 Nav2 代价地图 | 7727-7735 | io/ros_nav2_client.cpp；开关键 `clear_costmap_on_history_reset` 进 params.yaml |
| 运动帧框架 / 输入话题 | 7737-7749 | 话题名与 scale → config/params.yaml；打包与下发 → io/ros_motion_pub.cpp |
| 路径跟随参数 | 7751-7775 | config/params.yaml（motion_* 14 项） |
| A* 兜底参数 | 7777-7788 | config/params.yaml（astar_* 8 项）+ tasks/astar_planner.cpp（缓存） |
| 占点 | 7790-7816 | config/params.yaml（hold_point_*）+ tasks/hold_point.cpp（状态） |
| 追击 | 7818-7846 | config/params.yaml（chase_*）+ tasks/chase.cpp（状态与轨迹） |
| 轨迹 + A* 缓存 + 目标点缓存 | 7847-7867 | tasks/chase.cpp（chase_track_）+ tasks/astar_planner.cpp（astar_cache_*） |
| 运动状态 / 里程计 / 路径缓冲 | 7869-7903 | tasks/motion_control.cpp（状态）+ src/callbacks.cpp（写入） |
| 运动请求 + 可视化快照 | 7905-7911 | tasks/motion_control.cpp（lookahead_view_ 的写入上移到此） |
| SocketCAN 直发 | 7913-7921 | io/can_bus.cpp → CanBus；`can_*` 键进 params.yaml |
| 参数回调句柄 | 7923-7924 | src/parameter_callback.cpp |
