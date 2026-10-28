# 按目标文件汇总（BY_FILE）

同一份拆分方案的"文件视角"：每个目标文件一节，先列该文件要装哪些符号，再给出**合并后的完整源码**（各段前有 `// ===== 源码行 a-b: 符号名 =====` 分隔注释，便于对照原文件）。

层级顺序：`config/`（只有 yaml） → `tools/`（纯计算） → `io/`（硬件与 ROS/文件交互） → `tasks/`（业务流程） → `src/`（装配与回调），依赖只允许自上而下。

> 注意：合并后的代码是**搬运视图**，不是可直接编译的完整文件——类方法的声明仍需在对应 `.hpp` 里按类组织，`private:` 成员、`#include`、`namespace` 包裹都要你在新建文件时补齐。成员变量的分配见文末附录。

对照文档：`01_config.md` / `02_tools.md` / `03_io.md` / `04_tasks.md` / `05_node.md`，索引见 `INDEX.md`。

## 文件总览

| 目标文件 | 建议类 / 命名空间 | 符号数 | 源码行合计 |
|---|---|---|---|
| `config/params.yaml` | 参数默认值（由 declare_parameter 提炼） | 1 | 487 |
| `config/nav2_params.yaml` | Nav2 参数（人工维护，无源码条目） | 0 | 0 |
| `tools/types.hpp` | namespace nav::tools | 7 | 68 |
| `tools/constants.hpp` | namespace nav::tools | 1 | 10 |
| `tools/tf_math.hpp` | namespace nav::tools (inline) | 6 | 99 |
| `tools/tf_math.cpp` | namespace nav::tools | 1 | 41 |
| `tools/registration.cpp` | namespace nav::tools | 1 | 37 |
| `tools/motion_codec.hpp` | namespace nav::tools | 4 | 37 |
| `tools/grid.cpp` | namespace nav::tools | 4 | 208 |
| `tools/grid_geom.cpp` | namespace nav::tools | 5 | 174 |
| `tools/grid_cluster.cpp` | namespace nav::tools | 2 | 123 |
| `tools/grid_clearance.cpp` | namespace nav::tools | 3 | 161 |
| `io/timed_cloud.hpp` | namespace nav::io（带 rclcpp::Time，故不放 tools） | 1 | 5 |
| `io/ros_pc_utils.hpp` | namespace nav::io（依赖 sensor_msgs，故不放 tools） | 1 | 11 |
| `io/can_bus.cpp` | class CanBus | 3 | 104 |
| `io/ros_cloud_pub.cpp` | class RosCloudPublisher | 5 | 261 |
| `io/ros_path_pub.cpp` | class RosPathPublisher | 6 | 207 |
| `io/ros_motion_pub.cpp` | class RosMotionPublisher | 1 | 43 |
| `io/ros_pose_pub.cpp` | class RosPosePublisher | 1 | 18 |
| `io/ros_stamp.cpp` | namespace nav::io | 1 | 26 |
| `io/ros_nav2_client.cpp` | class Nav2ServiceClient | 1 | 45 |
| `io/file_pcd.cpp` | namespace nav::io | 1 | 43 |
| `io/file_bin.cpp` | namespace nav::io | 2 | 135 |
| `io/file_yaml.cpp` | namespace nav::io | 1 | 37 |
| `io/debug_log_file.cpp` | class DebugLogFile | 1 | 41 |
| `tasks/cloud_pipeline.cpp` | class CloudPipeline | 2 | 257 |
| `tasks/dynamic_split.cpp` | class DynamicSplitter | 3 | 323 |
| `tasks/pose_correction.cpp` | class PoseCorrector | 5 | 341 |
| `tasks/map_render.cpp` | class MapRenderer | 3 | 315 |
| `tasks/map_io.cpp` | class MapIo | 3 | 232 |
| `tasks/astar_planner.cpp` | class AstarPlanner | 4 | 476 |
| `tasks/hold_point.cpp` | class HoldPointTask | 1 | 201 |
| `tasks/chase.cpp` | class ChaseTask | 2 | 297 |
| `tasks/motion_control.cpp` | class MotionController | 4 | 338 |
| `tasks/debug_log.cpp` | class DebugLogger | 1 | 126 |
| `src/main.cpp` | int main() | 1 | 11 |
| `src/realtime_pgm_node.cpp` | class RealtimePgmNode | 3 | 327 |
| `src/callbacks.cpp` | class RealtimePgmNode (回调段) | 6 | 708 |
| `src/parameter_callback.cpp` | class RealtimePgmNode (参数段) | 1 | 732 |

## `config/params.yaml`

建议落点：参数默认值（由 declare_parameter 提炼）

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `参数声明区 (constructor 118-604)` | 118-604 | 代码块 | 构造函数里的全部 declare_parameter: 参数名/类型/默认值的唯一来源 |

```cpp
// ===================== 源码行 118-604 · 参数声明区 (constructor 118-604) =====================
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

## `config/nav2_params.yaml`

建议落点：Nav2 参数（人工维护，无源码条目）

人工维护的参数文件，没有对应源码条目。

## `tools/types.hpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `Point` | 57-62 | struct | 点云基本单元 float x/y/z (被 tools 与 tasks 共用, 建议放最底层头文件) |
| `LookaheadView` | 70-82 | struct | 路径跟随可视化快照: valid/base_x/base_y/yaw/target_x/target_y/vx/vy/lookahead/dist_end |
| `Config` | 84-106 | struct | 建图配置: resolution/zmin/zmax/dilate/baseline_dilate/self_filter_size/地图范围/输出路径/历史时长/每帧点数上限/投票阈值 |
| `PoseCorrection` | 1233-1238 | struct | map->odom 刚体变换: double r[9] 行主序 + double t[3] + bool valid |
| `HoldObstacleCluster` | 4717-4721 | struct | 障碍簇世界系质心 x/y (占点与追击共用) |
| `FootprintBounds` | 6196-6203 | struct | 车体自过滤 AABB: valid / xmin / xmax / ymin / ymax |
| `ChaseTrackPoint` | 7848-7854 | struct | 追击目标轨迹条目: stamp / x / y / 该时刻簇点云 |

```cpp
// ===================== 源码行 57-62 · Point =====================
  struct Point
  {
    float x;
    float y;
    float z;
  };

// ===================== 源码行 70-82 · LookaheadView =====================
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

// ===================== 源码行 84-106 · Config =====================
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

// ===================== 源码行 1233-1238 · PoseCorrection =====================
  struct PoseCorrection
  {
    double r[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};   // 行主序
    double t[3] = {0.0, 0.0, 0.0};
    bool valid = false;
  };

// ===================== 源码行 4717-4721 · HoldObstacleCluster =====================
  struct HoldObstacleCluster
  {
    double x{0.0};
    double y{0.0};
  };

// ===================== 源码行 6196-6203 · FootprintBounds =====================
  struct FootprintBounds
  {
    bool valid{false};      // false: 无里程计或已关闭自过滤, 不做任何过滤
    double xmin{0.0};
    double xmax{0.0};
    double ymin{0.0};
    double ymax{0.0};
  };

// ===================== 源码行 7848-7854 · ChaseTrackPoint =====================
  struct ChaseTrackPoint
  {
    rclcpp::Time stamp;
    double x{0.0};
    double y{0.0};
    std::vector<Point> cloud;   // 记录该时刻目标簇的 3D 点云 (map 系)
  };

```

## `tools/constants.hpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `consts` | 46-55 | 代码块 | 运动帧协议常量: kMotionSpeedScale(量化 200) / kMotionMaxSpeed(±2 m/s) / kSlowRadiusTime(减速时间常数) / kMotionRawLimit(12 位饱和 2040) |

```cpp
// ===================== 源码行 46-55 · consts =====================
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

## `tools/tf_math.hpp`

建议落点：namespace nav::tools (inline)

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `quaternionToMatrix` | 1240-1265 | 函数 | 四元数 -> 行主序 3x3 旋转矩阵, 含归一化与退化时取单位阵 |
| `matrixToQuaternion` | 1267-1298 | 函数 | 3x3 旋转矩阵 -> 四元数 (trace 分支法) |
| `yawFromMatrix` | 1301-1304 | 函数 | 从行主序 3x3 取纯 z 旋转偏航角 atan2(r[3], r[0]) |
| `wrapAngle` | 1307-1318 | 函数 | 角度归一化到 (-pi, pi] |
| `multiplyMatrix` | 1321-1333 | 函数 | 3x3 行主序矩阵相乘 |
| `transformPointByCorrection` | 1398-1409 | 函数 | 单点刚体变换 p_map = R * p + t |

```cpp
// ===================== 源码行 1240-1265 · quaternionToMatrix =====================
  static void quaternionToMatrix(
    double qx, double qy, double qz, double qw, double r[9])
  {
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

    if (n < 1e-12) {
      const double identity[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      std::copy(identity, identity + 9, r);
      return;
    }

    const double x = qx / n;
    const double y = qy / n;
    const double z = qz / n;
    const double w = qw / n;

    r[0] = 1.0 - 2.0 * (y * y + z * z);
    r[1] = 2.0 * (x * y - w * z);
    r[2] = 2.0 * (x * z + w * y);
    r[3] = 2.0 * (x * y + w * z);
    r[4] = 1.0 - 2.0 * (x * x + z * z);
    r[5] = 2.0 * (y * z - w * x);
    r[6] = 2.0 * (x * z - w * y);
    r[7] = 2.0 * (y * z + w * x);
    r[8] = 1.0 - 2.0 * (x * x + y * y);
  }

// ===================== 源码行 1267-1298 · matrixToQuaternion =====================
  static void matrixToQuaternion(
    const double r[9],
    double & qx, double & qy, double & qz, double & qw)
  {
    const double trace = r[0] + r[4] + r[8];

    if (trace > 0.0) {
      const double s = std::sqrt(trace + 1.0) * 2.0;
      qw = 0.25 * s;
      qx = (r[7] - r[5]) / s;
      qy = (r[2] - r[6]) / s;
      qz = (r[3] - r[1]) / s;
    } else if (r[0] > r[4] && r[0] > r[8]) {
      const double s = std::sqrt(1.0 + r[0] - r[4] - r[8]) * 2.0;
      qw = (r[7] - r[5]) / s;
      qx = 0.25 * s;
      qy = (r[1] + r[3]) / s;
      qz = (r[2] + r[6]) / s;
    } else if (r[4] > r[8]) {
      const double s = std::sqrt(1.0 + r[4] - r[0] - r[8]) * 2.0;
      qw = (r[2] - r[6]) / s;
      qx = (r[1] + r[3]) / s;
      qy = 0.25 * s;
      qz = (r[5] + r[7]) / s;
    } else {
      const double s = std::sqrt(1.0 + r[8] - r[0] - r[4]) * 2.0;
      qw = (r[3] - r[1]) / s;
      qx = (r[2] + r[6]) / s;
      qy = (r[5] + r[7]) / s;
      qz = 0.25 * s;
    }
  }

// ===================== 源码行 1301-1304 · yawFromMatrix =====================
  static double yawFromMatrix(const double r[9])
  {
    return std::atan2(r[3], r[0]);
  }

// ===================== 源码行 1307-1318 · wrapAngle =====================
  static double wrapAngle(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }

    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }

    return a;
  }

// ===================== 源码行 1321-1333 · multiplyMatrix =====================
  static void multiplyMatrix(
    const double a[9], const double b[9], double out[9])
  {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) {
          sum += a[i * 3 + k] * b[k * 3 + j];
        }
        out[i * 3 + j] = sum;
      }
    }
  }

// ===================== 源码行 1398-1409 · transformPointByCorrection =====================
  void transformPointByCorrection(Point & point, const PoseCorrection & c) const
  {
    const double x = point.x;
    const double y = point.y;
    const double z = point.z;
    point.x = static_cast<float>(
      c.r[0] * x + c.r[1] * y + c.r[2] * z + c.t[0]);
    point.y = static_cast<float>(
      c.r[3] * x + c.r[4] * y + c.r[5] * z + c.t[1]);
    point.z = static_cast<float>(
      c.r[6] * x + c.r[7] * y + c.r[8] * z + c.t[2]);
  }

```

## `tools/tf_math.cpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `transformToBaseLink` | 2454-2494 | 函数 | map -> base_link: 先用 odom + 雷达外参反推车体中心, 再 R(-yaw)*(p-base) |

```cpp
// ===================== 源码行 2454-2494 · transformToBaseLink =====================
  bool transformToBaseLink(
    const std::vector<Point> & map_points,
    std::vector<Point> & base_points)
  {
    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_yaw = 0.0;
    double base_to_lidar_x = 0.0;
    double base_to_lidar_y = 0.0;
    bool odom_ok = false;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      odom_ok = odom_received_;
      odom_x = odom_x_;
      odom_y = odom_y_;
      odom_yaw = odom_yaw_;
      base_to_lidar_x = motion_base_to_lidar_x_;
      base_to_lidar_y = motion_base_to_lidar_y_;
    }

    if (!odom_ok) {
      return false;
    }

    const double cos_yaw = std::cos(odom_yaw);
    const double sin_yaw = std::sin(odom_yaw);
    const double base_x = odom_x -
      (cos_yaw * base_to_lidar_x - sin_yaw * base_to_lidar_y);
    const double base_y = odom_y -
      (sin_yaw * base_to_lidar_x + cos_yaw * base_to_lidar_y);

    base_points.resize(map_points.size());
    for (std::size_t i = 0; i < map_points.size(); ++i) {
      const double dx = map_points[i].x - base_x;
      const double dy = map_points[i].y - base_y;
      base_points[i].x =  cos_yaw * dx + sin_yaw * dy;   // R(-yaw) * (p - base)
      base_points[i].y = -sin_yaw * dx + cos_yaw * dy;
      base_points[i].z = map_points[i].z;
    }
    return true;
  }

```

## `tools/registration.cpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `score_at` | 1469-1505 | 函数 | 位姿命中率评分 (estimatePoseCorrection 内部的 lambda): 采样点变换后查栅格, 出界计入分母 |

```cpp
// ===================== 源码行 1469-1505 · score_at =====================
    auto score_at = [&](double x, double y, double yaw, double & out_score) {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        std::size_t hits = 0;
        std::size_t considered = 0;

        for (const auto & point : samples) {
          const double tx = c * point.x - s * point.y + x;
          const double ty = s * point.x + c * point.y + y;
          ++considered;
          const int ix = static_cast<int>(
            std::floor((tx - config.xmin) / config.resolution));
          const int iy = static_cast<int>(
            std::floor((ty - config.ymin) / config.resolution));

          if (ix < 0 || ix >= width_i || iy < 0 || iy >= height_i) {
            continue;
          }

          const std::size_t index =
            static_cast<std::size_t>(iy) *
            static_cast<std::size_t>(width_i) +
            static_cast<std::size_t>(ix);

          if (score_mask[index] != 0) {
            ++hits;
          }
        }

        if (considered < 40) {
          return false;
        }

        out_score = static_cast<double>(hits) /
          static_cast<double>(considered);
        return true;
      };

```

## `tools/motion_codec.hpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `limitMotionSpeed` | 3590-3601 | 函数 | 速度保方向等比限幅: 峰值超 kMotionMaxSpeed 时两轴同比缩放 |
| `effectiveSlowRadius` | 3606-3615 | 函数 | 减速起始距离: 手动值优先, 否则 max(阈值+0.05, kSlowRadiusTime*目标速度) |
| `encodeMotionRaw` | 3619-3628 | 函数 | m/s -> 12 位有符号原始量: 就近量化 + ±2040 饱和 + &0x0FFF |
| `resolveAstarSpeed` | 5808-5812 | 函数 | A* 兜底速度解析: <=0 时回落到 motion_target_speed_, 并夹到 kMotionMaxSpeed |

```cpp
// ===================== 源码行 3590-3601 · limitMotionSpeed =====================
  static void limitMotionSpeed(double & vx, double & vy)
  {
    const double peak = std::max(std::fabs(vx), std::fabs(vy));

    if (peak <= kMotionMaxSpeed || peak <= 0.0) {
      return;
    }

    const double ratio = kMotionMaxSpeed / peak;
    vx *= ratio;
    vy *= ratio;
  }

// ===================== 源码行 3606-3615 · effectiveSlowRadius =====================
  static double effectiveSlowRadius(
    double slow_radius, double target_speed, double goal_tolerance)
  {
    if (slow_radius > 0.0) {
      return slow_radius;
    }

    const double speed = std::clamp(target_speed, 0.0, 5.0);
    return std::max(goal_tolerance + 0.05, kSlowRadiusTime * speed);
  }

// ===================== 源码行 3619-3628 · encodeMotionRaw =====================
  static uint16_t encodeMotionRaw(double value, double scale)
  {
    const double s = (scale > 0.0) ? scale : kMotionSpeedScale;
    const double step = 1.0 / s;
    // 就近量化, 保证静止(0 m/s)时原始量严格为 0
    const long scaled = std::lround(value / step);
    // 兜底饱和: 防止 scale 参数被改小后原始量溢出 12 位
    const long raw = std::clamp(scaled, -kMotionRawLimit, kMotionRawLimit);
    return static_cast<uint16_t>(static_cast<int16_t>(raw) & 0x0FFF);
  }

// ===================== 源码行 5808-5812 · resolveAstarSpeed =====================
  double resolveAstarSpeed() const
  {
    const double base = (astar_speed_ > 0.0) ? astar_speed_ : motion_target_speed_;
    return std::clamp(base, 0.0, kMotionMaxSpeed);
  }

```

## `tools/grid.cpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `clearVehicleFootprint` | 6337-6390 | 函数 | 栅格级清除: 车体方框内占据格一律置空 |
| `accumulateCloud` | 6395-6459 | 函数 | 单帧点云投影入占据栅格 (z 过滤), 支持"同帧只计一次"的命中计数模式 |
| `applyDilate` | 6462-6519 | 函数 | 占据栅格圆形(方形核)膨胀指定格数 |
| `occupancyToPixels` | 6522-6552 | 函数 | 占据栅格 -> mono8 像素 (0=占据, 205=未知), 行序按 PGM 约定翻转 |

```cpp
// ===================== 源码行 6337-6390 · clearVehicleFootprint =====================
  void clearVehicleFootprint(
    std::vector<uint8_t> & occupancy,
    const Config & config,
    int width,
    int height)
  {
    const FootprintBounds bounds = computeVehicleFootprint();

    if (!bounds.valid) {
      return;
    }

    const double footprint_size = bounds.xmax - bounds.xmin;

    const int ix0 = static_cast<int>(
      std::floor((bounds.xmin - config.xmin) / config.resolution));
    const int ix1 = static_cast<int>(
      std::floor((bounds.xmax - config.xmin) / config.resolution));
    const int iy0 = static_cast<int>(
      std::floor((bounds.ymin - config.ymin) / config.resolution));
    const int iy1 = static_cast<int>(
      std::floor((bounds.ymax - config.ymin) / config.resolution));

    const int cx0 = std::max(0, std::min(ix0, ix1));
    const int cx1 = std::min(width - 1, std::max(ix0, ix1));
    const int cy0 = std::max(0, std::min(iy0, iy1));
    const int cy1 = std::min(height - 1, std::max(iy0, iy1));

    std::size_t cleared = 0;

    for (int iy = cy0; iy <= cy1; ++iy) {
      for (int ix = cx0; ix <= cx1; ++ix) {
        const std::size_t index =
          static_cast<std::size_t>(iy) *
          static_cast<std::size_t>(width) +
          static_cast<std::size_t>(ix);

        if (occupancy[index] != 0) {
          occupancy[index] = 0;
          ++cleared;
        }
      }
    }

    if (cleared > 0) {
      RCLCPP_DEBUG(
        get_logger(),
        "车体自过滤: 车体中心 (%.2f, %.2f) %.2f m 见方内清除 %zu 个占据格",
        (bounds.xmin + bounds.xmax) * 0.5,
        (bounds.ymin + bounds.ymax) * 0.5,
        footprint_size,
        cleared);
    }
  }

// ===================== 源码行 6395-6459 · accumulateCloud =====================
  void accumulateCloud(
    const std::vector<Point> & cloud,
    const Config & config,
    int width,
    int height,
    std::vector<uint8_t> & occupancy,
    std::size_t & filtered_points,
    std::vector<uint8_t> * hit_counts = nullptr)
  {
    std::vector<std::size_t> frame_cells;
    if (hit_counts != nullptr) {
      frame_cells.reserve(cloud.size());
    }

    for (const auto & point : cloud) {
      if (point.z < config.zmin ||
        point.z > config.zmax)
      {
        continue;
      }

      const int ix = static_cast<int>(
        std::floor(
          (static_cast<double>(point.x) - config.xmin) /
          config.resolution));

      const int iy = static_cast<int>(
        std::floor(
          (static_cast<double>(point.y) - config.ymin) /
          config.resolution));

      if (ix < 0 || ix >= width ||
        iy < 0 || iy >= height)
      {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(iy) *
        static_cast<std::size_t>(width) +
        static_cast<std::size_t>(ix);

      ++filtered_points;

      if (hit_counts == nullptr) {
        occupancy[index] = 1;
        continue;
      }

      if (occupancy[index] == 0) {
        occupancy[index] = 1;
        frame_cells.push_back(index);
      }
    }

    if (hit_counts != nullptr) {
      for (const std::size_t index : frame_cells) {
        occupancy[index] = 0;

        if ((*hit_counts)[index] != 255U) {
          ++(*hit_counts)[index];
        }
      }
    }
  }

// ===================== 源码行 6462-6519 · applyDilate =====================
  void applyDilate(
    const Config & config,
    int width,
    int height,
    std::vector<uint8_t> & occupancy,
    std::size_t filtered_points)
  {
    if (config.dilate <= 0) {
      return;
    }

    const int radius = config.dilate;
    const int radius_squared = radius * radius;

    std::vector<std::pair<int, int>> occupied_cells;
    // 统计滤波后真实占据格通常远少于累计命中点数, 预留量按栅格总数封顶
    occupied_cells.reserve(
      std::min<std::size_t>(
        filtered_points,
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height)));

    for (int iy = 0; iy < height; ++iy) {
      for (int ix = 0; ix < width; ++ix) {
        const std::size_t index =
          static_cast<std::size_t>(iy) *
          static_cast<std::size_t>(width) +
          static_cast<std::size_t>(ix);

        if (occupancy[index] != 0) {
          occupied_cells.emplace_back(ix, iy);
        }
      }
    }

    for (const auto & cell : occupied_cells) {
      const int sx = cell.first;
      const int sy = cell.second;

      const int x0 = std::max(0, sx - radius);
      const int x1 = std::min(width - 1, sx + radius);
      const int y0 = std::max(0, sy - radius);
      const int y1 = std::min(height - 1, sy + radius);

      for (int iy = y0; iy <= y1; ++iy) {
        for (int ix = x0; ix <= x1; ++ix) {
          const int dx = ix - sx;
          const int dy = iy - sy;

          if (dx * dx + dy * dy <= radius_squared) {
            occupancy[
              static_cast<std::size_t>(iy) *
              static_cast<std::size_t>(width) +
              static_cast<std::size_t>(ix)] = 1;
          }
        }
      }
    }
  }

// ===================== 源码行 6522-6552 · occupancyToPixels =====================
  void occupancyToPixels(
    const std::vector<uint8_t> & occupancy,
    int width,
    int height,
    std::vector<uint8_t> & pixels)
  {
    const std::size_t pixel_count =
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(height);

    pixels.assign(pixel_count, 205);

    for (int iy = 0; iy < height; ++iy) {
      const int image_row = height - 1 - iy;

      for (int ix = 0; ix < width; ++ix) {
        const std::size_t source_index =
          static_cast<std::size_t>(iy) *
          static_cast<std::size_t>(width) +
          static_cast<std::size_t>(ix);

        const std::size_t target_index =
          static_cast<std::size_t>(image_row) *
          static_cast<std::size_t>(width) +
          static_cast<std::size_t>(ix);

        pixels[target_index] =
          occupancy[source_index] != 0 ? 0 : 205;
      }
    }
  }

```

## `tools/grid_geom.cpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `computeMapSize` | 6153-6191 | 函数 | 由 Config 的地图范围/分辨率算 width/height, 并校验像素总数上限 |
| `computeVehicleFootprint` | 6210-6253 | 函数 | 由里程计位姿 + self_filter_size 算出世界系轴对齐车体方框 FootprintBounds |
| `clipCloudToMapBounds` | 6262-6294 | 函数 | 按半开区间 [xmin,xmax) x [ymin,ymax) 就地裁剪点云, 返回丢弃点数 |
| `filterVehicleFootprintPoints` | 6301-6327 | 函数 | 点级自过滤: 丢弃落在车体方框(闭区间)内的本体点 |
| `computeHoldBlockPose` | 4825-4855 | 函数 | 沿"障碍 -> 中心"连线取距障碍 standoff 的阻挡站位点 |

```cpp
// ===================== 源码行 6153-6191 · computeMapSize =====================
  bool computeMapSize(const Config & config, int & width, int & height)
  {
    const double width_f =
      (config.xmax - config.xmin) / config.resolution;
    const double height_f =
      (config.ymax - config.ymin) / config.resolution;

    width = static_cast<int>(std::ceil(width_f));
    height = static_cast<int>(std::ceil(height_f));

    if (width <= 0 || height <= 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "PGM 尺寸无效: %d x %d",
        width,
        height);
      return false;
    }

    const std::size_t pixel_count =
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(height);

    constexpr std::size_t max_pixels = 200000000ULL;

    if (pixel_count > max_pixels) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "PGM 像素数量过大: %zu，请增大 resolution 或缩小地图范围",
        pixel_count);
      return false;
    }

    return true;
  }

// ===================== 源码行 6210-6253 · computeVehicleFootprint =====================
  FootprintBounds computeVehicleFootprint()
  {
    FootprintBounds bounds;

    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_yaw = 0.0;
    bool odom_ok = false;
    double footprint_size = 0.0;

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      odom_ok = odom_received_;
      odom_x = odom_x_;
      odom_y = odom_y_;
      odom_yaw = odom_yaw_;
    }

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      footprint_size = config_.self_filter_size;
    }

    if (!odom_ok || footprint_size <= 0.0) {
      return bounds;   // 无里程计 / <=0 表示关闭自过滤
    }

    const double cos_yaw = std::cos(odom_yaw);
    const double sin_yaw = std::sin(odom_yaw);
    const double base_x = odom_x -
      (cos_yaw * motion_base_to_lidar_x_ - sin_yaw * motion_base_to_lidar_y_);
    const double base_y = odom_y -
      (sin_yaw * motion_base_to_lidar_x_ + cos_yaw * motion_base_to_lidar_y_);

    // 车体为全向底盘, 车头朝向不影响车身方形包络, 故用世界系轴对齐正方形
    const double half = footprint_size * 0.5;

    bounds.valid = true;
    bounds.xmin = base_x - half;
    bounds.xmax = base_x + half;
    bounds.ymin = base_y - half;
    bounds.ymax = base_y + half;
    return bounds;
  }

// ===================== 源码行 6262-6294 · clipCloudToMapBounds =====================
  std::size_t clipCloudToMapBounds(std::vector<Point> & cloud)
  {
    double xmin = 0.0;
    double xmax = 0.0;
    double ymin = 0.0;
    double ymax = 0.0;

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      xmin = config_.xmin;
      xmax = config_.xmax;
      ymin = config_.ymin;
      ymax = config_.ymax;
    }

    std::size_t write = 0;

    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const Point & point = cloud[i];

      const bool inside =
        point.x >= xmin && point.x < xmax &&
        point.y >= ymin && point.y < ymax;

      if (inside) {
        cloud[write++] = point;
      }
    }

    const std::size_t removed = cloud.size() - write;
    cloud.resize(write);
    return removed;
  }

// ===================== 源码行 6301-6327 · filterVehicleFootprintPoints =====================
  std::size_t filterVehicleFootprintPoints(
    std::vector<Point> & cloud)
  {
    const FootprintBounds bounds = computeVehicleFootprint();

    if (!bounds.valid) {
      return 0;   // 无里程计/关闭自过滤: 不过滤, 行为与改动前一致
    }

    std::size_t write = 0;

    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const Point & point = cloud[i];

      const bool inside =
        point.x >= bounds.xmin && point.x <= bounds.xmax &&
        point.y >= bounds.ymin && point.y <= bounds.ymax;

      if (!inside) {
        cloud[write++] = point;
      }
    }

    const std::size_t removed = cloud.size() - write;
    cloud.resize(write);
    return removed;
  }

// ===================== 源码行 4825-4855 · computeHoldBlockPose =====================
  void computeHoldBlockPose(
    double obs_x, double obs_y, double center_x, double center_y,
    double & block_x, double & block_y) const
  {
    double standoff = astar_inflate_radius_;

    {
      std::lock_guard<std::mutex> lock(hold_point_mutex_);
      standoff = hold_point_standoff_scale_ * astar_inflate_radius_;
    }

    const double dx = center_x - obs_x;
    const double dy = center_y - obs_y;
    const double dist = std::hypot(dx, dy);

    if (dist <= 1e-6) {
      // 障碍与中心重合 (异常): 站位点退化为障碍所在处外侧
      block_x = obs_x + standoff;
      block_y = obs_y;
      return;
    }

    // 距障碍 standoff 处、朝中心方向
    const double ux = dx / dist;
    const double uy = dy / dist;
    // 若障碍离中心比站位间距还近, 则退到障碍"外侧" (背离中心) 避免与障碍重叠
    const double t = (dist > standoff) ? standoff : -standoff;

    block_x = obs_x + ux * t;
    block_y = obs_y + uy * t;
  }

```

## `tools/grid_cluster.cpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `extractHoldObstacles` | 4726-4818 | 函数 | 按固定步长扫描占据栅格, 在线聚类(加权桶)求障碍簇质心, 只保留范围内且间距达标的簇 |
| `findNearestObstacleCluster` | 5318-5347 | 函数 | 在 (ref_x,ref_y) 为圆心、radius 为半径内取最近障碍簇质心 |

```cpp
// ===================== 源码行 4726-4818 · extractHoldObstacles =====================
  std::vector<HoldObstacleCluster> extractHoldObstacles(
    const std::vector<uint8_t> & occupancy,
    const Config & config,
    int width,
    int height,
    double center_x,
    double center_y,
    double radius)
  {
    std::vector<HoldObstacleCluster> clusters;

    if (occupancy.empty() || width <= 0 || height <= 0 ||
      config.resolution <= 0.0)
    {
      return clusters;
    }

    // 只关心"占点范围 + 硬性间距"内的障碍: 更远的障碍即使朝中心来也还有
    //   足够距离, 此刻不需要拦截, 避免无谓机动。
    const double reach = radius + std::max(0.0, astar_inflate_radius_);
    const double reach_sq = reach * reach;

    // 粗聚类: 按固定格步长扫描, 把相邻的占据格归并到同一个簇 (取质心)。
    //   采用"网格桶 + 桶内合并"的轻量做法, 复杂度 O(格数), 满足 100 Hz 线程。
    constexpr int kClusterStep = 2;   // 每隔多少格取一个样本点
    struct Bucket
    {
      double sx{0.0};
      double sy{0.0};
      double sw{0.0};
    };

    std::vector<Bucket> buckets;
    buckets.reserve(256);

    for (int iy = 0; iy < height; iy += kClusterStep) {
      for (int ix = 0; ix < width; ix += kClusterStep) {
        if (occupancy[static_cast<std::size_t>(iy) *
          static_cast<std::size_t>(width) +
          static_cast<std::size_t>(ix)] == 0)
        {
          continue;
        }

        const double wx = config.xmin + (ix + 0.5) * config.resolution;
        const double wy = config.ymin + (iy + 0.5) * config.resolution;
        const double dx = wx - center_x;
        const double dy = wy - center_y;

        if (dx * dx + dy * dy > reach_sq) {
          continue;
        }

        // 归并到最近的已有桶 (距离小于一个聚类半径即视为同一障碍)
        const double merge_r = std::max(config.resolution * kClusterStep * 1.5, 0.15);
        const double merge_r_sq = merge_r * merge_r;
        bool merged = false;

        for (auto & bucket : buckets) {
          const double cx = bucket.sx / bucket.sw;
          const double cy = bucket.sy / bucket.sw;
          const double ddx = wx - cx;
          const double ddy = wy - cy;

          if (ddx * ddx + ddy * ddy <= merge_r_sq) {
            bucket.sx += wx;
            bucket.sy += wy;
            bucket.sw += 1.0;
            merged = true;
            break;
          }
        }

        if (!merged) {
          buckets.push_back(Bucket{wx, wy, 1.0});
        }
      }
    }

    clusters.reserve(buckets.size());

    for (const auto & bucket : buckets) {
      if (bucket.sw <= 0.0) {
        continue;
      }
      HoldObstacleCluster cluster;
      cluster.x = bucket.sx / bucket.sw;
      cluster.y = bucket.sy / bucket.sw;
      clusters.push_back(cluster);
    }

    return clusters;
  }

// ===================== 源码行 5318-5347 · findNearestObstacleCluster =====================
  bool findNearestObstacleCluster(
    const std::vector<uint8_t> & occupancy,
    const Config & config,
    int width,
    int height,
    double ref_x,
    double ref_y,
    double radius,
    double & out_x,
    double & out_y)
  {
    std::vector<HoldObstacleCluster> clusters =
      extractHoldObstacles(occupancy, config, width, height, ref_x, ref_y, radius);

    bool found = false;
    double best = std::numeric_limits<double>::max();

    for (const auto & cluster : clusters) {
      const double d = std::hypot(cluster.x - ref_x, cluster.y - ref_y);

      if (d <= radius && d < best) {
        best = d;
        out_x = cluster.x;
        out_y = cluster.y;
        found = true;
      }
    }

    return found;
  }

```

## `tools/grid_clearance.cpp`

建议落点：namespace nav::tools

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `isHoldPointClear` | 4863-4928 | 函数 | 点级硬性间距自检: 圆盘邻域内不得有未豁免的占据格 |
| `isHoldPointClearPath` | 4934-5012 | 函数 | 线段级硬性间距自检: 等间隔采样(上限 4000 点)后逐点判定 |
| `chaseLineClear` | 5352-5367 | 函数 | 车体 -> 目标连线是否通畅(豁免目标本体), 决定直连还是 A* 绕行 |

```cpp
// ===================== 源码行 4863-4928 · isHoldPointClear =====================
  bool isHoldPointClear(
    const std::vector<uint8_t> & occupancy,
    const Config & config,
    int width,
    int height,
    double x, double y,
    double safe_dist,
    double obs_x, double obs_y, double exempt_radius) const
  {
    if (occupancy.empty() || width <= 0 || height <= 0 ||
      config.resolution <= 0.0)
    {
      return false;
    }

    const int cx = static_cast<int>(
      std::floor((x - config.xmin) / config.resolution));
    const int cy = static_cast<int>(
      std::floor((y - config.ymin) / config.resolution));

    if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
      return false;
    }

    const int radius_cells = static_cast<int>(
      std::ceil(std::max(0.0, safe_dist) / config.resolution));
    const double exempt_sq = exempt_radius * exempt_radius;

    const int x0c = std::max(0, cx - radius_cells);
    const int x1c = std::min(width - 1, cx + radius_cells);
    const int y0c = std::max(0, cy - radius_cells);
    const int y1c = std::min(height - 1, cy + radius_cells);
    const int r_sq = radius_cells * radius_cells;

    for (int jy = y0c; jy <= y1c; ++jy) {
      for (int jx = x0c; jx <= x1c; ++jx) {
        const int ddx = jx - cx;
        const int ddy = jy - cy;

        if (ddx * ddx + ddy * ddy > r_sq) {
          continue;
        }

        if (occupancy[static_cast<std::size_t>(jy) *
          static_cast<std::size_t>(width) +
          static_cast<std::size_t>(jx)] == 0)
        {
          continue;
        }

        // 该占据格若属于被豁免的目标敌人 (落在障碍质心 exempt_radius 内), 跳过
        const double wx = config.xmin + (jx + 0.5) * config.resolution;
        const double wy = config.ymin + (jy + 0.5) * config.resolution;
        const double odx = wx - obs_x;
        const double ody = wy - obs_y;

        if (exempt_radius > 0.0 && odx * odx + ody * ody <= exempt_sq) {
          continue;
        }

        return false;
      }
    }

    return true;
  }

// ===================== 源码行 4934-5012 · isHoldPointClearPath =====================
  bool isHoldPointClearPath(
    const std::vector<uint8_t> & occupancy,
    const Config & config,
    int width,
    int height,
    double x0, double y0, double x1, double y1,
    double safe_dist,
    double obs_x, double obs_y, double exempt_radius) const
  {
    if (occupancy.empty() || width <= 0 || height <= 0 ||
      config.resolution <= 0.0)
    {
      return false;
    }

    const double length = std::hypot(x1 - x0, y1 - y0);
    const double step = std::max(config.resolution,
      std::max(0.0, safe_dist) * 0.5);
    const int samples = static_cast<int>(std::ceil(length / step)) + 1;
    const int max_samples = 4000;
    const int radius_cells = static_cast<int>(
      std::ceil(std::max(0.0, safe_dist) / config.resolution));
    const double exempt_sq = exempt_radius * exempt_radius;

    for (int s = 0; s < std::min(samples, max_samples); ++s) {
      const double t = (samples <= 1) ? 0.0 :
        static_cast<double>(s) / static_cast<double>(samples - 1);
      const double wx = x0 + (x1 - x0) * t;
      const double wy = y0 + (y1 - y0) * t;

      const int cx = static_cast<int>(
        std::floor((wx - config.xmin) / config.resolution));
      const int cy = static_cast<int>(
        std::floor((wy - config.ymin) / config.resolution));

      if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
        return false;
      }

      const int x0c = std::max(0, cx - radius_cells);
      const int x1c = std::min(width - 1, cx + radius_cells);
      const int y0c = std::max(0, cy - radius_cells);
      const int y1c = std::min(height - 1, cy + radius_cells);
      const int r_sq = radius_cells * radius_cells;

      for (int jy = y0c; jy <= y1c; ++jy) {
        for (int jx = x0c; jx <= x1c; ++jx) {
          const int ddx = jx - cx;
          const int ddy = jy - cy;

          if (ddx * ddx + ddy * ddy > r_sq) {
            continue;
          }

          if (occupancy[static_cast<std::size_t>(jy) *
            static_cast<std::size_t>(width) +
            static_cast<std::size_t>(jx)] == 0)
          {
            continue;
          }

          if (exempt_radius > 0.0) {
            const double ox = config.xmin + (jx + 0.5) * config.resolution;
            const double oy = config.ymin + (jy + 0.5) * config.resolution;
            const double odx = ox - obs_x;
            const double ody = oy - obs_y;

            if (odx * odx + ody * ody <= exempt_sq) {
              continue;
            }
          }

          return false;
        }
      }
    }

    return true;
  }

// ===================== 源码行 5352-5367 · chaseLineClear =====================
  bool chaseLineClear(
    const std::vector<uint8_t> & occupancy,
    const Config & config,
    int width,
    int height,
    double x0, double y0, double x1, double y1,
    double target_x, double target_y)
  {
    const double exempt_radius =
      astar_inflate_radius_ + config.resolution * 1.5;

    return isHoldPointClearPath(
      occupancy, config, width, height,
      x0, y0, x1, y1, astar_inflate_radius_,
      target_x, target_y, exempt_radius);
  }

```

## `io/timed_cloud.hpp`

建议落点：namespace nav::io（带 rclcpp::Time，故不放 tools）

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `TimedCloud` | 109-113 | struct | 带时间戳的点云缓存(滑动窗元素): rclcpp::Time stamp + shared_ptr<const vector<Point>> |

```cpp
// ===================== 源码行 109-113 · TimedCloud =====================
  struct TimedCloud
  {
    rclcpp::Time stamp;
    std::shared_ptr<const std::vector<Point>> points;
  };

```

## `io/ros_pc_utils.hpp`

建议落点：namespace nav::io（依赖 sensor_msgs，故不放 tools）

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `hasField` | 1217-1227 | 函数 | 检查 PointCloud2 是否含指定字段名 (static, 无成员访问) |

```cpp
// ===================== 源码行 1217-1227 · hasField =====================
  static bool hasField(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & field_name)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == field_name) {
        return true;
      }
    }
    return false;
  }

```

## `io/can_bus.cpp`

建议落点：class CanBus

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `openCanSocket` | 3487-3536 | 函数 | socket / ioctl(SIOCGIFINDEX) / bind 建 SocketCAN 链路, 失败打错并返回 false (下次重试) |
| `closeCanSocket` | 3478-3484 | 函数 | 关闭 CAN fd 并置 -1 |
| `canFrameFromMotion` | 3540-3586 | 函数 | 按 can_send_period_ms_ 节流组 can_frame 直写总线; 失败关 socket 待重连, 成功累加计数 |

```cpp
// ===================== 源码行 3487-3536 · openCanSocket =====================
  bool openCanSocket()
  {
    closeCanSocket();

    can_socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (can_socket_fd_ < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "创建 CAN socket 失败: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (::ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "找不到 CAN 接口 %s: %s (请先执行 ip link set %s up)",
        can_interface_.c_str(), std::strerror(errno),
        can_interface_.c_str());
      closeCanSocket();
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(
        can_socket_fd_, reinterpret_cast<struct sockaddr *>(&addr),
        sizeof(addr)) < 0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "绑定 CAN 接口 %s 失败: %s",
        can_interface_.c_str(), std::strerror(errno));
      closeCanSocket();
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "SocketCAN 已就绪: %s, 帧 ID=0x%03X, 直发周期 %d ms",
      can_interface_.c_str(), can_id_, can_send_period_ms_);
    return true;
  }

// ===================== 源码行 3478-3484 · closeCanSocket =====================
  void closeCanSocket()
  {
    if (can_socket_fd_ >= 0) {
      ::close(can_socket_fd_);
      can_socket_fd_ = -1;
    }
  }

// ===================== 源码行 3540-3586 · canFrameFromMotion =====================
  void canFrameFromMotion(const uint8_t * data)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(
      std::max(1, can_send_period_ms_));

    if (can_last_send_.time_since_epoch().count() != 0 &&
      now - can_last_send_ < period)
    {
      return;
    }
    can_last_send_ = now;

    if (can_socket_fd_ < 0) {
      if (!openCanSocket()) {
        return;
      }
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id_;
    frame.can_dlc = 8;
    for (std::size_t i = 0; i < 8; ++i) {
      frame.data[i] = data[i];
    }

    const ssize_t nbytes = ::write(can_socket_fd_, &frame, sizeof(frame));
    if (nbytes < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "CAN 发送失败 (ID=0x%03X): %s (接口可能 DOWN, 将自动重连)",
        can_id_, std::strerror(errno));
      closeCanSocket();
      return;
    }

    ++can_sent_frames_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "CAN 直发 %s ID=0x%03X [8] %02X %02X %02X %02X %02X %02X %02X %02X "
      "(累计 %llu 帧)",
      can_interface_.c_str(), can_id_,
      frame.data[0], frame.data[1], frame.data[2], frame.data[3],
      frame.data[4], frame.data[5], frame.data[6], frame.data[7],
      static_cast<unsigned long long>(can_sent_frames_.load()));
  }

```

## `io/ros_cloud_pub.cpp`

建议落点：class RosCloudPublisher

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `publishCloudPreview` | 2323-2369 | 函数 | 按 cloud_preview_max_points_ 等步长抽稀后发布预览 PointCloud2; 无订阅者直接返回 |
| `publishPointCloud` | 2412-2445 | 函数 | 把点列打包成 PointCloud2 (frame_id 入参) 并发布 |
| `publishSelfFilteredCloud` | 2510-2578 | 函数 | 限频 + 抽稀 + 转 base_link 后发布 (Nav2 obstacle_layer 的 clearing 源) |
| `publishDynamicObstacleCloud` | 7346-7418 | 函数 | 组避障点云(整帧 + 基准栅格取点) 发 Nav2 |
| `publishClearRingCloud` | 7432-7469 | 函数 | 生成 base_link 系环点并发布, 让代价图沿射线清除旧红色区域 |

```cpp
// ===================== 源码行 2323-2369 · publishCloudPreview =====================
  void publishCloudPreview(const std::vector<Point> & points)
  {
    if (!cloud_preview_publisher_ ||
      cloud_preview_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    const std::size_t total = points.size();

    if (total == 0) {
      return;
    }

    const std::size_t max_points =
      static_cast<std::size_t>(cloud_preview_max_points_);

    const std::size_t stride =
      (total + max_points - 1) / max_points;

    const std::size_t count =
      (total + stride - 1) / stride;

    sensor_msgs::msg::PointCloud2 preview_msg;
    preview_msg.header.stamp = get_clock()->now();
    preview_msg.header.frame_id = "map";
    preview_msg.height = 1;

    sensor_msgs::PointCloud2Modifier modifier(preview_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(preview_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(preview_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(preview_msg, "z");

    for (std::size_t i = 0; i < total; i += stride) {
      *iter_x = points[i].x;
      *iter_y = points[i].y;
      *iter_z = points[i].z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud_preview_publisher_->publish(preview_msg);
  }

// ===================== 源码行 2412-2445 · publishPointCloud =====================
  void publishPointCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const std::vector<Point> & points,
    const std::string & frame_id)
  {
    if (!publisher || publisher->get_subscription_count() == 0) {
      return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = pointCloudStamp();
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    for (const auto & point : points) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    publisher->publish(msg);
  }

// ===================== 源码行 2510-2578 · publishSelfFilteredCloud =====================
  void publishSelfFilteredCloud(const std::vector<Point> & points)
  {
    if (!self_filtered_publisher_ ||
      self_filtered_publisher_->get_subscription_count() == 0)
    {
      return;   // 无人订阅 (Nav2 未启动): 不做任何抽稀/打包开销
    }

    // 限频: <=0 表示不限频
    if (self_filtered_publish_rate_ > 0.0) {
      const rclcpp::Time now = get_clock()->now();
      const double min_period = 1.0 / self_filtered_publish_rate_;

      if (self_filtered_last_publish_.nanoseconds() != 0 &&
        (now - self_filtered_last_publish_).seconds() < min_period)
      {
        return;
      }
      self_filtered_last_publish_ = now;
    }

    // 等间隔抽稀: 保留整帧的空间分布特征, 又控制单帧点数
    int max_points = 0;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      max_points = self_filtered_max_points_;
    }

    std::vector<Point> base_points;

    // 先把 map 系点云真实变换到 base_link 系 (Nav2 会把坐标当作 base_link 系数值,
    // 否则机器人位姿被施加两次, 整张点云搬到车体上并随航向旋转)。变换与抽稀同循环完成。
    if (transformToBaseLink(points, base_points)) {
      const std::size_t cap =
        max_points > 0 ? static_cast<std::size_t>(max_points) : 0;

      if (cap > 0 && base_points.size() > cap) {
        const std::size_t stride = (base_points.size() + cap - 1) / cap;
        std::vector<Point> sampled;
        sampled.reserve(cap + 1);

        for (std::size_t i = 0; i < base_points.size(); i += stride) {
          sampled.push_back(base_points[i]);
        }
        base_points.swap(sampled);
      }

      // 坐标与 frame_id 一致, Nav2 再做一次 TF 即回到 map 系正确位置
      publishPointCloud(self_filtered_publisher_, base_points, "base_link");
      return;
    }

    // 无里程计: 无法反推车体中心, 退回 map 帧 (marking 仍正确, 仅 clearing 退化)
    const std::size_t cap =
      max_points > 0 ? static_cast<std::size_t>(max_points) : 0;

    if (cap > 0 && points.size() > cap) {
      const std::size_t stride = (points.size() + cap - 1) / cap;
      base_points.reserve(cap + 1);

      for (std::size_t i = 0; i < points.size(); i += stride) {
        base_points.push_back(points[i]);
      }
    } else {
      base_points = points;
    }

    publishPointCloud(self_filtered_publisher_, base_points, "map");
  }

// ===================== 源码行 7346-7418 · publishDynamicObstacleCloud =====================
  void publishDynamicObstacleCloud(const std::vector<Point> & frame_points)
  {
    if (!dynamic_obstacle_publisher_ ||
      dynamic_obstacle_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    Config config;
    std::vector<uint8_t> baseline;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config = config_;
    }
    {
      std::lock_guard<std::mutex> lock(dynamic_mutex_);
      baseline = static_obstacle_mask_;
    }

    std::vector<Point> marking_points;
    marking_points.reserve(frame_points.size());

    for (const auto & point : frame_points) {
      if (point.z < config.zmin || point.z > config.zmax) {
        continue;
      }
      marking_points.push_back(point);
    }

    // 把锁定的累计历史图障碍 (冻结的静态基准) 也并入避障点云:
    //   baseline 与该帧点云同为 map 系、同一栅格几何, 直接按格中心取点。
    if (!baseline.empty()) {
      int width = 0;
      int height = 0;

      if (computeMapSize(config, width, height) &&
        baseline.size() ==
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
      {
        for (int iy = 0; iy < height; ++iy) {
          for (int ix = 0; ix < width; ++ix) {
            const std::size_t index =
              static_cast<std::size_t>(iy) *
              static_cast<std::size_t>(width) +
              static_cast<std::size_t>(ix);

            if (baseline[index] == 0) {
              continue;
            }

            marking_points.push_back(
              Point{
                static_cast<float>(config.xmin +
                (static_cast<double>(ix) + 0.5) * config.resolution),
                static_cast<float>(config.ymin +
                (static_cast<double>(iy) + 0.5) * config.resolution),
                0.0F});
          }
        }
      }
    }

    if (marking_points.empty()) {
      return;
    }

    std::vector<Point> base_points;
    if (transformToBaseLink(marking_points, base_points)) {
      publishPointCloud(dynamic_obstacle_publisher_, base_points, "base_link");
    } else {
      publishPointCloud(dynamic_obstacle_publisher_, marking_points, "map");
    }
  }

// ===================== 源码行 7432-7469 · publishClearRingCloud =====================
  void publishClearRingCloud()
  {
    if (!clear_ring_publisher_ ||
      clear_ring_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    int ring_points = 0;
    double radius = 0.0;

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      ring_points = clear_ring_points_;
      radius = clear_ring_radius_;
    }

    if (ring_points <= 0 || radius <= 0.0) {
      return;   // 关闭该机制
    }

    std::vector<Point> ring;
    ring.reserve(static_cast<std::size_t>(ring_points));

    const double step = 2.0 * M_PI / static_cast<double>(ring_points);

    for (int i = 0; i < ring_points; ++i) {
      const double angle = step * static_cast<double>(i);
      // Point 的字段是 float, 这里显式窄化避免 -Wnarrowing 警告
      ring.push_back(
        Point{
          static_cast<float>(radius * std::cos(angle)),
          static_cast<float>(radius * std::sin(angle)),
          0.0F});
    }

    publishPointCloud(clear_ring_publisher_, ring, "base_link");
  }

```

## `io/ros_path_pub.cpp`

建议落点：class RosPathPublisher

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `publishAstarPlan` | 4672-4694 | 函数 | A* 路径 -> nav_msgs/Path (map 系) 供网页显示 |
| `publishHoldPointViz` | 5221-5251 | 函数 | 发布"占点中心 + 本帧期望点"两点 Path |
| `publishHoldPointStatus` | 5254-5296 | 函数 | 发布占点状态 JSON (enabled/x/y/radius/threat*) |
| `publishChaseViz` | 5674-5710 | 函数 | 发布"目标点 + 站位点"两点 Path |
| `publishChaseStatus` | 5713-5758 | 函数 | 发布追击状态 JSON (valid/target/goal/radius/track 点数) |
| `publishChaseTrack` | 5761-5787 | 函数 | 发布目标历史轨迹 Path (网页画轨迹线) |

```cpp
// ===================== 源码行 4672-4694 · publishAstarPlan =====================
  void publishAstarPlan(const std::vector<Point> & path)
  {
    if (!astar_plan_publisher_) {
      return;
    }

    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.poses.reserve(path.size());

    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    astar_plan_publisher_->publish(msg);
  }

// ===================== 源码行 5221-5251 · publishHoldPointViz =====================
  void publishHoldPointViz(
    double center_x, double center_y, double target_x, double target_y)
  {
    if (!hold_point_viz_publisher_ ||
      hold_point_viz_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped center_pose;
    center_pose.header = path.header;
    center_pose.pose.position.x = center_x;
    center_pose.pose.position.y = center_y;
    center_pose.pose.orientation.w = 1.0;
    path.poses.push_back(center_pose);

    if (std::hypot(target_x - center_x, target_y - center_y) > 1e-3) {
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header = path.header;
      target_pose.pose.position.x = target_x;
      target_pose.pose.position.y = target_y;
      target_pose.pose.orientation.w = 1.0;
      path.poses.push_back(target_pose);
    }

    hold_point_viz_publisher_->publish(path);
  }

// ===================== 源码行 5254-5296 · publishHoldPointStatus =====================
  void publishHoldPointStatus()
  {
    if (!hold_point_status_publisher_ ||
      hold_point_status_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    bool enabled = false;
    double cx = 0.0;
    double cy = 0.0;
    double radius = 0.0;
    bool threat_valid = false;
    double tx = 0.0;
    double ty = 0.0;
    double tdist = 0.0;

    {
      std::lock_guard<std::mutex> lock(hold_point_mutex_);
      enabled = hold_point_enabled_;
      cx = hold_point_x_;
      cy = hold_point_y_;
      radius = hold_point_radius_;
      threat_valid = hold_point_threat_valid_;
      tx = hold_point_threat_x_;
      ty = hold_point_threat_y_;
      tdist = hold_point_threat_dist_;
    }

    std_msgs::msg::String msg;
    msg.data = "{";
    msg.data += "\"enabled\":" + std::string(enabled ? "true" : "false");
    msg.data += ",\"x\":" + std::to_string(cx);
    msg.data += ",\"y\":" + std::to_string(cy);
    msg.data += ",\"radius\":" + std::to_string(radius);
    msg.data += ",\"threat\":" + std::string(threat_valid ? "true" : "false");
    msg.data += ",\"threat_x\":" + std::to_string(tx);
    msg.data += ",\"threat_y\":" + std::to_string(ty);
    msg.data += ",\"threat_dist\":" + std::to_string(tdist);
    msg.data += "}";

    hold_point_status_publisher_->publish(msg);
  }

// ===================== 源码行 5674-5710 · publishChaseViz =====================
  void publishChaseViz(
    bool has_target, double target_x, double target_y,
    double goal_x, double goal_y)
  {
    if (!chase_viz_publisher_ ||
      chase_viz_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";

    // 第 1 点 = 当前目标位置 (has_target=false 时跳过, 站点仍会发布)
    if (has_target) {
      geometry_msgs::msg::PoseStamped p0;
      p0.header = path.header;
      p0.pose.position.x = target_x;
      p0.pose.position.y = target_y;
      p0.pose.orientation.w = 1.0;
      path.poses.push_back(p0);
    }

    if (path.poses.empty() ||
      std::hypot(goal_x - target_x, goal_y - target_y) > 1e-3)
    {
      geometry_msgs::msg::PoseStamped p1;
      p1.header = path.header;
      p1.pose.position.x = goal_x;
      p1.pose.position.y = goal_y;
      p1.pose.orientation.w = 1.0;
      path.poses.push_back(p1);
    }

    chase_viz_publisher_->publish(path);
  }

// ===================== 源码行 5713-5758 · publishChaseStatus =====================
  void publishChaseStatus()
  {
    if (!chase_status_publisher_ ||
      chase_status_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    bool enabled = false;
    bool valid = false;
    double tx = 0.0;
    double ty = 0.0;
    double gx = 0.0;
    double gy = 0.0;
    double radius = 0.0;
    double track_radius = 0.0;
    std::size_t track_size = 0;

    {
      std::lock_guard<std::mutex> lock(chase_mutex_);
      enabled = chase_enabled_;
      valid = chase_target_valid_;
      tx = chase_target_x_;
      ty = chase_target_y_;
      gx = chase_goal_x_;
      gy = chase_goal_y_;
      radius = chase_radius_;
      track_radius = chase_track_radius_;
      track_size = chase_track_.size();
    }

    std_msgs::msg::String msg;
    msg.data = "{";
    msg.data += "\"enabled\":" + std::string(enabled ? "true" : "false");
    msg.data += ",\"valid\":" + std::string(valid ? "true" : "false");
    msg.data += ",\"target_x\":" + std::to_string(tx);
    msg.data += ",\"target_y\":" + std::to_string(ty);
    msg.data += ",\"goal_x\":" + std::to_string(gx);
    msg.data += ",\"goal_y\":" + std::to_string(gy);
    msg.data += ",\"radius\":" + std::to_string(radius);
    msg.data += ",\"track_radius\":" + std::to_string(track_radius);
    msg.data += ",\"track\":" + std::to_string(track_size);
    msg.data += "}";

    chase_status_publisher_->publish(msg);
  }

// ===================== 源码行 5761-5787 · publishChaseTrack =====================
  void publishChaseTrack()
  {
    if (!chase_track_publisher_ ||
      chase_track_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";

    {
      std::lock_guard<std::mutex> lock(chase_mutex_);

      for (const auto & p : chase_track_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
    }

    chase_track_publisher_->publish(path);
  }

```

## `io/ros_motion_pub.cpp`

建议落点：class RosMotionPublisher

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `publishMotionFrame` | 3432-3474 | 函数 | 组装 8 字节运动帧(control 位 + vx/vy 各 12 位, vy 下发取反) -> 发 /motion -> 可选 CAN 直发 |

```cpp
// ===================== 源码行 3432-3474 · publishMotionFrame =====================
  void publishMotionFrame(bool control, double vx, double vy)
  {
    double vx_scale = kMotionSpeedScale;
    double vy_scale = kMotionSpeedScale;
    int control_value = 1;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      vx_scale = motion_vx_scale_;
      vy_scale = motion_vy_scale_;
      control_value = motion_control_;
    }

    // 先做保方向限幅: 任一轴超过单轴上限时, 两轴按同一比例等比缩小
    limitMotionSpeed(vx, vy);

    // 12 位有符号速度量: 低 12 位为有效数据, 负数为补码
    // 下发前对 vy 取反: 下位机对 vy 的符号约定与导航侧相反 (横向分量整体反号)
    const uint16_t vx_raw = encodeMotionRaw(vx, vx_scale);
    const uint16_t vy_raw = encodeMotionRaw(-vy, vy_scale);

    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(8);
    // control 位: 运动时取参数 motion_control (默认 1), 停车时恒为 0
    const uint8_t control_bit = control ? static_cast<uint8_t>(
      (std::clamp(control_value, 0, 255) != 0) ? 1 : 0) : 0U;
    msg.data[0] = static_cast<uint8_t>(control_bit & 0x01U);  // bit0=control, bit1=shoot(0)
    msg.data[1] = 0U;                 // tar_yaw    高 8 位 (导航不涉及)
    msg.data[2] = 0U;                 // tar_yaw    低 8 位
    msg.data[3] = 0U;                 // tar_pitch  高 8 位 (导航不涉及)
    msg.data[4] = 0U;                 // tar_pitch  低 8 位
    msg.data[5] = static_cast<uint8_t>((vx_raw >> 4) & 0xFF);       // vx 高 8 位
    msg.data[6] = static_cast<uint8_t>(((vx_raw & 0x0F) << 4) |    // vx 低 4 位
      ((vy_raw >> 8) & 0x0F));                                     // vy 高 4 位
    msg.data[7] = static_cast<uint8_t>(vy_raw & 0xFF);              // vy 低 8 位

    motion_publisher_->publish(msg);

    // 同一份 8 字节直接写进 CAN 总线 (开启 can_enabled 时)
    // 这样即使没有任何桥接节点, candump 也能看到 0xFF 报文
    if (can_enabled_) {
      canFrameFromMotion(msg.data.data());
    }
  }

```

## `io/ros_pose_pub.cpp`

建议落点：class RosPosePublisher

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `publishPoseCorrection` | 1342-1359 | 函数 | 修正量转 PoseStamped 并发布到 /pose_correction |

```cpp
// ===================== 源码行 1342-1359 · publishPoseCorrection =====================
  void publishPoseCorrection(const PoseCorrection & c)
  {
    geometry_msgs::msg::PoseStamped correction;
    correction.header.stamp = get_clock()->now();
    correction.header.frame_id = "map";
    correction.pose.position.x = c.t[0];
    correction.pose.position.y = c.t[1];
    correction.pose.position.z = c.t[2];

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    matrixToQuaternion(c.r, qx, qy, qz, qw);
    correction.pose.orientation.x = qx;
    correction.pose.orientation.y = qy;
    correction.pose.orientation.z = qz;
    correction.pose.orientation.w = qw;

    pose_correction_publisher_->publish(correction);
  }

```

## `io/ros_stamp.cpp`

建议落点：namespace nav::io

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `pointCloudStamp` | 2384-2409 | 函数 | 生成 Nav2 安全时间戳: 最新里程计戳 - 0.1 s (避免 TF 外推到未来导致整帧被丢) |

```cpp
// ===================== 源码行 2384-2409 · pointCloudStamp =====================
  rclcpp::Time pointCloudStamp()
  {
    constexpr double kTfStampMargin = 0.1;
    constexpr double kMaxOdomAge = 1.0;

    bool odom_ok = false;
    rclcpp::Time odom_stamp(0, 0, RCL_ROS_TIME);
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      odom_ok = odom_received_;
      odom_stamp = odom_stamp_;
    }

    const rclcpp::Time now = get_clock()->now();

    // 里程计掉线/长时间未更新: TF 同样不会有新数据, 退回 now() 并交给 Nav2 按现状丢弃
    if (!odom_ok || (now - odom_stamp).seconds() > kMaxOdomAge) {
      return now;
    }

    const rclcpp::Time stamp =
      odom_stamp - rclcpp::Duration::from_seconds(kTfStampMargin);

    // 里程计戳异常跳到未来时兜底, 绝不发出比 now() 更晚的戳
    return stamp < now ? stamp : now;
  }

```

## `io/ros_nav2_client.cpp`

建议落点：class Nav2ServiceClient

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `clearCostmaps` | 3317-3361 | 函数 | 异步调用 Nav2 ClearEntireCostmap 清空局部/全局代价地图, 不阻塞渲染线程 |

```cpp
// ===================== 源码行 3317-3361 · clearCostmaps =====================
  void clearCostmaps()
  {
    if (!clear_costmap_on_history_reset_) {
      return;
    }

    const auto try_clear =
      [this](const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr & client,
        const std::string & name)
      {
        if (!client) {
          return;
        }

        if (!client->service_is_ready()) {
          RCLCPP_WARN(
            get_logger(),
            "刷新历史图: 代价地图清空服务 %s 未就绪, 本次跳过 (红区可能残留)",
            name.c_str());
          return;
        }

        auto request =
          std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

        client->async_send_request(
          request,
          [this, name](
            rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
          {
            if (future.valid()) {
              RCLCPP_INFO(
                get_logger(),
                "刷新历史图: 已清空代价地图 %s", name.c_str());
            } else {
              RCLCPP_WARN(
                get_logger(),
                "刷新历史图: 清空代价地图 %s 调用失败", name.c_str());
            }
          });
      };

    try_clear(clear_local_costmap_client_, clear_local_costmap_service_);
    try_clear(clear_global_costmap_client_, clear_global_costmap_service_);
  }

```

## `io/file_pcd.cpp`

建议落点：namespace nav::io

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `saveCloudPcd` | 867-909 | 函数 | 把一帧点云写成 ascii PCD 快照 (ASCII, 带时间戳文件名) |

```cpp
// ===================== 源码行 867-909 · saveCloudPcd =====================
  bool saveCloudPcd(
    const std::vector<Point> & cloud,
    const std::string & tag,
    const rclcpp::Time & stamp)
  {
    if (!debug_pcd_enabled_ ||
      debug_pcd_saved_ >= static_cast<std::size_t>(
        std::max(0, debug_pcd_max_files_)))
    {
      return false;
    }

    // 文件名: cloud_<tag>_<sec>_<nanosec>.pcd
    const std::string path =
      debug_log_dir_ + "/cloud_" + tag + "_" +
      std::to_string(stamp.nanoseconds()) + ".pcd";

    std::ofstream file(path, std::ios::out | std::ios::trunc);

    if (!file) {
      return false;
    }

    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << cloud.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << cloud.size() << "\n";
    file << "DATA ascii\n";

    for (const auto & point : cloud) {
      file << point.x << ' ' << point.y << ' ' << point.z << '\n';
    }

    file.flush();
    ++debug_pcd_saved_;
    return true;
  }

```

## `io/file_bin.cpp`

建议落点：namespace nav::io

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `writeHistoryCloudsBin` | 6839-6892 | 函数 | 历史点云队列 -> 自有二进制 (magic + 每帧时间戳 + float32 xyz) |
| `readHistoryCloudsBin` | 6895-6975 | 函数 | 自有二进制 -> 历史点云队列 (反序列化, 校验 magic/长度) |

```cpp
// ===================== 源码行 6839-6892 · writeHistoryCloudsBin =====================
  bool writeHistoryCloudsBin(
    const std::deque<TimedCloud> & history,
    const std::string & path)
  {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);

    if (!file) {
      RCLCPP_ERROR(get_logger(), "无法创建点云导出文件: %s", path.c_str());
      return false;
    }

    const char magic[8] = {'R', 'P', 'G', 'M', 'C', 'L', 'D', '1'};
    file.write(magic, 8);

    const uint32_t frame_count =
      static_cast<uint32_t>(history.size());
    file.write(
      reinterpret_cast<const char *>(&frame_count), sizeof(frame_count));

    for (const auto & timed : history) {
      const double stamp = timed.stamp.seconds();
      file.write(reinterpret_cast<const char *>(&stamp), sizeof(stamp));

      const uint32_t point_count = timed.points ?
        static_cast<uint32_t>(timed.points->size()) : 0u;
      file.write(
        reinterpret_cast<const char *>(&point_count), sizeof(point_count));

      if (point_count == 0) {
        continue;
      }

      // 逐点写 float32 xyz, 避免结构体对齐/填充差异导致的跨平台不一致
      std::vector<float> buffer;
      buffer.reserve(static_cast<std::size_t>(point_count) * 3);

      for (const auto & point : *timed.points) {
        buffer.push_back(static_cast<float>(point.x));
        buffer.push_back(static_cast<float>(point.y));
        buffer.push_back(static_cast<float>(point.z));
      }

      file.write(
        reinterpret_cast<const char *>(buffer.data()),
        static_cast<std::streamsize>(buffer.size() * sizeof(float)));
    }

    if (!file.good()) {
      RCLCPP_ERROR(get_logger(), "写入点云导出文件失败: %s", path.c_str());
      return false;
    }

    return true;
  }

// ===================== 源码行 6895-6975 · readHistoryCloudsBin =====================
  bool readHistoryCloudsBin(
    const std::string & path,
    std::deque<TimedCloud> & history)
  {
    std::ifstream file(path, std::ios::binary);

    if (!file) {
      RCLCPP_ERROR(get_logger(), "无法打开点云导入文件: %s", path.c_str());
      return false;
    }

    char magic[8] = {0};
    file.read(magic, 8);

    if (!file.good() ||
      std::memcmp(magic, "RPGMCLD1", 8) != 0)
    {
      RCLCPP_ERROR(get_logger(), "点云导入文件格式不匹配: %s", path.c_str());
      return false;
    }

    uint32_t frame_count = 0;
    file.read(reinterpret_cast<char *>(&frame_count), sizeof(frame_count));

    if (!file.good() || frame_count > 100000u) {
      RCLCPP_ERROR(get_logger(), "点云导入文件帧数异常: %s", path.c_str());
      return false;
    }

    std::deque<TimedCloud> loaded;

    for (uint32_t frame = 0; frame < frame_count; ++frame) {
      double stamp = 0.0;
      file.read(reinterpret_cast<char *>(&stamp), sizeof(stamp));

      uint32_t point_count = 0;
      file.read(reinterpret_cast<char *>(&point_count), sizeof(point_count));

      if (!file.good() || point_count > 10000000u) {
        RCLCPP_ERROR(
          get_logger(), "点云导入文件第 %u 帧异常: %s", frame, path.c_str());
        return false;
      }

      auto points = std::make_shared<std::vector<Point>>();
      points->reserve(point_count);

      std::vector<float> buffer(
        static_cast<std::size_t>(point_count) * 3);

      if (point_count > 0) {
        file.read(
          reinterpret_cast<char *>(buffer.data()),
          static_cast<std::streamsize>(buffer.size() * sizeof(float)));

        if (!file.good()) {
          RCLCPP_ERROR(
            get_logger(), "点云导入文件第 %u 帧数据截断: %s",
            frame, path.c_str());
          return false;
        }

        for (uint32_t index = 0; index < point_count; ++index) {
          Point point;
          point.x = buffer[index * 3 + 0];
          point.y = buffer[index * 3 + 1];
          point.z = buffer[index * 3 + 2];
          points->push_back(point);
        }
      }

      TimedCloud timed;
      timed.stamp = rclcpp::Time(
        static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);
      timed.points = points;
      loaded.push_back(timed);
    }

    history = std::move(loaded);
    return true;
  }

```

## `io/file_yaml.cpp`

建议落点：namespace nav::io

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `writeMapYaml` | 6985-7021 | 函数 | 写 map_server 风格 YAML (image/resolution/origin/negate/occupied_thresh) |

```cpp
// ===================== 源码行 6985-7021 · writeMapYaml =====================
  bool writeMapYaml(
    const std::string & path,
    const Config & config,
    const std::string & image_name,
    int width,
    int height)
  {
    std::ofstream file(path, std::ios::trunc);

    if (!file) {
      RCLCPP_ERROR(get_logger(), "无法创建地图 YAML: %s", path.c_str());
      return false;
    }

    // PGM 第 0 行在图像顶部 = ymax 侧, 因此 origin 的 y 取 ymin (map_server
    //   的 origin 是图像左下角像素在世界系的坐标)。
    file << "image: " << image_name << "\n";
    file << "resolution: " << config.resolution << "\n";
    file << "origin: [" << config.xmin << ", " << config.ymin << ", 0.0]\n";
    file << "negate: 0\n";
    file << "occupied_thresh: 0.1\n";
    file << "free_thresh: 0.9\n";
    // 附注字段 (不影响 map_server, 供本节点导入时校验尺寸/一致性)
    file << "# width: " << width << "\n";
    file << "# height: " << height << "\n";
    file << "# xmin: " << config.xmin << "\n";
    file << "# ymin: " << config.ymin << "\n";
    file << "# xmax: " << config.xmax << "\n";
    file << "# ymax: " << config.ymax << "\n";

    if (!file.good()) {
      RCLCPP_ERROR(get_logger(), "写入地图 YAML 失败: %s", path.c_str());
      return false;
    }

    return true;
  }

```

## `io/debug_log_file.cpp`

建议落点：class DebugLogFile

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `initDebugLog` | 824-864 | 函数 | 启动期一次性初始化: 建 debug_log_dir_ 目录, 打开 cloud_frames.csv 写表头 |

```cpp
// ===================== 源码行 824-864 · initDebugLog =====================
  void initDebugLog()
  {
    if (!debug_log_enabled_) {
      RCLCPP_INFO(get_logger(), "调试日志: 已关闭 (debug_log_enabled=false)");
      return;
    }

    std::error_code ec;
    std::filesystem::create_directories(debug_log_dir_, ec);

    if (ec) {
      RCLCPP_WARN(
        get_logger(),
        "调试日志: 创建目录失败 %s (%s), 调试日志关闭",
        debug_log_dir_.c_str(), ec.message().c_str());
      debug_log_enabled_ = false;
      return;
    }

    const std::string csv_path = debug_log_dir_ + "/cloud_frames.csv";
    debug_cloud_log_.open(csv_path, std::ios::out | std::ios::trunc);

    if (!debug_cloud_log_) {
      RCLCPP_WARN(
        get_logger(), "调试日志: 无法打开 %s, 调试日志关闭", csv_path.c_str());
      debug_log_enabled_ = false;
      return;
    }

    debug_cloud_log_ <<
      "stamp,frame,raw,valid,self_filtered,queued,"
      "cx,cy,cz,xmin,xmax,ymin,ymax,zmin,zmax,anomaly\n";
    debug_cloud_log_.flush();

    RCLCPP_INFO(
      get_logger(),
      "调试日志: 已开启, 逐帧统计 -> %s, PCD 快照=%s (上限 %d 个)",
      csv_path.c_str(),
      debug_pcd_enabled_ ? "开" : "关",
      debug_pcd_max_files_);
  }

```

## `tasks/cloud_pipeline.cpp`

建议落点：class CloudPipeline

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `cloudCallback` | 1715-1949 | 函数 | 点云主流水线: 字段校验 -> 解析+isfinite -> 跳变拒绝窗丢弃 -> 位姿修正 -> 地图裁剪 -> 车体点级自过滤 -> 发自过滤点云 -> 动态障碍分离/绿层/清除环 -> 历史降采样 -> 调试日志 -> 入队 |
| `pruneHistory` | 1952-1973 | 函数 | 按 history_duration 淘汰历史队列队首; 动态障碍模式下直接跳过 |

```cpp
// ===================== 源码行 1715-1949 · cloudCallback =====================
  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!hasField(*msg, "x") ||
      !hasField(*msg, "y") ||
      !hasField(*msg, "z"))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "点云缺少 x/y/z 字段，跳过当前帧");
      return;
    }

    const std::size_t raw_point_count =
      static_cast<std::size_t>(msg->width) *
      static_cast<std::size_t>(msg->height);
    const rclcpp::Time cloud_stamp = msg->header.stamp;

    std::vector<Point> new_cloud;
    new_cloud.reserve(raw_point_count);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      for (; iter_x != iter_x.end();
        ++iter_x, ++iter_y, ++iter_z)
      {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;

        if (std::isfinite(x) &&
          std::isfinite(y) &&
          std::isfinite(z))
        {
          new_cloud.push_back({x, y, z});
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "解析 PointCloud2 失败: %s",
        e.what());
      return;
    }

    // 解析后有效点数 (isfinite 过滤后); raw 与 valid 之差即 NaN/Inf 点数
    const std::size_t valid_point_count = new_cloud.size();

    // ---- 位姿跳变"拒绝窗口": 丢弃该帧, 阻止错位点云污染地图 ----
    // 原理: 里程计跳变后, 本帧已是 map 系的点云会被整体搬到错误位置。若照常写入
    //   历史缓冲, 只要错位持续 >= history_min_hits 帧就被判成占据, 并留存整个
    //   history_duration (默认 300 s); 同时经 publishSelfFilteredCloud/
    //   publishDynamicObstacleCloud 以 marking 进 Nav2, inflate 后形成大片假红区,
    //   直接导致导航中止。
    //   处理: 窗口内(默认跳变后 0.8 s)的点云帧一律丢弃 —— 不入历史缓冲、不发
    //   Nav2、不做动态障碍判定; 仅保留上面的预览发布与下方 latest_cloud_ 更新,
    //   使网页仍能看到实时点云。历史缓冲按既定策略"只丢帧、不清空", 靠后续正常
    //   帧与滑动时间窗自然淘汰旧数据。
    if (pose_jump_reject_sec_ > 0.0 && pose_jump_has_until_.load()) {
      const double cloud_sec = cloud_stamp.seconds();

      if (cloud_sec <= pose_jump_until_sec_.load()) {
        {
          std::lock_guard<std::mutex> lock(cloud_mutex_);
          latest_cloud_.swap(new_cloud);
          received_cloud_ = true;
          ++cloud_frames_;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "位姿跳变拒绝窗口内, 已丢弃当前点云帧 (不写入历史/不发 Nav2)");
        return;
      }
      // 窗口已过, 关闭标记, 恢复正常处理
      pose_jump_has_until_ = false;
    }

    // 锁定历史图后，对当前 map 系点云进行二维校正：校正结果会同时用于
    // 动态障碍判定、Nav2 点云发布以及内部导航位姿。
    correctCloudForPose(new_cloud);

    // ---- 地图矩形范围裁剪 (开阔场景远距离回波滤波) ----
    //   开阔场景下雷达会把 xmin..xmax/ymin..ymax 之外的远距离回波一并送回。
    //   这些点既进不了栅格图, 又会被动态障碍分离判为"范围外"并持续刷 WARN。
    //   这里在校正之后、任何下游消费之前统一裁剪一次, 使:
    //     - 单帧图/历史图 (accumulateCloud)、绿色动态层 (subtractBaseline/
    //       splitDynamicObstacles)、发往 Nav2 的自过滤点云看到的都是同一份
    //       "框内点云";
    //     - splitDynamicObstacles 的 out_of_bounds 恒为 0, "范围外" WARN 消除。
    const std::size_t clipped = clipCloudToMapBounds(new_cloud);

    if (clipped > 0) {
      RCLCPP_DEBUG(
        get_logger(),
        "地图范围裁剪: 丢弃 %zu 个 xmin..xmax/ymin..ymax 之外的点 (剩余 %zu 点)",
        clipped,
        new_cloud.size());
    }

    // ---- 车体自身点云点级自过滤 (治本, 在进入任何缓冲之前执行) ----
    // 雷达装在车上, 近距离必然扫到车体/支架/线缆。若这些点随点云一起进入
    // 历史缓冲, 由于点云已是 map 系, 它们会被"烧"在当时的车体位置上; 车辆一
    // 移动, 各时刻的车体位置连起来就在历史图上留下一条"轨迹带"(假障碍)。
    // 栅格级清除 (clearVehicleFootprint) 只能挡住当前时刻的那一格方框, 挡不住
    // 历史位置, 故必须在点云入缓冲前直接剔除。
    //   放在这里(而非前面)是为了不影响 publishCloudPreview: 预览仍显示原始点云。
    //   放在 splitDynamicObstacles 之前, 是为了让车体自身点既不会进单帧图/历史
    //   缓冲, 也不会因"基准图该处为空"而被误判成动态障碍(绿色)。
    const std::size_t self_filtered =
      filterVehicleFootprintPoints(new_cloud);

    if (self_filtered > 0) {
      RCLCPP_DEBUG(
        get_logger(),
        "车体点级自过滤: 丢弃 %zu 个车体方框内的自身点 (剩余 %zu 点)",
        self_filtered,
        new_cloud.size());
    }

    // ---- 发布自过滤点云 (供 Nav2 obstacle_layer 的 clearing 射线) ----
    // 此刻 new_cloud 已是"剔除车体自身点"的完整当前帧点云 (静态墙面 + 动态障碍都
    // 在里面)。Nav2 会从 base_link 原点向每个点连线, 射线穿过的格子置为空闲 ->
    // 车体视线可达范围内的旧红色残留全部被清掉。
    //   注意: 必须在 splitDynamicObstacles 之前拷贝, 否则只剩静态点, 对人离开后
    //   的区域 (那里现在什么点都没有) 反而没有任何射线去清除。
    publishSelfFilteredCloud(new_cloud);

    // ---- 动态障碍处理 (网页"锁定历史累计图"后生效) ----
    // 锁定后, 落在"静态基准图未占据格"上的点被判为临时障碍, 分两条用途:
    //   - splitDynamicObstacles: 就地剔除这些点, 使单帧图/静态图只保留环境结构;
    //   - subtractBaseline: 另出一份"整帧 − 基准占据"的绿色层点云 (不修改整帧),
    //     供绿色栅格图/避障点云使用 —— 绿色层与建图共用同一帧点云。
    // 未锁定时两者都不产出, 行为与改动前完全一致。
    std::vector<Point> dynamic_obstacles;

    // 绿色层数据源: "整帧点云 − 基准占据" (构造见 subtractBaseline)。
    //   与建图共用同一帧自过滤点云, 只把已确认为静态环境的基准格扣除, 剩余点
    //   即为绿色层显示的动态障碍。必须在 splitDynamicObstacles 之前捕获整帧:
    //   该函数是原地修改 new_cloud (把动态点摘走, 只留静态点)。
    //   非锁定模式下保持为空 (绿色层本就不发布)。
    std::vector<Point> green_layer_points;

    if (dynamic_obstacle_enabled_.load()) {
      // 用整帧点云做基准扣除 (输入不修改 new_cloud, 故与下方 split 互不影响)
      subtractBaseline(new_cloud, green_layer_points);

      splitDynamicObstacles(new_cloud, dynamic_obstacles);

      // 即时发布: 绿色障碍出现/消失后立刻通知 Nav2, 不再等渲染定时器。
      // 放在这里(而非 renderDynamicObstaclePgm)是为了让代价地图的红色区域
      // 尽可能紧跟绿色障碍 —— 这是"实时跟随"的核心提速点。
      // 输入用 green_layer_points: 与绿色栅格层完全同源 (整帧 − 基准), 两者
      // 障碍一致; 函数内部再并入锁定的静态基准障碍, 保证不撞累计历史图里的
      // 冻结障碍。
      publishDynamicObstacleCloud(green_layer_points);

      // 清除射线: 每帧都发一圈 base_link 系稀疏点到"只 clearing 不 marking"的
      // clear_ring 源上, 让 obstacle_layer 从车体原点向四周射射线, 把绿色障碍
      // 消失后残留的旧红格持续清成空闲。
      // 原理: obstacle_layer 是累积栅格, 红色只在被射线穿过时才消失; 而
      // dynamic_obstacles 源只有在障碍还看得见时才有射线, 障碍一离开就再没有
      // 点经过旧红格 -> 单靠它清不动 (原来正是"红区消失慢/残留"的根因)。
      // 为什么每帧都发而不是只在绿色消失时发: 环点落在 marking:false 的源上,
      // 本身不会被标成障碍, 恒定发射让清除覆盖不依赖"绿色是否为空"的瞬时状态,
      // 时序上更稳。详见 publishClearRingCloud。
      publishClearRingCloud();
    }

    // 历史缓冲帧降采样, 控制内存占用
    std::vector<Point> history_points;

    // 动态障碍模式下本帧点云根本不进历史缓冲 (见下方 push_back 的条件), 若这里
    // 照旧降采样整帧, 就是每帧白做一次 stride 扫描 + 一次拷贝, 结果直接丢弃。
    if (!dynamic_obstacle_enabled_.load()) {
      int history_max_points_per_frame = 5000;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        history_max_points_per_frame = config_.history_max_points_per_frame;
      }

      const std::size_t history_cap =
        static_cast<std::size_t>(
          std::max(1, history_max_points_per_frame));

      if (new_cloud.size() > history_cap) {
        const std::size_t stride =
          (new_cloud.size() + history_cap - 1) / history_cap;

        for (std::size_t i = 0; i < new_cloud.size(); i += stride) {
          history_points.push_back(new_cloud[i]);
        }
      } else {
        history_points = new_cloud;
      }
    }

    // ---- 调试日志: 逐帧统计 + 异常检测 (点数/质心/包围盒/自过滤数) ----
    //   必须在 latest_cloud_.swap(new_cloud) 之前调用 — swap 后 new_cloud 会变成
    //   旧的 latest_cloud_ 内容, 统计与快照都会错。queued 用 history_points.size()
    //   (动态模式下为 0, 表示本帧不入历史缓冲)。
    logCloudFrame(
      cloud_stamp,
      raw_point_count,
      valid_point_count,
      self_filtered,
      history_points.size(),
      new_cloud);

    const rclcpp::Time now = get_clock()->now();

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_cloud_.swap(new_cloud);
      // 动态障碍模式下不再累积历史: 历史图已冻结为静态基准, 继续累积会
      // 把新点云重新烧进静态图, 使动态障碍判定失去参照 (也会重现假障碍)。
      if (!dynamic_obstacle_enabled_.load()) {
        history_.push_back(
          TimedCloud{
            now,
            std::make_shared<std::vector<Point>>(
              std::move(history_points))});
        pruneHistory(now);
      }
      latest_dynamic_obstacles_ = std::move(green_layer_points);
      received_cloud_ = true;
      ++cloud_frames_;
    }
  }

// ===================== 源码行 1952-1973 · pruneHistory =====================
  void pruneHistory(const rclcpp::Time & now)
  {
    // 动态障碍模式 (网页"锁定历史累计图") 下历史缓冲已冻结: 锁定后本帧点云
    //   不再入队 (见 cloudCallback), 若仍按时长淘汰队首, 历史图会在
    //   history_duration 后被单向清空 -> 表现为"锁定后地图过一会儿自动消失"。
    //   因此锁定时直接跳过裁剪, 让历史点云保持锁定那一刻的内容。
    if (dynamic_obstacle_enabled_.load()) {
      return;
    }

    double history_duration = 300.0;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      history_duration = config_.history_duration;
    }

    while (!history_.empty() &&
      (now - history_.front().stamp).seconds() > history_duration)
    {
      history_.pop_front();
    }
  }

```

## `tasks/dynamic_split.cpp`

建议落点：class DynamicSplitter

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `splitDynamicObstacles` | 1985-2077 | 函数 | 用冻结的静态基准掩码把"新出现的临时障碍点"从整帧中原地摘出 |
| `subtractBaseline` | 2087-2156 | 函数 | 同判据但不改输入, 输出"整帧 - 基准占据"的动态点 (绿色层数据源) |
| `buildStaticBaseline` | 2161-2320 | 函数 | 锁定瞬间: 历史累积图按 history_min_hits 投票 + 并入成品图 + 扣车体 + baseline_dilate 膨胀 -> 基准栅格; 再膨胀 1 格得到配准评分容差掩码 |

```cpp
// ===================== 源码行 1985-2077 · splitDynamicObstacles =====================
  void splitDynamicObstacles(
    std::vector<Point> & cloud,
    std::vector<Point> & dynamic)
  {
    int width = 0;
    int height = 0;
    Config config;

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config = config_;
    }

    if (!computeMapSize(config, width, height)) {
      return;
    }

    std::vector<uint8_t> baseline;
    {
      std::lock_guard<std::mutex> lock(dynamic_mutex_);
      if (static_obstacle_mask_.empty()) {
        return;
      }
      baseline = static_obstacle_mask_;
    }

    const double resolution = config.resolution;
    const double xmin = config.xmin;
    const double ymin = config.ymin;

    const auto to_index = [&](double x, double y,
        std::size_t & index) -> bool
      {
        const int ix = static_cast<int>(std::floor((x - xmin) / resolution));
        const int iy = static_cast<int>(std::floor((y - ymin) / resolution));

        if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
          return false;
        }

        index = static_cast<std::size_t>(iy) *
          static_cast<std::size_t>(width) +
          static_cast<std::size_t>(ix);
        return true;
      };

    std::size_t write = 0;
    std::size_t out_of_bounds = 0;

    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const Point & point = cloud[i];
      std::size_t index = 0;

      // z 过滤与 accumulateCloud 保持一致 (高度范围外的点既不静态也不动态)
      if (point.z < config.zmin || point.z > config.zmax) {
        cloud[write++] = point;
        continue;
      }

      if (!to_index(point.x, point.y, index)) {
        // 地图范围外的点: 基准图里根本没有它的信息, 不能当作"静态环境"
        // 保留 (那会让范围外的障碍永远无法变绿), 一并判为动态障碍。
        // 注意 renderDynamicObstaclePgm 用同一套 accumulateCloud 过滤,
        // 范围外的点同样进不了动态栅格图, 故此处仍会提示用户扩大范围。
        dynamic.push_back(point);
        ++out_of_bounds;
        continue;
      }

      if (baseline[index] == 0) {
        dynamic.push_back(point);
        continue;
      }

      cloud[write++] = point;
    }

    cloud.resize(write);

    if (out_of_bounds > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "动态障碍分离: %zu 个点在当前地图范围 [%.1f,%.1f]x[%.1f,%.1f] 之外, "
        "已按动态障碍处理 (请检查 xmin/xmax/ymin/ymax 设置)",
        out_of_bounds,
        config.xmin,
        config.xmax,
        config.ymin,
        config.ymax);
    }
  }

// ===================== 源码行 2087-2156 · subtractBaseline =====================
  void subtractBaseline(
    const std::vector<Point> & cloud,
    std::vector<Point> & dynamic)
  {
    int width = 0;
    int height = 0;
    Config config;

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config = config_;
    }

    if (!computeMapSize(config, width, height)) {
      return;
    }

    std::vector<uint8_t> baseline;
    {
      std::lock_guard<std::mutex> lock(dynamic_mutex_);
      baseline = static_obstacle_mask_;
    }

    if (baseline.empty()) {
      return;   // 未锁定: 不做扣除
    }

    const double resolution = config.resolution;
    const double xmin = config.xmin;
    const double ymin = config.ymin;

    const auto to_index = [&](double x, double y,
        std::size_t & index) -> bool
      {
        const int ix = static_cast<int>(std::floor((x - xmin) / resolution));
        const int iy = static_cast<int>(std::floor((y - ymin) / resolution));

        if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
          return false;
        }

        index = static_cast<std::size_t>(iy) *
          static_cast<std::size_t>(width) +
          static_cast<std::size_t>(ix);
        return true;
      };

    dynamic.clear();
    dynamic.reserve(cloud.size());

    for (const auto & point : cloud) {
      // z 过滤与 accumulateCloud / splitDynamicObstacles 保持一致
      if (point.z < config.zmin || point.z > config.zmax) {
        continue;
      }

      std::size_t index = 0;

      // 地图范围外的点: 基准无信息 -> 视为动态 (与 splitDynamicObstacles 一致)
      if (!to_index(point.x, point.y, index)) {
        dynamic.push_back(point);
        continue;
      }

      // 落在基准占据格上的点 = 静态环境, 扣除; 其余即为动态障碍
      if (baseline[index] == 0) {
        dynamic.push_back(point);
      }
    }
  }

// ===================== 源码行 2161-2320 · buildStaticBaseline =====================
  void buildStaticBaseline()
  {
    std::deque<TimedCloud> history_copy;
    Config config;

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config = config_;
    }

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      history_copy = history_;
    }

    int width = 0;
    int height = 0;

    if (!computeMapSize(config, width, height)) {
      return;
    }

    std::vector<uint8_t> occupancy(
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(height), 0);
    std::vector<uint8_t> hit_counts(occupancy.size(), 0);

    std::size_t filtered_points = 0;

    for (const auto & timed_cloud : history_copy) {
      accumulateCloud(
        *timed_cloud.points, config, width, height, occupancy,
        filtered_points, &hit_counts);
    }

    const uint8_t min_hits = static_cast<uint8_t>(
      std::min(255, std::max(1, config.history_min_hits)));

    for (std::size_t index = 0; index < occupancy.size(); ++index) {
      occupancy[index] = hit_counts[index] >= min_hits ? 1 : 0;
    }

    // 若存在导入的成品图 (尺寸一致), 把其占据格并入静态基准: 这样即使只导入
    //   成品图 (没有点云队列), 锁定历史累计图后仍能正确区分静态环境与动态障碍。
    {
      std::lock_guard<std::mutex> lock(imported_map_mutex_);

      if (imported_map_valid_ &&
        imported_map_width_ == width && imported_map_height_ == height &&
        imported_map_pixels_.size() == occupancy.size())
      {
        // 图像第 0 行在顶部 (y 轴翻转), 需与 occupancy 的 y 方向对齐
        for (int iy = 0; iy < height; ++iy) {
          const int image_row = height - 1 - iy;

          for (int ix = 0; ix < width; ++ix) {
            const std::size_t image_index =
              static_cast<std::size_t>(image_row) *
              static_cast<std::size_t>(width) +
              static_cast<std::size_t>(ix);
            const std::size_t occ_index =
              static_cast<std::size_t>(iy) *
              static_cast<std::size_t>(width) +
              static_cast<std::size_t>(ix);

            if (imported_map_pixels_[image_index] == 0) {
              occupancy[occ_index] = 1;
            }
          }
        }
      }
    }

    // 车体自身点同样要从基准里扣除: 否则锁定瞬间车体占据的格会成为
    // 基准的"静态环境", 之后车开走、该处出现真实障碍时会被误判为静态
    clearVehicleFootprint(occupancy, config, width, height);

    // 基准必须膨胀 (与主图/历史图渲染保持一致的保守度)。
    //   若不膨胀, 障碍靠近时其点云会落在"锁定前静态物体表面边缘处尚未被
    //   标记的空隙"上, 被误判成动态障碍是小事; 更常见的是反过来 ——
    //   障碍与静态物表面重叠/紧贴 (如推到桌边), 其点落在基准已占据格上,
    //   被判为静态 -> 近距离时绿色消失。膨胀让基准更宽, 保证"贴边障碍"
    //   仍能被识别为动态。膨胀量用 baseline_dilate (默认同 dilate)。
    const int baseline_dilate =
      config.baseline_dilate >= 0 ? config.baseline_dilate : config.dilate;

    if (baseline_dilate > 0) {
      Config baseline_config = config;
      baseline_config.dilate = baseline_dilate;
      applyDilate(
        baseline_config, width, height, occupancy, filtered_points);
      // 膨胀后补扣: 保证基准图中车体自身位置也是空的
      clearVehicleFootprint(occupancy, config, width, height);
    }

    std::size_t baseline_hits = 0;

    for (const auto value : occupancy) {
      if (value != 0) {
        ++baseline_hits;
      }
    }

    // ---- 配准评分容差掩码 ----
    // 在原基准占据掩码上再膨胀 1 格, 得到"容差 = 基准膨胀(dilate) + 1 格"的评分掩码。
    //   原实现按"点恰好落在占据格才得分"评估, 容差只有 ±1 格 (0.03 m 网格下即
    //   ±0.03 m), 而搜索步长是 0.2 m(局部)/1.0 m(全局) —— 采样点几乎全部错过基准,
    //   最优分数退化为噪声, 修正量在粗格点间乱跳 (锁定后点云乱飘的根源之一)。
    //   这里一次性预膨胀, 评分仍是单次查表, 不增加每候选点的开销。
    std::vector<uint8_t> tolerance(occupancy.size(), 0);
    const std::size_t width_sz = static_cast<std::size_t>(width);
    const std::size_t height_sz = static_cast<std::size_t>(height);

    for (std::size_t iy = 0; iy < height_sz; ++iy) {
      for (std::size_t ix = 0; ix < width_sz; ++ix) {
        bool hit = false;

        for (int dy = -1; dy <= 1 && !hit; ++dy) {
          const long long ny = static_cast<long long>(iy) + dy;

          if (ny < 0 || ny >= static_cast<long long>(height_sz)) {
            continue;
          }

          for (int dx = -1; dx <= 1 && !hit; ++dx) {
            const long long nx = static_cast<long long>(ix) + dx;

            if (nx < 0 || nx >= static_cast<long long>(width_sz)) {
              continue;
            }

            if (occupancy[
                static_cast<std::size_t>(ny) * width_sz +
                static_cast<std::size_t>(nx)] != 0)
            {
              hit = true;
            }
          }
        }

        tolerance[iy * width_sz + ix] = hit ? 1 : 0;
      }
    }

    {
      std::lock_guard<std::mutex> lock(dynamic_mutex_);
      static_obstacle_mask_ = std::move(occupancy);
      static_obstacle_tolerance_ = std::move(tolerance);
    }

    RCLCPP_INFO(
      get_logger(),
      "静态基准图已建立: %d x %d 栅格 (%.2f m/格), 历史帧 %zu, "
      "基准占据格 %zu; 此后新出现的障碍将判为动态障碍(绿色)",
      width,
      height,
      config.resolution,
      history_copy.size(),
      baseline_hits);
  }

```

## `tasks/pose_correction.cpp`

建议落点：class PoseCorrector

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `getPoseCorrection` | 1335-1339 | 函数 | 加锁拷贝当前 map->odom 修正量 |
| `setPoseCorrection2D` | 1362-1378 | 函数 | (x,y,yaw) -> SE(3) 写入成员并广播 |
| `resetPoseCorrection` | 1380-1396 | 函数 | 置恒等修正; 仅 apply_pose_correction_ 为真时广播 |
| `estimatePoseCorrection` | 1411-1688 | 函数 | 2D 栅格配准: 粗搜(±range/步长) + 亚格细化 + 单帧速率限幅 + 滞回/EMA/死区; 失败计数达阈值触发全局重搜; 成功则 setPoseCorrection2D |
| `correctCloudForPose` | 1690-1713 | 函数 | 按 apply_pose_correction_ 二选一(自估 / 外部修正)后把整帧点云变换到 map 系 |

```cpp
// ===================== 源码行 1335-1339 · getPoseCorrection =====================
  PoseCorrection getPoseCorrection()
  {
    std::lock_guard<std::mutex> lock(pose_correction_mutex_);
    return pose_correction_;
  }

// ===================== 源码行 1362-1378 · setPoseCorrection2D =====================
  void setPoseCorrection2D(double x, double y, double yaw)
  {
    PoseCorrection c;
    const double cs = std::cos(yaw);
    const double sn = std::sin(yaw);
    c.r[0] = cs; c.r[1] = -sn; c.r[2] = 0.0;
    c.r[3] = sn; c.r[4] = cs;  c.r[5] = 0.0;
    c.r[6] = 0.0; c.r[7] = 0.0; c.r[8] = 1.0;
    c.t[0] = x; c.t[1] = y; c.t[2] = 0.0;
    c.valid = true;

    {
      std::lock_guard<std::mutex> lock(pose_correction_mutex_);
      pose_correction_ = c;
    }
    publishPoseCorrection(c);
  }

// ===================== 源码行 1380-1396 · resetPoseCorrection =====================
  void resetPoseCorrection()
  {
    PoseCorrection c;
    c.valid = true;

    {
      std::lock_guard<std::mutex> lock(pose_correction_mutex_);
      pose_correction_ = c;
    }

    // 本节点是修正量的发布者时, 广播恒等变换让桥接/网页归零;
    //   外部修正模式 (apply_pose_correction=false) 下 /pose_correction 由外部
    //   节点独占发布, 本节点只更新内部状态, 不发消息避免双源打架。
    if (apply_pose_correction_) {
      publishPoseCorrection(c);
    }
  }

// ===================== 源码行 1411-1688 · estimatePoseCorrection =====================
  bool estimatePoseCorrection(const std::vector<Point> & cloud)
  {
    if (!dynamic_obstacle_enabled_.load()) {
      return false;
    }

    Config config;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config = config_;
    }

    std::vector<uint8_t> baseline;
    {
      std::lock_guard<std::mutex> lock(dynamic_mutex_);
      baseline = static_obstacle_mask_;
    }
    if (baseline.empty()) {
      return false;
    }

    int width = 0;
    int height = 0;
    if (!computeMapSize(config, width, height)) {
      return false;
    }

    std::vector<Point> samples;
    samples.reserve(300);
    const std::size_t stride = std::max<std::size_t>(
      1, cloud.size() / 300);
    for (std::size_t i = 0; i < cloud.size(); i += stride) {
      const auto & point = cloud[i];
      if (std::isfinite(point.x) && std::isfinite(point.y)) {
        samples.push_back(point);
      }
    }
    if (samples.size() < 40) {
      return false;
    }

    // 评分容差掩码 (基准膨胀 + 1 格, 见 buildStaticBaseline); 缺失时退化为基准本身
    std::vector<uint8_t> score_mask;
    {
      std::lock_guard<std::mutex> lock(dynamic_mutex_);
      score_mask = static_obstacle_tolerance_;
    }

    if (score_mask.size() != baseline.size()) {
      score_mask = baseline;
    }

    const int width_i = width;
    const int height_i = height;

    // 单次位姿评分 (命中率): 出界点计入分母(视为未命中), 与原口径一致。
    //   出界点算未命中, 等价于要求整片点云留在栅格内, 从评分上排除
    //   "平移到空旷处得高分"的错误解。
    auto score_at = [&](double x, double y, double yaw, double & out_score) {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        std::size_t hits = 0;
        std::size_t considered = 0;

        for (const auto & point : samples) {
          const double tx = c * point.x - s * point.y + x;
          const double ty = s * point.x + c * point.y + y;
          ++considered;
          const int ix = static_cast<int>(
            std::floor((tx - config.xmin) / config.resolution));
          const int iy = static_cast<int>(
            std::floor((ty - config.ymin) / config.resolution));

          if (ix < 0 || ix >= width_i || iy < 0 || iy >= height_i) {
            continue;
          }

          const std::size_t index =
            static_cast<std::size_t>(iy) *
            static_cast<std::size_t>(width_i) +
            static_cast<std::size_t>(ix);

          if (score_mask[index] != 0) {
            ++hits;
          }
        }

        if (considered < 40) {
          return false;
        }

        out_score = static_cast<double>(hits) /
          static_cast<double>(considered);
        return true;
      };

    // 2D 搜索以上一次修正量的 XY/yaw 为起点
    const PoseCorrection previous_correction = getPoseCorrection();
    const double previous_x = previous_correction.t[0];
    const double previous_y = previous_correction.t[1];
    const double previous_yaw = yawFromMatrix(previous_correction.r);

    const double now_sec = get_clock()->now().seconds();

    // ---- 全局搜索降级为兜底: 仅当局部搜索连续失败若干帧才触发 ----
    //   原实现每隔 global_period 无条件做 ±15 m/±180° 全搜。稀疏/自相似的基准图上
    //   极易锁到错误的局部极大值, 一旦锁错, 后续局部搜索会围着错值打转 ->
    //   表现为绿色点云持续偏离黑色底图。
    const bool global_period_ok =
      now_sec - pose_correction_last_global_sec_.load() >=
      pose_correction_global_period_sec_;
    const bool global_search =
      global_period_ok &&
      pose_correction_local_fail_count_.load() >=
      pose_correction_global_trigger_fails_;
    const double translation_range = global_search ?
      pose_correction_global_range_m_ : pose_correction_local_range_m_;
    const double translation_step = global_search ? 1.0 : 0.2;
    const double yaw_range = global_search ? M_PI : (15.0 * M_PI / 180.0);
    const double yaw_step = global_search ?
      (15.0 * M_PI / 180.0) : (5.0 * M_PI / 180.0);

    double best_score = -1.0;
    double best_x = previous_x;
    double best_y = previous_y;
    double best_yaw = previous_yaw;

    // ---- 粗搜 (原栅格) ----
    for (double yaw = previous_yaw - yaw_range;
      yaw <= previous_yaw + yaw_range + 1e-9; yaw += yaw_step)
    {
      for (double x = previous_x - translation_range;
        x <= previous_x + translation_range + 1e-9; x += translation_step)
      {
        for (double y = previous_y - translation_range;
          y <= previous_y + translation_range + 1e-9; y += translation_step)
        {
          double score = 0.0;

          if (!score_at(x, y, yaw, score)) {
            continue;
          }

          if (score > best_score) {
            best_score = score;
            best_x = x;
            best_y = y;
            best_yaw = yaw;
          }
        }
      }
    }

    // ---- 亚格细化: 在粗解邻域内用 1 格/1° 再搜一遍 ----
    //   粗步长 (0.2 m/1.0 m) 远大于栅格 (0.03~0.05 m), 粗解被量化到粗格点上;
    //   细化把修正量拉到格级精度, 消除"在粗格点之间来回跳"的观感, 且不增加
    //   全局粗搜的候选数 (性能不变)。
    if (best_score >= 0.0) {
      const double coarse_x = best_x;
      const double coarse_y = best_y;
      const double coarse_yaw = best_yaw;
      // 细化范围取 ±5 格: 必须覆盖粗搜步长(0.2 m)带来的最大半格误差(0.1 m),
      //   否则 0.03 m 网格下 ±3 格(0.09 m)可能够不到真值, 细化也就无从收敛。
      const double fine_range = 5.0 * config.resolution;
      const double fine_step = config.resolution;
      const double fine_yaw_range = 5.0 * M_PI / 180.0;
      const double fine_yaw_step = 1.0 * M_PI / 180.0;

      for (double yaw = coarse_yaw - fine_yaw_range;
        yaw <= coarse_yaw + fine_yaw_range + 1e-9; yaw += fine_yaw_step)
      {
        for (double x = coarse_x - fine_range;
          x <= coarse_x + fine_range + 1e-9; x += fine_step)
        {
          for (double y = coarse_y - fine_range;
            y <= coarse_y + fine_range + 1e-9; y += fine_step)
          {
            double score = 0.0;

            if (!score_at(x, y, yaw, score)) {
              continue;
            }

            if (score > best_score) {
              best_score = score;
              best_x = x;
              best_y = y;
              best_yaw = yaw;
            }
          }
        }
      }
    }

    if (best_score < pose_correction_score_min_) {
      const int fails = pose_correction_local_fail_count_.fetch_add(1) + 1;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "冻结历史图配准失败: score=%.3f < %.3f, 沿用上次校正值 "
        "(连续失败 %d 帧, 达 %d 帧触发全局兜底搜索)",
        best_score, pose_correction_score_min_, fails,
        pose_correction_global_trigger_fails_);
      return false;
    }

    // 单帧校正变化上限 —— 改为"速率限制"而非硬拒绝。
    //   原实现: 变化超过上限就 return false (拒绝并沿用上次值)。问题是当前修正量
    //   一旦落后于真值 (或停在评分平台区), 合法的大修正会被同一条规则永久拒绝 ->
    //   修正量冻结、点云一直偏离, 且不再发布 /pose_correction (网页/桥接失去输入)。
    //   现改为: 每帧最多朝目标移动 max_delta, 既抑制突跳, 又允许连续追赶。
    const double delta = std::hypot(best_x - previous_x, best_y - previous_y);

    if (pose_correction_max_delta_m_ > 0.0 &&
      delta > pose_correction_max_delta_m_)
    {
      const double scale = pose_correction_max_delta_m_ / delta;
      best_x = previous_x + (best_x - previous_x) * scale;
      best_y = previous_y + (best_y - previous_y) * scale;
      best_yaw = wrapAngle(
        previous_yaw + wrapAngle(best_yaw - previous_yaw) * scale);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "校正变化=%.2f m 超过单帧上限, 按 %.2f m/帧 速率逼近",
        delta, pose_correction_max_delta_m_);
    }

    // ---- 时序滤波: 滞回 + EMA (治"乱飘"的关键) ----
    //   原实现把单帧最优解直接落盘, 估计噪声 1:1 变成整帧点云的位移。
    //   滞回: 新解须比"当前解"评分高出一档才接受, 否则保持不动 (抑制抖动);
    //   EMA : 接受后按 alpha 平滑过渡, 避免一帧跳到新值;
    //   死区: 半格以内的调整不落盘, 抑制亚格噪声引起的可见抖动。
    double current_score = 0.0;
    const bool current_scored =
      score_at(previous_x, previous_y, previous_yaw, current_score);

    double target_x = best_x;
    double target_y = best_y;
    double target_yaw = best_yaw;

    if (current_scored &&
      best_score < current_score + pose_correction_score_gain_)
    {
      target_x = previous_x;
      target_y = previous_y;
      target_yaw = previous_yaw;
    }

    const double alpha =
      std::min(1.0, std::max(0.05, pose_correction_ema_alpha_));
    const double smooth_x = previous_x + alpha * (target_x - previous_x);
    const double smooth_y = previous_y + alpha * (target_y - previous_y);
    const double smooth_yaw =
      wrapAngle(previous_yaw + alpha * wrapAngle(target_yaw - previous_yaw));

    double apply_x = smooth_x;
    double apply_y = smooth_y;

    if (std::hypot(apply_x - previous_x, apply_y - previous_y) <
      0.5 * config.resolution)
    {
      apply_x = previous_x;
      apply_y = previous_y;
    }

    setPoseCorrection2D(apply_x, apply_y, smooth_yaw);
    pose_correction_local_fail_count_.store(0);

    if (global_search) {
      pose_correction_last_global_sec_.store(now_sec);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "冻结历史图配准成功: score=%.3f (当前解 %.3f), "
      "map->odom correction=(%.2f, %.2f, %.1f deg)",
      best_score, current_score, apply_x, apply_y, smooth_yaw * 180.0 / M_PI);
    return true;
  }

// ===================== 源码行 1690-1713 · correctCloudForPose =====================
  void correctCloudForPose(std::vector<Point> & cloud)
  {
    if (!dynamic_obstacle_enabled_.load()) {
      return;
    }

    // 修正来源二选一:
    //   apply_pose_correction=true : 本节点自带的 2D 栅格配准估计并发布;
    //   apply_pose_correction=false: 使用订阅到的外部修正量 (如 reloc_3d GICP),
    //     本节点不估计、不发布, 只把外部修正施加到点云 (动态障碍检测/绿层/
    //     Nav2 点云全部基于修正后的点云, 与底图保持一致)。
    if (apply_pose_correction_) {
      estimatePoseCorrection(cloud);
    }

    const PoseCorrection correction = getPoseCorrection();
    if (!correction.valid) {
      return;
    }

    for (auto & point : cloud) {
      transformPointByCorrection(point, correction);
    }
  }

```

## `tasks/map_render.cpp`

建议落点：class MapRenderer

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `renderPgm` | 6554-6655 | 函数 | 单帧建图流程: 算尺寸 -> 累积点云 -> 扣车体 -> 膨胀 -> 转像素 -> 发 Image -> 写 PGM 文件 |
| `renderHistoryPgm` | 6659-6827 | 函数 | 历史多帧投票累积渲染 -> 占据比例暴涨检测(WARN) -> 发 Image -> 写 PGM |
| `renderDynamicObstaclePgm` | 7279-7322 | 函数 | 动态障碍栅格 -> 绿色层 Image 发布 |

```cpp
// ===================== 源码行 6554-6655 · renderPgm =====================
  void renderPgm(
    const std::vector<Point> & cloud,
    const Config & config)
  {
    int width = 0;
    int height = 0;

    if (!computeMapSize(config, width, height)) {
      return;
    }

    std::vector<uint8_t> occupancy(
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(height), 0);

    std::size_t filtered_points = 0;

    accumulateCloud(cloud, config, width, height, occupancy, filtered_points);
    clearVehicleFootprint(occupancy, config, width, height);
    applyDilate(config, width, height, occupancy, filtered_points);
    // 膨胀会让方框外的障碍反向盖回方框内部边缘, 故膨胀后再扣除一次,
    // 保证车体自身位置在实时图上也是绝对干净的
    clearVehicleFootprint(occupancy, config, width, height);

    std::vector<uint8_t> pixels;

    occupancyToPixels(occupancy, width, height, pixels);

    // 有订阅者时发布图像（mono8: 0=占据, 205=未知）
    if (image_publisher_ &&
      image_publisher_->get_subscription_count() > 0)
    {
      sensor_msgs::msg::Image image_msg;
      image_msg.header.stamp = get_clock()->now();
      image_msg.header.frame_id = "map";
      image_msg.height = static_cast<uint32_t>(height);
      image_msg.width = static_cast<uint32_t>(width);
      image_msg.encoding = "mono8";
      image_msg.is_bigendian = 0;
      image_msg.step = static_cast<uint32_t>(width);
      image_msg.data = pixels;
      image_publisher_->publish(image_msg);
    }

    const std::string temporary_file =
      config.output + ".tmp";

    {
      std::ofstream file(
        temporary_file,
        std::ios::binary | std::ios::trunc);

      if (!file) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "无法创建 PGM 临时文件: %s",
          temporary_file.c_str());
        return;
      }

      file << "P5\n"
           << width << " " << height << "\n"
           << "255\n";

      file.write(
        reinterpret_cast<const char *>(pixels.data()),
        static_cast<std::streamsize>(pixels.size()));

      if (!file.good()) {
        RCLCPP_ERROR(
          get_logger(),
          "写入 PGM 失败: %s",
          temporary_file.c_str());
        return;
      }
    }

    if (std::rename(
        temporary_file.c_str(),
        config.output.c_str()) != 0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "无法替换 PGM 文件: %s",
        config.output.c_str());
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "PGM 已更新: %s, 点数=%zu, 栅格=%dx%d",
      config.output.c_str(),
      filtered_points,
      width,
      height);
  }

// ===================== 源码行 6659-6827 · renderHistoryPgm =====================
  void renderHistoryPgm(
    const std::deque<TimedCloud> & history,
    const Config & config)
  {
    // 清空历史(重建)后 history 为空: 仍渲染并发布一张"全未知"的空白图,
    // 让网页刷新按钮的效果立即可见, 随后随新点云逐帧重新累积。
    std::vector<const std::vector<Point> *> clouds;
    clouds.reserve(history.size());

    for (const auto & timed_cloud : history) {
      clouds.push_back(timed_cloud.points.get());
    }

    int width = 0;
    int height = 0;

    if (!computeMapSize(config, width, height)) {
      return;
    }

    std::vector<uint8_t> occupancy(
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(height), 0);
    // 命中帧数计数器: uint8 饱和计数(单帧每格最多计一次),
    // 历史窗口内的命中次数上限远小于 255
    std::vector<uint8_t> hit_counts(occupancy.size(), 0);

    std::size_t filtered_points = 0;

    for (const auto * cloud : clouds) {
      accumulateCloud(
        *cloud, config, width, height, occupancy, filtered_points, &hit_counts);
    }

    const uint8_t min_hits = static_cast<uint8_t>(
      std::min(255, std::max(1, config.history_min_hits)));

    for (std::size_t index = 0; index < occupancy.size(); ++index) {
      occupancy[index] = hit_counts[index] >= min_hits ? 1 : 0;
    }

    // 车体自身点自过滤: 按当前车体中心位置扣除 self_filter_size 见方内的占据格。
    //   必须放在"投票判定"之后: 投票判定会用 hit_counts 覆写 occupancy, 若在
    //   此之前扣除会被整体冲掉 (车体曾经过的格子票数早已累积, 历史图上残留)。
    clearVehicleFootprint(occupancy, config, width, height);

    // 膨胀: 会把方框四周的障碍向外扩散, 也会反向盖回方框内部边缘。
    //   方框内是"车体自身位置", 无论任何情况下都不应有障碍, 故膨胀之后再扣除
    //   一次, 保证方框内绝对干净 (对历史累计图尤其重要: 否则残留会永久烧进地图)。
    applyDilate(config, width, height, occupancy, filtered_points);
    clearVehicleFootprint(occupancy, config, width, height);

    // ---- 建图统计: 占据格占比 ----
    //   "雷达脑乱"时最直观的表现就是占据格占比突然暴涨; 这里每帧统计, 占比相对
    //   上帧暴涨时打 WARN, 便于事后把时间点与逐帧日志/PCD 快照对上。
    {
      const std::size_t occupied = static_cast<std::size_t>(
        std::count(occupancy.begin(), occupancy.end(), uint8_t{1}));
      const double ratio =
        occupancy.empty() ? 0.0 :
        static_cast<double>(occupied) /
        static_cast<double>(occupancy.size());

      if (debug_log_enabled_ && debug_prev_occ_valid_ &&
        ratio - debug_prev_occ_ratio_ > debug_occ_ratio_jump_)
      {
        RCLCPP_WARN(
          get_logger(),
          "历史图占据比例暴涨: %.1f%% -> %.1f%% (帧数=%zu, 点数=%zu, "
          "栅格=%dx%d, 疑似雷达/LIO 异常)",
          debug_prev_occ_ratio_ * 100.0, ratio * 100.0,
          history.size(), filtered_points, width, height);
      }

      debug_prev_occ_ratio_ = ratio;
      debug_prev_occ_valid_ = true;

      RCLCPP_DEBUG(
        get_logger(),
        "历史图占据统计: %.2f%% (%zu/%zu 格), 帧数=%zu, 点数=%zu",
        ratio * 100.0, occupied, occupancy.size(),
        history.size(), filtered_points);
    }

    std::vector<uint8_t> pixels;

    occupancyToPixels(occupancy, width, height, pixels);

    // 若存在导入的成品图, 把它的占据格叠加到底图上 (两者尺寸一致时), 使导入的
    //   静态环境与当前实时累积历史图共存: 导入图不会随新点云消失, 实时点云
    //   继续在其上累积。尺寸不一致时仅发布实时历史图, 避免错位。
    mergeImportedMap(pixels, width, height);

    // 有订阅者时发布历史地图图像（mono8: 0=占据, 205=未知）
    if (history_image_publisher_ &&
      history_image_publisher_->get_subscription_count() > 0)
    {
      sensor_msgs::msg::Image image_msg;
      image_msg.header.stamp = get_clock()->now();
      image_msg.header.frame_id = "map";
      image_msg.height = static_cast<uint32_t>(height);
      image_msg.width = static_cast<uint32_t>(width);
      image_msg.encoding = "mono8";
      image_msg.is_bigendian = 0;
      image_msg.step = static_cast<uint32_t>(width);
      image_msg.data = pixels;
      history_image_publisher_->publish(image_msg);
    }

    const std::string temporary_file =
      config.history_output + ".tmp";

    {
      std::ofstream file(
        temporary_file,
        std::ios::binary | std::ios::trunc);

      if (!file) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "无法创建历史 PGM 临时文件: %s",
          temporary_file.c_str());
        return;
      }

      file << "P5\n"
           << width << " " << height << "\n"
           << "255\n";

      file.write(
        reinterpret_cast<const char *>(pixels.data()),
        static_cast<std::streamsize>(pixels.size()));

      if (!file.good()) {
        RCLCPP_ERROR(
          get_logger(),
          "写入历史 PGM 失败: %s",
          temporary_file.c_str());
        return;
      }
    }

    if (std::rename(
        temporary_file.c_str(),
        config.history_output.c_str()) != 0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "无法替换历史 PGM 文件: %s",
        config.history_output.c_str());
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "历史地图已更新: %s, 保留 %.1f s, 帧数=%zu, 点数=%zu, 栅格=%dx%d",
      config.history_output.c_str(),
      config.history_duration,
      history.size(),
      filtered_points,
      width,
      height);
  }

// ===================== 源码行 7279-7322 · renderDynamicObstaclePgm =====================
  void renderDynamicObstaclePgm(
    const std::vector<Point> & dynamic_points,
    const Config & config)
  {
    int width = 0;
    int height = 0;

    if (!computeMapSize(config, width, height)) {
      return;
    }

    std::vector<uint8_t> occupancy(
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(height), 0);

    std::size_t filtered_points = 0;

    accumulateCloud(
      dynamic_points, config, width, height, occupancy, filtered_points);
    // 膨胀量沿用主图设置: 让动态障碍在网页上可见、且给 Nav2 留出安全余量
    applyDilate(config, width, height, occupancy, filtered_points);
    // 车体方框扣除: 清除车体当前位置方框内的绿色格 (车下地面/自身残留)。
    //   放在膨胀之后: 膨胀会把方框外障碍反向盖回方框边缘, 后扣除可保证方框内绝对干净。
    clearVehicleFootprint(occupancy, config, width, height);

    std::vector<uint8_t> pixels;

    occupancyToPixels(occupancy, width, height, pixels);

    if (dynamic_obstacle_image_publisher_ &&
      dynamic_obstacle_image_publisher_->get_subscription_count() > 0)
    {
      sensor_msgs::msg::Image image_msg;
      image_msg.header.stamp = get_clock()->now();
      image_msg.header.frame_id = "map";
      image_msg.height = static_cast<uint32_t>(height);
      image_msg.width = static_cast<uint32_t>(width);
      image_msg.encoding = "mono8";
      image_msg.is_bigendian = 0;
      image_msg.step = static_cast<uint32_t>(width);
      image_msg.data = pixels;
      dynamic_obstacle_image_publisher_->publish(image_msg);
    }
  }

```

## `tasks/map_io.cpp`

建议落点：class MapIo

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `exportHistoryMap` | 7046-7155 | 函数 | 导出编排: 渲染历史图 + 写 PGM/YAML/BIN |
| `importHistoryMap` | 7158-7261 | 函数 | 导入编排: 读 PGM/BIN -> 重建 history_ 与 imported_map_, 清 Nav2 代价图 |
| `mergeImportedMap` | 7026-7043 | 函数 | 导入图占据格叠加到当前像素(仅覆盖未知格) |

```cpp
// ===================== 源码行 7046-7155 · exportHistoryMap =====================
  void exportHistoryMap()
  {
    const std::string dir = export_map_dir_;

    if (dir.empty()) {
      RCLCPP_ERROR(get_logger(), "导出目录为空, 请先设置 export_map_dir");
      return;
    }

    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    if (ec) {
      RCLCPP_ERROR(
        get_logger(), "创建导出目录失败: %s (%s)",
        dir.c_str(), ec.message().c_str());
      return;
    }

    // 拷贝历史点云队列 + 配置 (持锁), 避免边遍历边被写
    std::deque<TimedCloud> history_copy;
    Config config_copy;

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      history_copy = history_;
    }

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_copy = config_;
    }

    // ---- 1) 成品历史图 PGM ----
    int width = 0;
    int height = 0;

    if (!computeMapSize(config_copy, width, height)) {
      RCLCPP_ERROR(get_logger(), "导出失败: 地图尺寸非法");
      return;
    }

    std::vector<uint8_t> occupancy(
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(height), 0);
    std::vector<uint8_t> hit_counts(occupancy.size(), 0);
    std::size_t filtered_points = 0;

    for (const auto & timed : history_copy) {
      if (timed.points) {
        accumulateCloud(
          *timed.points, config_copy, width, height,
          occupancy, filtered_points, &hit_counts);
      }
    }

    const uint8_t min_hits = static_cast<uint8_t>(
      std::min(255, std::max(1, config_copy.history_min_hits)));

    for (std::size_t index = 0; index < occupancy.size(); ++index) {
      occupancy[index] = hit_counts[index] >= min_hits ? 1 : 0;
    }

    applyDilate(config_copy, width, height, occupancy, filtered_points);

    std::vector<uint8_t> pixels;
    occupancyToPixels(occupancy, width, height, pixels);

    // 与网页显示保持一致: 若已导入过成品图, 一并叠加导出, 使"导出->导入"链路
    //   不丢失之前导入的静态底图。
    mergeImportedMap(pixels, width, height);

    const std::string pgm_path = dir + "/map.pgm";
    const std::string yaml_path = dir + "/map.yaml";
    const std::string bin_path = dir + "/map_clouds.bin";

    {
      std::ofstream file(pgm_path, std::ios::binary | std::ios::trunc);

      if (!file) {
        RCLCPP_ERROR(get_logger(), "无法创建导出 PGM: %s", pgm_path.c_str());
        return;
      }

      file << "P5\n" << width << " " << height << "\n" << "255\n";
      file.write(
        reinterpret_cast<const char *>(pixels.data()),
        static_cast<std::streamsize>(pixels.size()));

      if (!file.good()) {
        RCLCPP_ERROR(get_logger(), "写入导出 PGM 失败: %s", pgm_path.c_str());
        return;
      }
    }

    // ---- 2) YAML 元数据 ----
    if (!writeMapYaml(yaml_path, config_copy, "map.pgm", width, height)) {
      return;
    }

    // ---- 3) 原始点云队列 ----
    if (!writeHistoryCloudsBin(history_copy, bin_path)) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "历史累计图已导出到 %s (栅格=%dx%d, 帧数=%zu, 点数=%zu)",
      dir.c_str(), width, height, history_copy.size(), filtered_points);
  }

// ===================== 源码行 7158-7261 · importHistoryMap =====================
  void importHistoryMap()
  {
    const std::string dir = import_map_dir_;

    if (dir.empty()) {
      RCLCPP_ERROR(get_logger(), "导入目录为空, 请先设置 import_map_dir");
      return;
    }

    Config config_copy;

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_copy = config_;
    }

    bool any_ok = false;

    // ---- 1) 成品图 PGM (作为导入底图; 尺寸若不匹配当前配置则按导入图尺寸显示) ----
    {
      const std::string pgm_path = dir + "/map.pgm";
      std::ifstream file(pgm_path, std::ios::binary);

      if (file) {
        std::string magic;
        int width = 0;
        int height = 0;
        int maxval = 0;
        file >> magic >> width >> height >> maxval;
        file.get();   // 消费头部后的单个空白字符

        if (magic == "P5" && width > 0 && height > 0 &&
          maxval > 0 && maxval <= 255)
        {
          std::vector<uint8_t> pixels(
            static_cast<std::size_t>(width) *
            static_cast<std::size_t>(height));

          file.read(
            reinterpret_cast<char *>(pixels.data()),
            static_cast<std::streamsize>(pixels.size()));

          if (file.good()) {
            std::lock_guard<std::mutex> lock(imported_map_mutex_);
            imported_map_pixels_ = std::move(pixels);
            imported_map_width_ = width;
            imported_map_height_ = height;
            imported_map_valid_ = true;
            any_ok = true;

            RCLCPP_INFO(
              get_logger(),
              "已导入成品历史图: %s (栅格=%dx%d)",
              pgm_path.c_str(), width, height);
          } else {
            RCLCPP_ERROR(
              get_logger(), "导入 PGM 数据读取不完整: %s", pgm_path.c_str());
          }
        } else {
          RCLCPP_ERROR(
            get_logger(), "导入 PGM 头部非法: %s", pgm_path.c_str());
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "未找到成品图 %s, 跳过 (仅导入点云队列)",
          pgm_path.c_str());
      }
    }

    // ---- 2) 原始点云队列 -> 重建 history_ ----
    {
      const std::string bin_path = dir + "/map_clouds.bin";
      std::deque<TimedCloud> loaded;

      if (readHistoryCloudsBin(bin_path, loaded)) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        history_ = std::move(loaded);
        pruneHistory(get_clock()->now());
        any_ok = true;

        RCLCPP_INFO(
          get_logger(),
          "已导入历史点云队列: %s (帧数=%zu, 后续将继续累积新点云)",
          bin_path.c_str(), history_.size());
      } else {
        RCLCPP_WARN(
          get_logger(), "未找到/无法解析点云队列 %s, 跳过",
          bin_path.c_str());
      }
    }

    if (any_ok) {
      // 导入后重建动态障碍静态基准 (若已处于锁定态), 并清空 Nav2 红区,
      // 避免旧地图的红色代价区残留。
      if (dynamic_obstacle_enabled_.load()) {
        buildStaticBaseline();
      }
      clearCostmaps();
      RCLCPP_INFO(get_logger(), "历史累计图导入完成");
    } else {
      RCLCPP_ERROR(get_logger(), "历史累计图导入失败: 目录中无有效文件 (%s)",
        dir.c_str());
    }
  }

// ===================== 源码行 7026-7043 · mergeImportedMap =====================
  void mergeImportedMap(
    std::vector<uint8_t> & pixels, int width, int height)
  {
    std::lock_guard<std::mutex> lock(imported_map_mutex_);

    if (!imported_map_valid_ ||
      imported_map_width_ != width || imported_map_height_ != height ||
      imported_map_pixels_.size() != pixels.size())
    {
      return;
    }

    for (std::size_t index = 0; index < pixels.size(); ++index) {
      if (imported_map_pixels_[index] == 0 && pixels[index] == 205) {
        pixels[index] = 0;
      }
    }
  }

```

## `tasks/astar_planner.cpp`

建议落点：class AstarPlanner

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `buildMergedOccupancy` | 4154-4214 | 函数 | 合并占据栅格 = 历史累计 ∪ 动态障碍(同一 accumulateCloud/applyDilate 口径), 再扣车体方框; A* / 占点 / 追击共用 |
| `planAstarPath` | 4239-4265 | 函数 | A* 入口: 把雷达位姿反推为车体中心, 以 Web 目标 goal_x_/goal_y_ 为终点调用 planAstarToGoal |
| `planAstarToGoal` | 4273-4602 | 函数 | A* 核心: 建图 -> 膨胀 -> 障碍代价场 -> 八连通搜索 -> 回溯 -> 弧长抽稀 |
| `getAstarPath` | 4612-4669 | 函数 | 带限频的规划入口: 按时间或位移判断是否重规划, 否则复用缓存 |

```cpp
// ===================== 源码行 4154-4214 · buildMergedOccupancy =====================
  void buildMergedOccupancy(
    const Config & config,
    int width,
    int height,
    std::vector<uint8_t> & occupancy)
  {
    const std::size_t grid_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

    occupancy.assign(grid_size, 0);

    std::deque<TimedCloud> history_copy;
    std::vector<Point> dynamic_copy;

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      history_copy = history_;
      dynamic_copy = latest_dynamic_obstacles_;
    }

    // 历史累计栅格: 与 renderHistoryPgm 同一套判定 (命中帧数 >= history_min_hits)
    {
      std::vector<uint8_t> history_occ(grid_size, 0);
      std::vector<uint8_t> hit_counts(grid_size, 0);
      std::size_t filtered_points = 0;

      for (const auto & timed_cloud : history_copy) {
        accumulateCloud(
          *timed_cloud.points, config, width, height, history_occ,
          filtered_points, &hit_counts);
      }

      const uint8_t min_hits = static_cast<uint8_t>(
        std::min(255, std::max(1, config.history_min_hits)));

      for (std::size_t i = 0; i < grid_size; ++i) {
        if (hit_counts[i] >= min_hits) {
          occupancy[i] = 1;
        }
      }
    }

    // 绿色动态障碍栅格: 与 renderDynamicObstaclePgm 同一套 (投影 + 膨胀)
    if (!dynamic_copy.empty()) {
      std::vector<uint8_t> dyn_occ(grid_size, 0);
      std::size_t filtered_points = 0;

      accumulateCloud(
        dynamic_copy, config, width, height, dyn_occ, filtered_points);
      applyDilate(config, width, height, dyn_occ, filtered_points);

      for (std::size_t i = 0; i < grid_size; ++i) {
        if (dyn_occ[i] != 0) {
          occupancy[i] = 1;
        }
      }
    }

    // 车体自身位置不应有障碍: 与其它渲染一致的方框扣除, 避免"自己挡自己"
    clearVehicleFootprint(occupancy, config, width, height);
  }

// ===================== 源码行 4239-4265 · planAstarPath =====================
  bool planAstarPath(
    const Config & config,
    int width,
    int height,
    double odom_x,
    double odom_y,
    double odom_yaw,
    std::vector<Point> & path)
  {
    path.clear();

    if (!astar_enabled_ || !goal_received_) {
      return false;
    }

    // 车体中心 (由雷达位姿 + 外参反推, 与 computeFollowCommand 步骤 0 同式)
    const double cos_yaw = std::cos(odom_yaw);
    const double sin_yaw = std::sin(odom_yaw);
    const double base_x = odom_x -
      (cos_yaw * motion_base_to_lidar_x_ - sin_yaw * motion_base_to_lidar_y_);
    const double base_y = odom_y -
      (sin_yaw * motion_base_to_lidar_x_ + cos_yaw * motion_base_to_lidar_y_);

    return planAstarToGoal(
      config, width, height, base_x, base_y,
      static_cast<double>(goal_x_), static_cast<double>(goal_y_), path);
  }

// ===================== 源码行 4273-4602 · planAstarToGoal =====================
  bool planAstarToGoal(
    const Config & config,
    int width,
    int height,
    double base_x,
    double base_y,
    double goal_wx,
    double goal_wy,
    std::vector<Point> & path)
  {
    path.clear();

    if (!astar_enabled_) {
      return false;
    }

    const double resolution = config.resolution;
    if (resolution <= 0.0) {
      return false;
    }

    // 网格坐标换算 (与 accumulateCloud / 地图渲染一致: 越界视为地图外)
    const auto world_to_cell = [&](double wx, double wy, int & ix, int & iy)
      {
        ix = static_cast<int>(std::floor((wx - config.xmin) / resolution));
        iy = static_cast<int>(std::floor((wy - config.ymin) / resolution));
      };

    int sx = 0;
    int sy = 0;
    int gx = 0;
    int gy = 0;
    world_to_cell(base_x, base_y, sx, sy);
    world_to_cell(goal_wx, goal_wy, gx, gy);

    // 起点/终点越界: 无法规划 (终点在地图外 -> 返回 false, 调用方停车重试)
    if (sx < 0 || sx >= width || sy < 0 || sy >= height ||
      gx < 0 || gx >= width || gy < 0 || gy >= height)
    {
      return false;
    }

    const std::size_t grid_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    const auto index_of = [&](int ix, int iy) -> std::size_t
      {
        return static_cast<std::size_t>(iy) *
               static_cast<std::size_t>(width) +
               static_cast<std::size_t>(ix);
      };

    // -------- 1) 合并占据栅格 + 障碍膨胀 --------
    std::vector<uint8_t> occupancy;
    buildMergedOccupancy(config, width, height, occupancy);

    std::vector<uint8_t> blocked(grid_size, 0);
    {
      const int inflate_cells = static_cast<int>(
        std::ceil(std::max(0.0, astar_inflate_radius_) / resolution));

      for (int iy = 0; iy < height; ++iy) {
        for (int ix = 0; ix < width; ++ix) {
          if (occupancy[index_of(ix, iy)] == 0) {
            continue;
          }

          const int x0 = std::max(0, ix - inflate_cells);
          const int x1 = std::min(width - 1, ix + inflate_cells);
          const int y0 = std::max(0, iy - inflate_cells);
          const int y1 = std::min(height - 1, iy + inflate_cells);

          for (int jy = y0; jy <= y1; ++jy) {
            for (int jx = x0; jx <= x1; ++jx) {
              blocked[index_of(jx, jy)] = 1;
            }
          }
        }
      }
    }

    // 起点自身所在格即使被膨胀覆盖, 也强制视为可通行:
    //   车正处于障碍内/贴障碍时, 起点格必被 blocked 覆盖, 若不放行则永远无解。
    blocked[index_of(sx, sy)] = 0;

    // 终点若落在(膨胀后)障碍上 (网页点在了障碍/贴墙处), 就近找最接近的可行格,
    //   否则 A* 永远无法抵达终点 -> 兜底失效。以方形环向外搜索, 取最近可行格。
    if (blocked[index_of(gx, gy)] != 0) {
      bool found = false;
      const int max_ring = std::max(width, height);
      int found_gx = gx;
      int found_gy = gy;

      for (int ring = 1; ring <= max_ring && !found; ++ring) {
        double best_d2 = std::numeric_limits<double>::max();

        for (int dy = -ring; dy <= ring; ++dy) {
          for (int dx = -ring; dx <= ring; ++dx) {
            // 只检查该环的边界 (内部已在更小的 ring 检查过)
            if (std::abs(dx) != ring && std::abs(dy) != ring) {
              continue;
            }

            const int nx = gx + dx;
            const int ny = gy + dy;

            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
              continue;
            }

            if (blocked[index_of(nx, ny)] != 0) {
              continue;
            }

            const double d2 = static_cast<double>(dx) * dx +
              static_cast<double>(dy) * dy;

            if (d2 < best_d2) {
              best_d2 = d2;
              found_gx = nx;
              found_gy = ny;
              found = true;
            }
          }
        }

        if (found) {
          break;
        }
      }

      if (!found) {
        return false;   // 终点被完全封死, 返回 false 由调用方停车重试
      }

      gx = found_gx;
      gy = found_gy;
    }

    // -------- 2) 障碍邻近代价场 (distance to nearest obstacle) --------
    //   多源 BFS: 从所有障碍格出发向四邻扩散, 得到每格到最近障碍的曼哈顿距离,
    //   距离越小代价越高 -> A* 倾向远离障碍的宽阔通路。
    std::vector<float> obstacle_cost(grid_size, 0.0F);
    {
      const float penalty = static_cast<float>(std::max(0.0, astar_obstacle_penalty_));

      if (penalty > 0.0F && astar_cost_reach_ > 0.0) {
        // 代价影响范围 (软性避让范围): 直接由 astar_cost_reach (m) 换算为格数,
        //   该范围内越近障碍代价越高。<= 0 时不产生邻近代价 (纯最短路)。
        const int reach = std::max(
          1, static_cast<int>(
            std::ceil(astar_cost_reach_ / resolution)));

        std::vector<int> dist(grid_size, -1);
        std::vector<int> queue;
        queue.reserve(grid_size);

        for (std::size_t i = 0; i < grid_size; ++i) {
          if (occupancy[i] != 0) {
            dist[i] = 0;
            queue.push_back(static_cast<int>(i));
          }
        }

        for (std::size_t head = 0; head < queue.size(); ++head) {
          const int cur = queue[head];
          const int cx = cur % width;
          const int cy = cur / width;

          if (dist[cur] >= reach) {
            continue;
          }

          const int dx4[4] = {1, -1, 0, 0};
          const int dy4[4] = {0, 0, 1, -1};

          for (int k = 0; k < 4; ++k) {
            const int nx = cx + dx4[k];
            const int ny = cy + dy4[k];

            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
              continue;
            }

            const int ni = ny * width + nx;
            if (dist[ni] != -1) {
              continue;
            }

            dist[ni] = dist[cur] + 1;
            queue.push_back(ni);
          }
        }

        for (std::size_t i = 0; i < grid_size; ++i) {
          if (dist[i] > 0) {
            // 距离越近代价越高 (线性衰减), 距离 >= reach 时为 0
            const float ratio = 1.0F -
              static_cast<float>(dist[i]) / static_cast<float>(reach);
            obstacle_cost[i] = penalty * std::max(0.0F, ratio);
          }
        }
      }
    }

    // -------- 3) A* 搜索 (八连通) --------
    const double diag = std::sqrt(2.0);
    const auto heuristic = [&](int x, int y) -> double
      {
        const double dx = std::abs(x - gx);
        const double dy = std::abs(y - gy);
        // 八连通可采纳启发式: 对角优先 + 直线补足
        return (dx + dy) + (diag - 2.0) * std::min(dx, dy);
      };

    const double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(grid_size, kInf);
    std::vector<int> parent(grid_size, -1);
    std::vector<uint8_t> closed(grid_size, 0);

    // 优先队列: (f, index); std::greater 使其成为最小堆
    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>>
      open;

    const int start = sy * width + sx;
    g_score[start] = 0.0;
    open.push({heuristic(sx, sy), start});

    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    int goal_index = -1;

    while (!open.empty()) {
      const QueueItem top = open.top();
      open.pop();
      const int cur = top.second;

      if (closed[cur] != 0) {
        continue;   // 已扩展过 (惰性删除)
      }
      closed[cur] = 1;

      if (cur == gy * width + gx) {
        goal_index = cur;
        break;
      }

      const int cx = cur % width;
      const int cy = cur / width;

      for (int k = 0; k < 8; ++k) {
        const int nx = cx + dx8[k];
        const int ny = cy + dy8[k];

        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }

        if (blocked[index_of(nx, ny)] != 0) {
          continue;
        }

        // 对角移动时禁止穿越两个相邻障碍格构成的墙角 (防穿墙)
        if (dx8[k] != 0 && dy8[k] != 0) {
          if (blocked[index_of(cx + dx8[k], cy)] != 0 ||
            blocked[index_of(cx, cy + dy8[k])] != 0)
          {
            continue;
          }
        }

        const int ni = index_of(nx, ny);
        const double step_cost = (dx8[k] != 0 && dy8[k] != 0) ? diag : 1.0;
        // 障碍邻近代价: 栅格距离单位, 与 step_cost 同量纲
        const double penalized = step_cost +
          static_cast<double>(obstacle_cost[ni]);
        const double tentative = g_score[cur] + penalized;

        if (tentative < g_score[ni]) {
          g_score[ni] = tentative;
          parent[ni] = cur;
          open.push({tentative + heuristic(nx, ny), ni});
        }
      }
    }

    if (goal_index < 0) {
      return false;   // 终点不可达
    }

    // -------- 4) 回溯路径 (栅格索引 -> 世界坐标) --------
    std::vector<Point> cells;
    for (int cur = goal_index; cur != -1; cur = parent[cur]) {
      const int cx = cur % width;
      const int cy = cur / width;
      cells.push_back(
        Point{
          static_cast<float>(config.xmin + (cx + 0.5) * resolution),
          static_cast<float>(config.ymin + (cy + 0.5) * resolution),
          0.0F});
    }
    std::reverse(cells.begin(), cells.end());

    // 起点格用真实车体中心替换 (栅格中心与车体中心可能有半格偏差)
    cells.front() = Point{
      static_cast<float>(base_x), static_cast<float>(base_y), 0.0F};

    // -------- 5) 弧长抽稀 --------
    const double spacing = astar_path_spacing_;
    if (spacing > 0.0 && cells.size() > 2) {
      path.push_back(cells.front());
      double acc = 0.0;

      for (std::size_t i = 1; i < cells.size(); ++i) {
        acc += std::hypot(
          cells[i].x - cells[i - 1].x, cells[i].y - cells[i - 1].y);

        const bool is_last = (i + 1 == cells.size());
        if (acc >= spacing || is_last) {
          path.push_back(cells[i]);
          acc = 0.0;
        }
      }
    } else {
      path = cells;
    }

    return path.size() >= 2;
  }

// ===================== 源码行 4612-4669 · getAstarPath =====================
  bool getAstarPath(
    const Config & config,
    int width,
    int height,
    double odom_x,
    double odom_y,
    double odom_yaw,
    std::vector<Point> & path)
  {
    // 车体中心 (与 planAstarPath / computeFollowCommand 同式)
    const double cos_yaw = std::cos(odom_yaw);
    const double sin_yaw = std::sin(odom_yaw);
    const double base_x = odom_x -
      (cos_yaw * motion_base_to_lidar_x_ - sin_yaw * motion_base_to_lidar_y_);
    const double base_y = odom_y -
      (sin_yaw * motion_base_to_lidar_x_ + cos_yaw * motion_base_to_lidar_y_);

    bool need_replan = !astar_cache_valid_;

    if (astar_cache_valid_) {
      if (astar_replan_interval_ > 0.0) {
        const double age = (now() - astar_last_plan_time_).seconds();
        const double moved = std::hypot(
          base_x - astar_last_plan_x_, base_y - astar_last_plan_y_);

        if (age >= astar_replan_interval_ ||
          moved >= std::max(0.0, astar_replan_move_dist_))
        {
          need_replan = true;
        }
      } else {
        need_replan = true;   // 不限频: 每帧重算
      }
    }

    if (need_replan) {
      std::vector<Point> fresh;
      if (planAstarPath(
          config, width, height, odom_x, odom_y, odom_yaw, fresh))
      {
        astar_cached_path_ = fresh;
        astar_cache_valid_ = true;
      } else {
        astar_cached_path_.clear();
        astar_cache_valid_ = false;
      }
      astar_last_plan_time_ = now();
      astar_last_plan_x_ = base_x;
      astar_last_plan_y_ = base_y;
    }

    if (!astar_cache_valid_) {
      return false;
    }

    path = astar_cached_path_;
    return path.size() >= 2;
  }

```

## `tasks/hold_point.cpp`

建议落点：class HoldPointTask

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `computeHoldPointCommand` | 5017-5217 | 函数 | 占点主流程: 读参数 -> 建图 -> 聚簇 -> 仲裁最近威胁 -> 算站位点 -> 硬性间距自检 -> 跟随到站 -> 机动安全兜底 -> 记录状态 -> 发可视化 |

```cpp
// ===================== 源码行 5017-5217 · computeHoldPointCommand =====================
  bool computeHoldPointCommand(
    const Config & config,
    int width,
    int height,
    double odom_x, double odom_y, double odom_yaw,
    double & vx, double & vy)
  {
    double center_x = 0.0;
    double center_y = 0.0;
    double radius = 1.50;
    double speed = -1.0;
    double slow_radius_cfg = -1.0;
    bool enabled = false;

    {
      std::lock_guard<std::mutex> lock(hold_point_mutex_);
      enabled = hold_point_enabled_;
      center_x = hold_point_x_;
      center_y = hold_point_y_;
      radius = hold_point_radius_;
      speed = hold_point_speed_;
      slow_radius_cfg = hold_point_slow_radius_;
    }

    if (!enabled) {
      return false;
    }

    // 占点机动速度: hold_point_speed_ <= 0 时跟随 motion_target_speed_
    //   (与 Nav2 正常巡航速度保持一致), 否则用手动指定值。
    const double hold_cruise_speed =
      std::clamp((speed > 0.0) ? speed : motion_target_speed_,
      0.0, kMotionMaxSpeed);

    // 占点减速起始距离: 比常规导航更短 (响应更快), 避免因控制/地图更新频率低
    //   而"刹不住"冲过站位点; 同时保留一小段缓冲, 避免全速硬停。
    //   slow_radius_cfg < 0 -> 自动: 取常规减速距离的一半 (但保证 > 到点阈值);
    //   = 0 -> 关闭; > 0 -> 手动值。
    double hold_slow_radius = slow_radius_cfg;

    if (hold_slow_radius < 0.0) {
      const double normal = effectiveSlowRadius(
        motion_slow_radius_, hold_cruise_speed, motion_goal_tolerance_);
      hold_slow_radius = std::max(
        motion_goal_tolerance_ + 0.05, normal * 0.5);
    }

    // 车体中心 (雷达位姿 + 外参反推, 与 computeFollowCommand 同式)
    const double cos_yaw = std::cos(odom_yaw);
    const double sin_yaw = std::sin(odom_yaw);
    const double base_x = odom_x -
      (cos_yaw * motion_base_to_lidar_x_ - sin_yaw * motion_base_to_lidar_y_);
    const double base_y = odom_y -
      (sin_yaw * motion_base_to_lidar_x_ + cos_yaw * motion_base_to_lidar_y_);

    // ---- 1) 构建合并占据栅格 (与导航/A* 同源) ----
    std::vector<uint8_t> occupancy;

    buildMergedOccupancy(config, width, height, occupancy);

    // ---- 2) 提取范围内障碍簇 (不做任何速度/运动预测) ----
    std::vector<HoldObstacleCluster> obstacles =
      extractHoldObstacles(
      occupancy, config, width, height, center_x, center_y, radius);

    // ---- 3) 多目标仲裁: 取"到占点中心直线距离最近"的障碍优先拦截 ----
    //   完全基于当前几何位置, 不依赖任何移动预测; 距离最近的敌人威胁最大,
    //   优先抢占其与中心之间、尽可能贴近敌人的位置进行阻挡。
    bool threat_valid = false;
    HoldObstacleCluster threat;
    double threat_dist = std::numeric_limits<double>::max();

    for (const auto & cluster : obstacles) {
      const double dist = std::hypot(cluster.x - center_x, cluster.y - center_y);

      if (dist < threat_dist) {
        threat_dist = dist;
        threat = cluster;
        threat_valid = true;
      }
    }

    // ---- 4) 计算本帧期望到达点 ----
    double target_x = center_x;
    double target_y = center_y;
    bool blocking = false;
    double block_x = center_x;
    double block_y = center_y;

    if (threat_valid) {
      computeHoldBlockPose(
        threat.x, threat.y, center_x, center_y, block_x, block_y);

      // 站位点自检: 站位点在"障碍 -> 中心"连线上、距障碍 standoff_scale * 硬性
      //   间距处。校验口径改为"点到点": 站位点到"除目标敌人以外"的最近占据格距离
      //   必须 >= 硬性间距。目标敌人本体被豁免 (它就在站位点前方 standoff 处, 若
      //   也计入判定则必然判失败 -> 与站位点定义自相矛盾)。这样既保证不撞墙/不撞
      //   其他障碍, 又能让站位点贴到硬性间距极限、尽可能顶住敌人。
      const double exempt_radius =
        astar_inflate_radius_ + config.resolution * 1.5;

      if (isHoldPointClear(
          occupancy, config, width, height,
          block_x, block_y, astar_inflate_radius_,
          threat.x, threat.y, exempt_radius))
      {
        target_x = block_x;
        target_y = block_y;
        blocking = true;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "占点: 阻挡站位点 (%.2f, %.2f) 与旁观障碍间距不足 %.2f m, "
          "放弃本次阻挡, 退回占点中心驻守",
          block_x, block_y, astar_inflate_radius_);
      }
    }

    // ---- 5) 前往期望到达点 (用跟随算法驱动, 复用 computeFollowCommand) ----
    //   把"期望到达点"包装成两点路径, 复用现有的前视点 + 到达判定逻辑; 但关闭
    //   接近终点的梯形减速 (见下方 disable_slowing), 全程按巡航速度顶上去。
    std::vector<Point> hold_plan;
    hold_plan.push_back(
      Point{static_cast<float>(base_x), static_cast<float>(base_y), 0.0F});
    hold_plan.push_back(
      Point{static_cast<float>(target_x), static_cast<float>(target_y), 0.0F});

    const double dist_to_target = std::hypot(
      target_x - base_x, target_y - base_y);

    // 回位死区: 无威胁且已在中心附近时不微调, 避免原地抖动
    double deadband = 0.08;

    {
      std::lock_guard<std::mutex> lock(hold_point_mutex_);
      deadband = hold_point_recenter_deadband_;
    }

    vx = 0.0;
    vy = 0.0;

    if (!blocking && dist_to_target <= deadband) {
      // 已在占点中心驻守且无威胁: 静止 (下发 0 速, 由调用方发帧)
      vx = 0.0;
      vy = 0.0;
    } else {
      const double cruise = hold_cruise_speed;
      // 到点为"到达期望点", 阈值取 motion_goal_tolerance_ 与死区中较小者,
      //   保证阻挡能贴到站位点、回位能停进死区。
      //   第 8 参 hold_slow_radius: 占点用比常规更短的梯形减速 (默认常规的一半),
      //   既保留末段缓冲、避免因更新频率低而刹不住冲过站位点, 又不会像常规导航
      //   那样"早早开始磨蹭"而跟不上敌人。
      computeFollowCommand(
        hold_plan, odom_x, odom_y, odom_yaw, vx, vy, cruise, hold_slow_radius);
    }

    // ---- 6) 机动安全兜底: 若按此速度行进会撞上障碍, 直接急停 ----
    //   预判下一小步的位置 (按速度 * 一个控制周期), 用圆形口径检查硬性间距。
    //   这是"绝不碰撞"的最后一道保险: 上面所有几何计算若因地图更新滞后而失准,
    //   这里也会拦住。阻挡时豁免目标敌人本体 (站位点几何已保证与其保持硬性间距),
    //   否则会把"正常贴近敌人"误判为碰撞而抖动停车。
    {
      const double dt = 1.0 / std::max(1.0, motion_rate_hz_);
      const double nx = base_x + (vx * cos_yaw - vy * sin_yaw) * dt;
      const double ny = base_y + (vx * sin_yaw + vy * cos_yaw) * dt;

      const double exempt_radius =
        blocking ? (astar_inflate_radius_ + config.resolution * 1.5) : 0.0;
      const double obs_x = blocking ? threat.x : 0.0;
      const double obs_y = blocking ? threat.y : 0.0;

      if (!isHoldPointClearPath(
          occupancy, config, width, height,
          base_x, base_y, nx, ny, astar_inflate_radius_,
          obs_x, obs_y, exempt_radius))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "占点: 预判机动会侵入硬性间距 (%.2f m), 本帧停车以保证不碰撞",
          astar_inflate_radius_);
        vx = 0.0;
        vy = 0.0;
      }
    }

    // ---- 7) 记录仲裁结果供网页展示 ----
    {
      std::lock_guard<std::mutex> lock(hold_point_mutex_);
      hold_point_threat_valid_ = threat_valid;
      hold_point_threat_x_ = threat_valid ? threat.x : 0.0;
      hold_point_threat_y_ = threat_valid ? threat.y : 0.0;
      hold_point_threat_dist_ = threat_valid ? threat_dist : 0.0;
      hold_point_block_x_ = block_x;
      hold_point_block_y_ = block_y;
    }

    // ---- 8) 发布可视化 + 状态 (供网页绘制占点圈/连线/状态文字) ----
    publishHoldPointViz(center_x, center_y, target_x, target_y);

    return true;
  }

```

## `tasks/chase.cpp`

建议落点：class ChaseTask

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `computeChaseCommand` | 5371-5633 | 函数 | 追击主流程: 锁定/跟踪目标 -> 节流记录轨迹 -> 算环带站位点 -> 连线检查 -> A* 或直连跟随 -> 安全兜底 -> 记录 + 发布 |
| `recordTargetCloud` | 5638-5671 | 函数 | 把目标邻域(半径约 radius*1.5)内的 3D 点云连同时间戳追加到 chase_track_, 按时长淘汰 |

```cpp
// ===================== 源码行 5371-5633 · computeChaseCommand =====================
  bool computeChaseCommand(
    const Config & config,
    int width,
    int height,
    double odom_x, double odom_y, double odom_yaw,
    double & vx, double & vy)
  {
    bool enabled = false;
    double radius = 1.0;
    double deadband = 0.15;
    double speed = -1.0;
    double lock_radius = 1.0;
    double track_radius = 1.0;
    double pick_x = 0.0;
    double pick_y = 0.0;

    {
      std::lock_guard<std::mutex> lock(chase_mutex_);
      enabled = chase_enabled_;
      radius = chase_radius_;
      deadband = chase_deadband_;
      speed = chase_speed_;
      lock_radius = chase_lock_radius_;
      track_radius = chase_track_radius_;
      pick_x = chase_pick_x_;
      pick_y = chase_pick_y_;
    }

    if (!enabled) {
      return false;
    }

    // 追击巡航速度: chase_speed_ <= 0 时跟随 motion_target_speed_
    const double chase_cruise_speed =
      std::clamp((speed > 0.0) ? speed : motion_target_speed_,
      0.0, kMotionMaxSpeed);

    // 减速起始距离: 沿用占点口径 (常规的一半), 便于贴住环带
    const double chase_slow_radius = std::max(
      motion_goal_tolerance_ + 0.05,
      effectiveSlowRadius(
        motion_slow_radius_, chase_cruise_speed, motion_goal_tolerance_) * 0.5);

    // 车体中心 (雷达位姿 + 外参反推, 与 computeFollowCommand/占点同式)
    const double cos_yaw = std::cos(odom_yaw);
    const double sin_yaw = std::sin(odom_yaw);
    const double base_x = odom_x -
      (cos_yaw * motion_base_to_lidar_x_ - sin_yaw * motion_base_to_lidar_y_);
    const double base_y = odom_y -
      (sin_yaw * motion_base_to_lidar_x_ + cos_yaw * motion_base_to_lidar_y_);

    // ---- 1) 构建合并占据栅格 (与导航/A*/占点同源) ----
    std::vector<uint8_t> occupancy;
    buildMergedOccupancy(config, width, height, occupancy);

    // ---- 2) 目标定位 / 跟踪 ----
    bool target_valid = false;
    double target_x = 0.0;
    double target_y = 0.0;

    {
      std::lock_guard<std::mutex> lock(chase_mutex_);
      target_valid = chase_target_valid_;
      target_x = chase_target_x_;
      target_y = chase_target_y_;
    }

    if (!target_valid) {
      // 尚未锁定: 以点选点为中心就近锁定一个障碍簇
      double fx = 0.0;
      double fy = 0.0;

      if (findNearestObstacleCluster(
          occupancy, config, width, height, pick_x, pick_y, lock_radius, fx, fy))
      {
        target_valid = true;
        target_x = fx;
        target_y = fy;
        RCLCPP_INFO(
          get_logger(),
          "追击: 已锁定目标 (%.2f, %.2f) m (点选 (%.2f, %.2f) 附近 %.2f m 内最近障碍簇)",
          target_x, target_y, pick_x, pick_y, lock_radius);
      }
    } else {
      // 已锁定: 以上一帧位置为中心, 在搜索半径内重新定位最近障碍簇。
      //   本帧若在搜索半径内找不到目标簇, 视为"当前无目标": 直接停车等待其
      //   重新出现 (丢失后的重寻/搜索点逻辑已移除, 以后再实现)。
      double fx = 0.0;
      double fy = 0.0;

      if (findNearestObstacleCluster(
          occupancy, config, width, height, target_x, target_y,
          track_radius, fx, fy))
      {
        target_x = fx;
        target_y = fy;
      } else {
        target_valid = false;
      }
    }

    // ---- 3) 记录目标轨迹 (节流) + 3D 点云 ----
    if (target_valid) {
      bool do_record = false;
      {
        std::lock_guard<std::mutex> lock(chase_mutex_);
        if (chase_last_record_time_.nanoseconds() == 0 ||
          (now() - chase_last_record_time_).seconds() >=
          chase_track_record_interval_)
        {
          chase_last_record_time_ = now();
          do_record = true;
        }
      }

      if (do_record) {
        // 采集目标簇附近的 3D 点云: 用当前完整点云 (含静态+动态) 抽取落在目标
        //   邻域的点, 记录其真实 z 值, 供丢失后回溯/网页展示目标 3D 信息。
        recordTargetCloud(target_x, target_y, radius);
      }
    }

    // ---- 4) 计算本帧期望到达点 ----
    //   目标可见: 计算"环形站位点" -> 距目标 radius 处; 若连线被挡, 用 A* 绕行。
    //   当前无目标 (本帧搜索半径内找不到): 直接停车等待其重新出现。
    double goal_x = target_x;
    double goal_y = target_y;
    bool use_astar = false;

    vx = 0.0;
    vy = 0.0;

    if (!target_valid) {
      // 当前无目标: 停车 (丢失后的重寻/搜索点逻辑已移除, 以后再实现)
      publishChaseViz(false, target_x, target_y, target_x, target_y);
      {
        std::lock_guard<std::mutex> lock(chase_mutex_);
        chase_target_valid_ = false;
        chase_target_x_ = target_x;
        chase_target_y_ = target_y;
        chase_goal_x_ = target_x;
        chase_goal_y_ = target_y;
      }
      publishChaseStatus();
      publishChaseTrack();
      return true;
    }

    {
      // 站位点: 目标 -> 车体方向上, 距目标 radius 处 (即车体朝目标方向保持环带)
      double ux = base_x - target_x;
      double uy = base_y - target_y;
      const double dist = std::hypot(ux, uy);

      if (dist < 1e-6) {
        // 与目标重合 (异常): 站位点退化为目标外侧一个环带半径处
        ux = -cos_yaw;
        uy = -sin_yaw;
        goal_x = target_x + ux * radius;
        goal_y = target_y + uy * radius;
      } else {
        ux /= dist;
        uy /= dist;
        goal_x = target_x + ux * radius;
        goal_y = target_y + uy * radius;
      }

      // 连线检查: 车体中心 -> 站位点 若被障碍挡住, 改用 A* 绕行到站位点
      if (!chaseLineClear(
          occupancy, config, width, height,
          base_x, base_y, goal_x, goal_y, target_x, target_y))
      {
        use_astar = true;

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "追击: 车体与目标连线间存在障碍, 改用 A* 绕行到站位点 (%.2f, %.2f)",
          goal_x, goal_y);
      }
    }

    if (use_astar) {
      // 用 A* 规划到站位点的绕障路径并跟随
      std::vector<Point> astar_path;

      if (planAstarToGoal(
          config, width, height, base_x, base_y, goal_x, goal_y, astar_path))
      {
        publishChaseViz(true, target_x, target_y, goal_x, goal_y);
        computeFollowCommand(
          astar_path, odom_x, odom_y, odom_yaw, vx, vy,
          chase_cruise_speed, chase_slow_radius);
      } else {
        // A* 失败: 停车, 保持追击态重试
        vx = 0.0;
        vy = 0.0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "追击: A* 规划失败 (站位点不可达), 本帧停车重试");
      }
    } else {
      // 直连站位点: 两点路径复用跟随算法
      std::vector<Point> chase_plan;
      chase_plan.push_back(
        Point{static_cast<float>(base_x), static_cast<float>(base_y), 0.0F});
      chase_plan.push_back(
        Point{static_cast<float>(goal_x), static_cast<float>(goal_y), 0.0F});

      // 已进入环带死区: 不再前后微调 (仅当站位点与当前位置接近时)
      const double d_goal = std::hypot(goal_x - base_x, goal_y - base_y);

      if (d_goal <= deadband) {
        vx = 0.0;
        vy = 0.0;
      } else {
        computeFollowCommand(
          chase_plan, odom_x, odom_y, odom_yaw, vx, vy,
          chase_cruise_speed, chase_slow_radius);
      }

      publishChaseViz(true, target_x, target_y, goal_x, goal_y);
    }

    // ---- 5) 机动安全兜底: 若按此速度行进会侵入硬性间距, 直接急停 ----
    {
      const double dt = 1.0 / std::max(1.0, motion_rate_hz_);
      const double nx = base_x + (vx * cos_yaw - vy * sin_yaw) * dt;
      const double ny = base_y + (vx * sin_yaw + vy * cos_yaw) * dt;

      // 目标可见时豁免目标本体 (站位点几何已保证与其保持环带/硬性间距)
      const double exempt_radius =
        astar_inflate_radius_ + config.resolution * 1.5;

      if (!isHoldPointClearPath(
          occupancy, config, width, height,
          base_x, base_y, nx, ny, astar_inflate_radius_,
          target_x, target_y, exempt_radius))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "追击: 预判机动会侵入硬性间距 (%.2f m), 本帧停车以保证不碰撞",
          astar_inflate_radius_);
        vx = 0.0;
        vy = 0.0;
      }
    }

    // ---- 6) 记录仲裁结果供网页展示 ----
    {
      std::lock_guard<std::mutex> lock(chase_mutex_);
      chase_target_valid_ = target_valid;
      chase_target_x_ = target_x;
      chase_target_y_ = target_y;
      chase_goal_x_ = goal_x;
      chase_goal_y_ = goal_y;
    }

    // ---- 7) 发布状态与轨迹 (供网页绘制) ----
    publishChaseStatus();
    publishChaseTrack();

    return true;
  }

// ===================== 源码行 5638-5671 · recordTargetCloud =====================
  void recordTargetCloud(double target_x, double target_y, double radius)
  {
    std::vector<Point> nearby;

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      const double r = std::max(0.2, radius * 1.5);
      const double r_sq = r * r;

      for (const auto & point : latest_cloud_) {
        const double dx = static_cast<double>(point.x) - target_x;
        const double dy = static_cast<double>(point.y) - target_y;

        if (dx * dx + dy * dy <= r_sq) {
          nearby.push_back(point);
        }
      }
    }

    std::lock_guard<std::mutex> lock(chase_mutex_);
    ChaseTrackPoint entry;
    entry.stamp = now();
    entry.x = target_x;
    entry.y = target_y;
    entry.cloud = std::move(nearby);
    chase_track_.push_back(std::move(entry));

    // 按时长淘汰: 保留 chase_track_history_sec_ 内的记录
    while (!chase_track_.empty() &&
      (now() - chase_track_.front().stamp).seconds() > chase_track_history_sec_)
    {
      chase_track_.pop_front();
    }
  }

```

## `tasks/motion_control.cpp`

建议落点：class MotionController

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `computeFollowCommand` | 5823-6035 | 函数 | 跟随核心: 外参反推车体中心 -> 到达判定 -> 前视半平面最近点 -> 横向偏差保护 -> 梯形减速 -> 速度自适应 lookahead -> 弧长推进取前视点 -> 投影成车体系 (vx,vy) -> 写前视可视化 |
| `computeAstarFollowCommand` | 5792-5804 | 函数 | A* 兜底路径跟随薄壳: 解析巡航速度后转调 computeFollowCommand |
| `startMotion` | 6038-6120 | 函数 | 开始运动: 校验里程计与 /plan (无 plan 但有目标点时允许, 交给 A* 兜底), 置运动态 |
| `stopMotion` | 6122-6150 | 函数 | 停车: 清运动/等待/重规划标志, 作废前视可视化, 下发 0 速帧 |

```cpp
// ===================== 源码行 5823-6035 · computeFollowCommand =====================
  bool computeFollowCommand(
    const std::vector<Point> & plan,
    double odom_x,
    double odom_y,
    double odom_yaw,
    double & vx,
    double & vy,
    double cruise_speed_override = -1.0,
    double slow_radius_override = -1.0)
  {
    // 0) 先把雷达位姿换算到车体中心: 之后所有几何量 (到达判定/最近点/
    //    前视点/速度投影) 都在同一参考点 (车体中心) 下计算, 消除雷达原点
    //    与车体中心之间约 0.29 m 外参带来的选择偏差。
    //    !!! 方向极易搞反 (历史 bug): motion_base_to_lidar_{x,y} 的语义是
    //    "车体中心 -> 雷达原点" 的平移, 表达在车体系 —— 即"从车体中心出发
    //    走到雷达"的向量。因此从雷达位姿反推车体中心必须做【减法】:
    //        base = lidar_odom - R(yaw) * motion_base_to_lidar
    //    若误写成加法, 车体中心会被镜像到雷达另一侧 (偏差约 0.57 m),
    //    表现为: 前视点明明在正侧方 (应 vx=0, vy=V), 实际却输出斜向速度。
    //    其数值恰为 lidar_to_base_{x,y} (雷达系表达) 的反向量, 两种写法
    //    结果一致: base = odom + R(yaw)*lidar_to_base = odom - R(yaw)*base_to_lidar。
    const double cos_yaw = std::cos(odom_yaw);   // 车头航向 (与雷达水平朝向一致)
    const double sin_yaw = std::sin(odom_yaw);
    const double base_x = odom_x -
      (cos_yaw * motion_base_to_lidar_x_ - sin_yaw * motion_base_to_lidar_y_);
    const double base_y = odom_y -
      (sin_yaw * motion_base_to_lidar_x_ + cos_yaw * motion_base_to_lidar_y_);

    // 1) 路径末端距离: 用于到达判定与减速
    const double dx_end = plan.back().x - base_x;
    const double dy_end = plan.back().y - base_y;
    const double dist_end = std::hypot(dx_end, dy_end);

    if (dist_end <= motion_goal_tolerance_) {
      RCLCPP_INFO(
        get_logger(),
        "已到达目标点, 距离 %.3f m <= 阈值 %.3f m, 停车",
        dist_end,
        motion_goal_tolerance_);
      return true;
    }

    // 2) 最近点: 优先在车头前方半平面 (车体系 forward >= -容差) 内搜索,
    //    避免路径折返段/起点处的杂点抢占最近点; 前方无候选时退回全局最近点
    //    注意: 最近点搜索本身与 lookahead 无关 (见 2b 的前视点推进), 因此可
    //    先做最近点 -> 再算减速后的实际速度 speed -> 再由 speed 推自适应
    //    lookahead, 从而用"实际速度"而非巡航速度计算前视距离 (更准版)。
    //    允许路径点略越过正侧线 (约 5 cm), 保证贴着车体侧面的路径点仍算"前方"
    constexpr double kFrontSectorTolerance = 0.05;

    double best_dist = std::numeric_limits<double>::max();
    std::size_t nearest_index = 0;
    double front_best_dist = std::numeric_limits<double>::max();
    std::size_t front_nearest_index = 0;
    bool has_front = false;

    for (std::size_t i = 0; i < plan.size(); ++i) {
      const double dx = plan[i].x - base_x;
      const double dy = plan[i].y - base_y;
      const double d = std::hypot(dx, dy);

      if (d < best_dist) {
        best_dist = d;
        nearest_index = i;
      }

      if (dx * cos_yaw + dy * sin_yaw >= -kFrontSectorTolerance &&
        d < front_best_dist)
      {
        front_best_dist = d;
        front_nearest_index = i;
        has_front = true;
      }
    }

    if (has_front) {
      best_dist = front_best_dist;
      nearest_index = front_nearest_index;
    }

    // 3) 横向偏差保护: 偏差过大说明路径已失效 (被顶开/卡住)
    if (motion_max_lateral_ > 0.0 &&
      best_dist > motion_max_lateral_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "偏离全局路径 %.3f m (上限 %.3f m), 停车等待重新规划",
        best_dist,
        motion_max_lateral_);
      return true;
    }

    // 4) 目标速度: 接近终点时按剩余距离线性减速
    //    减速起始距离由目标速度决定 (默认 = 目标速度 × kSlowRadiusTime, 见
    //    effectiveSlowRadius): 速度越大越早开始减速; 参数 motion_slow_radius > 0
    //    时改用手动指定值。剩余距离在 [到点阈值, 减速起点] 区间内线性降速。
    //    巡航速度: 自建 A* 兜底路径按 astar_speed_ 低速跟随 (cruise_speed_override>0),
    //    否则用 Nav2 路径的 motion_target_speed_。
    const double speed_cmd = std::clamp(
      cruise_speed_override > 0.0 ? cruise_speed_override : motion_target_speed_,
      0.0, 5.0);

    // 减速起始距离: slow_radius_override < 0 用默认 (motion_slow_radius_ /
    //   由目标速度自动决定); >= 0 时用覆盖值 (0 = 关闭减速)。
    const double slow_radius = (slow_radius_override < 0.0) ?
      effectiveSlowRadius(
      motion_slow_radius_, speed_cmd, motion_goal_tolerance_) :
      slow_radius_override;

    double speed = speed_cmd;

    // 梯形减速: 剩余距离在 [到点阈值, 减速起点] 区间内线性降速。
    //   占点模式传入比常规更小的 slow_radius_override, 大幅缩短减速段 ——
    //   既保留末段缓冲、避免全速硬停 (更新频率低时容易冲过站位点), 又不会像
    //   常规导航那样"早早开始磨蹭"而跟不上敌人。
    if (slow_radius > motion_goal_tolerance_ && dist_end < slow_radius) {
      const double ratio =
        (dist_end - motion_goal_tolerance_) /
        (slow_radius - motion_goal_tolerance_);
      speed = std::max(motion_min_speed_, speed * ratio);
    }

    // 4b) 前视距离速度自适应: L = L_min + k * V (V 取上一步算出的实际速度)
    //     原理: 纯方向跟踪 (vx,vy 直接指向"车体中心 -> 前视点") 下, 小横向误差
    //     e 的一阶收敛时间常数 tau = L / V。固定 L 时高速段 tau 过小, 叠加执行
    //     延迟必然过冲 -> 表现为 S 型振荡; 让 L 随 V 线性增长可使 tau 恒定,
    //     全速度段阻尼比一致 (Pure Pursuit 标准做法)。此处用实际下发速度 (含
    //     减速段) 而非巡航速度: 接近终点减速时前视距离同步收缩, 到点更贴路径。
    //     motion_lookahead_speed_gain_ = 0 时退化为固定前视距离。
    double lookahead = std::max(0.05, motion_lookahead_);
    if (motion_lookahead_speed_gain_ > 0.0) {
      lookahead += motion_lookahead_speed_gain_ * std::clamp(speed, 0.0, 5.0);
    }
    if (motion_lookahead_max_ > 0.0) {
      lookahead = std::min(lookahead, motion_lookahead_max_);
    }

    // 4c) 前视点: 沿路径按弧长推进 —— 从最近点的弧长 s 出发, 取 s + lookahead
    //     对应的路径点。相比旧逻辑"第一个距离超过 lookahead 的点", 弧长推进
    //     保证前视点严格沿路径向前, 路径折回/弯曲/起点偏移时不会取到绕回段点。
    //     (原步骤 2b 前移至此, 以便用自适应后的 lookahead。)
    std::vector<double> cumulative(plan.size(), 0.0);
    for (std::size_t i = 1; i < plan.size(); ++i) {
      cumulative[i] = cumulative[i - 1] +
        std::hypot(plan[i].x - plan[i - 1].x, plan[i].y - plan[i - 1].y);
    }

    const double lookahead_s = cumulative[nearest_index] + lookahead;
    std::size_t lookahead_index = plan.size() - 1;
    for (std::size_t i = nearest_index; i < plan.size(); ++i) {
      if (cumulative[i] >= lookahead_s) {
        lookahead_index = i;
        break;
      }
    }

    // 5) 前视点位移 -> 车辆坐标系 (vx, vy)
    //    全向底盘 + 车头朝向保持不动: 速度矢量直接指向"车体中心 -> 前视点",
    //    因此 (vx, vy) 就是该单位矢量在车体系下的两个分量, 不再引入任何
    //    转向增益 (motion_yaw_kp_ 不再参与速度计算)。
    //    此写法下点正前/正后/正左/正右时, 必有一轴严格为 0。
    //    雷达为"头朝地面"倒装 (绕其自身 X 轴旋转 180 度), 该旋转只翻转雷达
    //    的 Y/Z 轴: 水平方向上雷达 +X 依然指向车头, 故 odom_yaw 即车头航向。
    const double dx_t = plan[lookahead_index].x - base_x;
    const double dy_t = plan[lookahead_index].y - base_y;

    // 车体系投影: forward = 车头前方分量, left = 车辆左手边分量
    //   forward 轴 = ( cos_yaw,  sin_yaw)  —— 车头前方
    //   right  轴 = ( sin_yaw, -cos_yaw)  —— 前方轴顺时针 90 度, 车辆右手边
    //   left   轴 = (-sin_yaw,  cos_yaw)  —— 前方轴逆时针 90 度, 车辆左手边
    // !!! 横向分量的符号 (历史 bug, 极易搞反):
    //   运动帧协议与网页可视化统一约定 vy 以【车辆左手边】为正
    //   (见 LookaheadView 注释与网页 drawBodyFrame 绘制的 y 轴)。
    //   但里程计来自"头朝地面"倒装的雷达 —— 雷达绕自身 X 轴转 180 度,
    //   水平方向上其 +X 仍指向车头 (故 odom_yaw 可直接当车头航向), 而 +Y
    //   轴被翻转到【车辆右手边】。所以按 (sin,-cos) 投影得到的分量其实是
    //   "右手边/雷达 +Y" 分量, 直接当作 vy 下发会导致横向整体反号
    //   (目标在左却往右走)。这里必须取反得到车辆左手边分量。
    const double forward = dx_t * cos_yaw + dy_t * sin_yaw;
    const double veh_right = dx_t * sin_yaw - dy_t * cos_yaw;
    const double left = -veh_right;   // 雷达 +Y(右手边) -> 车体系 +Y(左手边)

    const double norm = std::hypot(forward, left);
    if (norm < 1e-6) {
      // 前视点与车体中心重合: 维持静止, 避免除零
      vx = 0.0;
      vy = 0.0;
    } else {
      // vx 可为负 => 正后方目标即真正后退 (车尾方向), 而非原地横移
      // 无倒车降速保护: 倒车时同样以目标速度行驶
      vx = speed * (forward / norm);
      vy = speed * (left / norm);
    }

    // 5b) 记录可视化状态 (供网页在规划路径上叠加前视点/速度矢量/车体系)
    {
      std::lock_guard<std::mutex> lock(lookahead_mutex_);
      lookahead_view_.valid = true;
      lookahead_view_.base_x = base_x;
      lookahead_view_.base_y = base_y;
      lookahead_view_.yaw = odom_yaw;
      lookahead_view_.target_x = plan[lookahead_index].x;
      lookahead_view_.target_y = plan[lookahead_index].y;
      lookahead_view_.vx = vx;
      lookahead_view_.vy = vy;
      lookahead_view_.lookahead = lookahead;
      lookahead_view_.dist_end = dist_end;
    }

    return false;
  }

// ===================== 源码行 5792-5804 · computeAstarFollowCommand =====================
  bool computeAstarFollowCommand(
    const std::vector<Point> & plan,
    double odom_x,
    double odom_y,
    double odom_yaw,
    double & vx,
    double & vy)
  {
    const double cruise = std::clamp(
      resolveAstarSpeed(), 0.0, kMotionMaxSpeed);
    return computeFollowCommand(
      plan, odom_x, odom_y, odom_yaw, vx, vy, cruise);
  }

// ===================== 源码行 6038-6120 · startMotion =====================
  void startMotion()
  {
    bool odom_ok = false;
    bool plan_ok = false;
    std::size_t plan_size = 0;
    bool goal_ok = false;
    double dist_end = 0.0;

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      odom_ok = odom_received_;
      plan_ok = motion_plan_received_;
      plan_size = motion_plan_.size();
      goal_ok = goal_received_;

      if (odom_ok && plan_ok && plan_size > 0) {
        dist_end = std::hypot(
          motion_plan_.back().x - odom_x_,
          motion_plan_.back().y - odom_y_);
      } else if (odom_ok && goal_ok) {
        // 无 /plan 但有 Web 目标点: 用目标点估算距离 (A* 兜底会接管)
        dist_end = std::hypot(goal_x_ - odom_x_, goal_y_ - odom_y_);
      }
    }

    if (!odom_ok) {
      RCLCPP_WARN(get_logger(), "无法开始运动: 未收到里程计 (/Odometry)");
      return;
    }

    // 允许两种情况开始运动:
    //   (a) 已有可用 /plan (正常 Nav2 路径跟随);
    //   (b) 无 /plan 但有 Web 目标点 —— 用户"从障碍区开始导航"/刚进障碍区时,
    //       Nav2 会因起点落在障碍膨胀区内而规划失败, /plan 不会到来; 此时不应
    //       拒绝启动, 否则用户根本走不了。交给自建 A* 兜底规划器接管。
    const bool have_plan = plan_ok && plan_size >= 2;

    if (!have_plan && !goal_ok) {
      RCLCPP_WARN(
        get_logger(),
        "无法开始运动: 未收到有效的 Nav2 全局路径 (/plan), 也未收到 Web 目标点, "
        "请先在地图上点击目标点");
      return;
    }

    if (have_plan && dist_end <= motion_goal_tolerance_) {
      RCLCPP_INFO(
        get_logger(),
        "当前位置距目标 %.3f m 已在阈值 %.3f m 内, 无需运动",
        dist_end,
        motion_goal_tolerance_);
      publishMotionFrame(false, 0.0, 0.0);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      motion_active_ = true;
      motion_waiting_ = false;
      // 新一次运动: 失效 A* 缓存, 首帧按最新障碍重新规划
      astar_cache_valid_ = false;
      // 新一次运动: 清零"到达后位置校验"重试计数
      motion_goal_check_retry_ = 0;
      motion_goal_check_replan_ = false;
    }

    if (have_plan) {
      RCLCPP_INFO(
        get_logger(),
        "开始运动: 跟随全局路径 (%zu 点), 距目标 %.2f m, 目标速度 %.2f m/s",
        plan_size,
        dist_end,
        motion_target_speed_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "开始运动: 暂无可用 Nav2 全局路径 (/plan), 交由自建 A* 兜底规划器接管 "
        "(驶向 Web 目标点, 距目标 %.2f m, 巡航 %.2f m/s); "
        "/plan 恢复后自动切回跟随 Nav2",
        dist_end,
        resolveAstarSpeed());
    }
  }

// ===================== 源码行 6122-6150 · stopMotion =====================
  void stopMotion(const char * reason)
  {
    bool was_active = false;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      was_active = motion_active_;
      motion_active_ = false;
      motion_waiting_ = false;   // 真正停车时退出等待/兜底态
      // 真正停车: 退出"校验失败已改用 A* 重导航"态, 下次开始运动重新判断
      motion_goal_check_replan_ = false;
      motion_goal_check_retry_ = 0;
    }

    // 停车后不再有当前前视点: 网页随之隐藏前视点/速度矢量/车体系叠加层
    {
      std::lock_guard<std::mutex> lock(lookahead_mutex_);
      lookahead_view_.valid = false;
    }

    publishMotionFrame(false, 0.0, 0.0);

    if (was_active) {
      if (reason != nullptr && reason[0] != '\0') {
        RCLCPP_WARN(get_logger(), "已停止运动: %s", reason);
      } else {
        RCLCPP_INFO(get_logger(), "已停止运动");
      }
    }
  }

```

## `tasks/debug_log.cpp`

建议落点：class DebugLogger

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `logCloudFrame` | 917-1042 | 函数 | 逐帧统计质心/包围盒, 判定点数突增与质心跳变, 写 CSV, 异常时落 PCD 快照 |

```cpp
// ===================== 源码行 917-1042 · logCloudFrame =====================
  void logCloudFrame(
    const rclcpp::Time & stamp,
    std::size_t raw_points,
    std::size_t valid_points,
    std::size_t self_filtered,
    std::size_t queued,
    const std::vector<Point> & cloud)
  {
    if (!debug_log_enabled_) {
      return;
    }

    ++debug_frame_index_;

    // ---- 统计: 质心 / 包围盒 / 越界 z 探测 ----
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    double xmin = 0.0;
    double xmax = 0.0;
    double ymin = 0.0;
    double ymax = 0.0;
    double zmin = 0.0;
    double zmax = 0.0;

    if (!cloud.empty()) {
      xmin = xmax = static_cast<double>(cloud.front().x);
      ymin = ymax = static_cast<double>(cloud.front().y);
      zmin = zmax = static_cast<double>(cloud.front().z);

      for (const auto & point : cloud) {
        cx += point.x;
        cy += point.y;
        cz += point.z;
        xmin = std::min(xmin, static_cast<double>(point.x));
        xmax = std::max(xmax, static_cast<double>(point.x));
        ymin = std::min(ymin, static_cast<double>(point.y));
        ymax = std::max(ymax, static_cast<double>(point.y));
        zmin = std::min(zmin, static_cast<double>(point.z));
        zmax = std::max(zmax, static_cast<double>(point.z));
      }

      const double inv = 1.0 / static_cast<double>(cloud.size());
      cx *= inv;
      cy *= inv;
      cz *= inv;
    }

    // ---- 异常判定 ----
    //   1) 点数突增: 本帧 > 上帧 * ratio 且 差值 > delta
    //   2) 质心跳变: 与上帧质心距离 > 阈值 (典型表现为 LIO 位姿跳变 -> 整帧点云
    //      整体位移, 在历史图上"烧"出一大片假障碍)
    bool anomaly = false;
    std::string anomaly_reason;

    if (debug_prev_valid_ && !cloud.empty()) {
      const double delta_points =
        static_cast<double>(cloud.size()) -
        static_cast<double>(debug_prev_points_);

      if (static_cast<double>(cloud.size()) >
        static_cast<double>(debug_prev_points_) *
        debug_cloud_count_jump_ratio_ &&
        delta_points >
        static_cast<double>(debug_cloud_count_jump_delta_))
      {
        anomaly = true;
        anomaly_reason = "点数突增";
      }

      const double centroid_jump =
        std::hypot(
        cx - debug_prev_cx_, cy - debug_prev_cy_);

      if (centroid_jump > debug_centroid_jump_m_) {
        anomaly = true;
        if (!anomaly_reason.empty()) {
          anomaly_reason += "+";
        }
        anomaly_reason += "质心跳变";
      }
    }

    debug_prev_valid_ = !cloud.empty();
    debug_prev_points_ = cloud.size();
    debug_prev_cx_ = cx;
    debug_prev_cy_ = cy;
    debug_prev_cz_ = cz;

    // ---- 写 CSV ----
    {
      std::lock_guard<std::mutex> lock(debug_log_mutex_);

      if (debug_cloud_log_) {
        debug_cloud_log_ << stamp.seconds() << ','
                         << debug_frame_index_ << ','
                         << raw_points << ','
                         << valid_points << ','
                         << self_filtered << ','
                         << queued << ','
                         << cx << ',' << cy << ',' << cz << ','
                         << xmin << ',' << xmax << ','
                         << ymin << ',' << ymax << ','
                         << zmin << ',' << zmax << ','
                         << (anomaly ? anomaly_reason : "-") << '\n';
        debug_cloud_log_.flush();
      }
    }

    // ---- 异常: WARN + PCD 快照 ----
    if (anomaly) {
      const bool saved = saveCloudPcd(cloud, "anomaly", stamp);

      RCLCPP_WARN(
        get_logger(),
        "点云异常帧 [%s]: 点数=%zu (上帧=%zu), 质心=(%.2f, %.2f, %.2f) m, "
        "包围盒 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f], 自过滤=%zu, "
        "PCD快照=%s",
        anomaly_reason.c_str(),
        cloud.size(), debug_prev_points_,
        cx, cy, cz,
        xmin, xmax, ymin, ymax, zmin, zmax,
        self_filtered,
        saved ? "已保存" : "未保存(关闭或达上限)");
    }
  }

```

## `src/main.cpp`

建议落点：int main()

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `main` | 7927-7937 | 函数 | rclcpp::init -> 构造 RealtimePgmNode -> spin -> shutdown |

```cpp
// ===================== 源码行 7927-7937 · main =====================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RealtimePgmNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

```

## `src/realtime_pgm_node.cpp`

建议落点：class RealtimePgmNode

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `constructor_assembly` | 605-815 | 代码块 | 构造函数装配区: 创建 6 个订阅 / 12 个发布器 / 2 个定时器 / 2 个服务客户端 / 参数回调句柄 |
| `printConfig` | 7471-7567 | 函数 | 把全部生效配置打印到日志 (含话题名与 CAN 状态) |
| `createMotionTimer` | 3631-3649 | 函数 | 按 motion_rate_hz_ 取消并重建 wall timer (频率热改) |

```cpp
// ===================== 源码行 605-815 · constructor_assembly =====================
    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &RealtimePgmNode::odomCallback,
        this,
        std::placeholders::_1));

    plan_subscription_ =
      create_subscription<nav_msgs::msg::Path>(
      global_plan_topic_,
      10,
      std::bind(
        &RealtimePgmNode::planCallback,
        this,
        std::placeholders::_1));

    local_plan_subscription_ =
      create_subscription<nav_msgs::msg::Path>(
      local_plan_topic_,
      10,
      std::bind(
        &RealtimePgmNode::localPlanCallback,
        this,
        std::placeholders::_1));

    // Web 目标点: 网页点击目标时同时向该话题发一帧 map 系 PoseStamped,
    //   供自建 A* 兜底规划器作为终点 (Nav2 的 NavigateToPose 动作不暴露话题,
    //   本节点无法直接订阅, 故由网页额外发布一份)。
    if (!goal_pose_topic_.empty()) {
      goal_pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_,
        10,
        std::bind(
          &RealtimePgmNode::goalPoseCallback,
          this,
          std::placeholders::_1));
    }

    // 自建 A* 路径发布 (供网页黄色叠加显示)
    if (!astar_plan_topic_.empty()) {
      astar_plan_publisher_ =
        create_publisher<nav_msgs::msg::Path>(astar_plan_topic_, 1);
    }

    // 占点可视化 (nav_msgs/Path, map 系): 供网页画占点圈/中心/拦截连线
    if (!hold_point_viz_topic_.empty()) {
      hold_point_viz_publisher_ =
        create_publisher<nav_msgs::msg::Path>(hold_point_viz_topic_, 1);
    }

    // 占点状态发布 (JSON, std_msgs/String): 供网页展示当前威胁/拦截状态
    if (!hold_point_status_topic_.empty()) {
      hold_point_status_publisher_ =
        create_publisher<std_msgs::msg::String>(hold_point_status_topic_, 1);
    }

    // 追击可视化 (nav_msgs/Path, map 系): 第 1 点=当前目标, 第 2 点=期望站位点
    if (!chase_viz_topic_.empty()) {
      chase_viz_publisher_ =
        create_publisher<nav_msgs::msg::Path>(chase_viz_topic_, 1);
    }

    // 追击状态发布 (JSON, std_msgs/String): 供网页展示目标/丢失/搜索状态
    if (!chase_status_topic_.empty()) {
      chase_status_publisher_ =
        create_publisher<std_msgs::msg::String>(chase_status_topic_, 1);
    }

    // 目标历史轨迹发布 (nav_msgs/Path, map 系): 供网页画目标运动轨迹线
    if (!chase_track_topic_.empty()) {
      chase_track_publisher_ =
        create_publisher<nav_msgs::msg::Path>(chase_track_topic_, 1);
    }

    createMotionTimer();

    // 降采样点云预览, 供网页查看器绘制可交互点云侧视图
    cloud_preview_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_preview_topic_, rclcpp::SensorDataQoS());

    // 持久化历史地图图像 (最近 history_duration 秒点云按帧投票累积)
    history_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(history_image_topic_, 1);

    // 动态障碍 (可变障碍): 点云给 Nav2 obstacle_layer, 栅格图给网页绿色叠加
    dynamic_obstacle_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      dynamic_obstacle_topic_, rclcpp::SensorDataQoS());
    dynamic_obstacle_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(
      dynamic_obstacle_image_topic_, 1);
    pose_correction_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_correction_topic_, rclcpp::QoS(10));

    // 外部修正输入 (apply_pose_correction=false 时生效): 订阅外部重定位节点
    //   (如 reloc_3d GICP) 发布的 map->odom 修正量, 仅内部使用 (不转发/不发布)。
    //   内部 2D 模式下本节点是该话题的发布者, 回调直接 return 忽略回环消息。
    pose_correction_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_correction_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (apply_pose_correction_) {
          return;
        }
        PoseCorrection c;
        quaternionToMatrix(
          msg->pose.orientation.x, msg->pose.orientation.y,
          msg->pose.orientation.z, msg->pose.orientation.w, c.r);
        c.t[0] = msg->pose.position.x;
        c.t[1] = msg->pose.position.y;
        c.t[2] = msg->pose.position.z;
        c.valid = true;
        std::lock_guard<std::mutex> lock(pose_correction_mutex_);
        pose_correction_ = c;
      });

    // 自过滤点云: 车体自过滤后的当前帧完整点云 (静态+动态), frame=base_link。
    // 供 Nav2 obstacle_layer 的 pointcloud 源开启 clearing 后做射线清除, 把旧红色
    // 残留格清成空闲 (动态障碍源只含"新出现的临时障碍", 点太少, 射线覆盖不足)。
    self_filtered_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      self_filtered_topic_, rclcpp::SensorDataQoS());
    self_filtered_last_publish_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // 清除射线点云: base_link 系一圈稀疏点, 供 nav2_params.yaml 里只开 clearing
    // 的 clear_ring 源做射线清除 (见 publishClearRingCloud)。
    clear_ring_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      clear_ring_topic_, rclcpp::SensorDataQoS());

    // Nav2 代价地图清空服务客户端: 刷新历史累计图时整张清空红区 (见
    // clearCostmaps)。服务端为同步实现, 故用异步调用 + 回调, 不阻塞渲染线程。
    clear_local_costmap_client_ =
      create_client<nav2_msgs::srv::ClearEntireCostmap>(
      clear_local_costmap_service_);
    clear_global_costmap_client_ =
      create_client<nav2_msgs::srv::ClearEntireCostmap>(
      clear_global_costmap_service_);

    cloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &RealtimePgmNode::cloudCallback,
        this,
        std::placeholders::_1));

    render_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(render_period_ms_),
      std::bind(
        &RealtimePgmNode::renderTimerCallback,
        this));

    warn_timer_ =
      create_wall_timer(
      std::chrono::seconds(5),
      [this]() {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (!received_cloud_) {
          RCLCPP_WARN(
            get_logger(),
            "尚未收到点云，请检查话题: %s",
            cloud_topic_.c_str());
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "实时 PGM 节点启动: topic=%s, output=%s, render_period=%d ms, "
      "cloud_preview=%s (max %d pts), history_image=%s (keep %.0f s, "
      "self_filtered=%s @%.1f Hz/max %d pts, CAN 直发=%s)",
      cloud_topic_.c_str(),
      initial_config.output.c_str(),
      render_period_ms_,
      cloud_preview_topic_.c_str(),
      cloud_preview_max_points_,
      history_image_topic_.c_str(),
      initial_config.history_duration,
      self_filtered_topic_.c_str(),
      self_filtered_publish_rate_,
      self_filtered_max_points_,
      can_enabled_.load() ? "开" : "关");

    RCLCPP_INFO(
      get_logger(),
      "清除射线点云: topic=%s, 环点数=%d (每 %.0f° 一个), 半径=%.2f m%s",
      clear_ring_topic_.c_str(),
      clear_ring_points_,
      clear_ring_points_ > 0 ? 360.0 / clear_ring_points_ : 0.0,
      clear_ring_radius_,
      (clear_ring_points_ > 0 && clear_ring_radius_ > 0.0) ?
      "" : " [已关闭]");

    RCLCPP_INFO(
      get_logger(),
      "刷新历史图清空代价区: %s, local=%s, global=%s",
      clear_costmap_on_history_reset_ ? "开" : "关",
      clear_local_costmap_service_.c_str(),
      clear_global_costmap_service_.c_str());

    printConfig(initial_config);

    initDebugLog();
  }

// ===================== 源码行 7471-7567 · printConfig =====================
  void printConfig(const Config & config)
  {
    RCLCPP_INFO(
      get_logger(),
      "当前配置: resolution=%.4f, zmin=%.3f, zmax=%.3f, dilate=%d, "
      "baseline_dilate=%d, self_filter_size=%.2f m",
      config.resolution,
      config.zmin,
      config.zmax,
      config.dilate,
      config.baseline_dilate,
      config.self_filter_size);

    RCLCPP_INFO(
      get_logger(),
      "地图范围: x=[%.2f, %.2f], y=[%.2f, %.2f]",
      config.xmin,
      config.xmax,
      config.ymin,
      config.ymax);

    RCLCPP_INFO(
      get_logger(),
      "历史地图: 保留 %.1f s, 最低命中帧数=%d, 每帧上限=%d, "
      "image=%s, 输出=%s",
      config.history_duration,
      config.history_min_hits,
      config.history_max_points_per_frame,
      history_image_topic_.c_str(),
      config.history_output.c_str());

    RCLCPP_INFO(
      get_logger(),
      "位姿修正 (锁定历史图后生效): 来源=%s, topic=%s, 评分容差=基准膨胀+1 格, "
      "局部搜索 ±%.1f m / ±15° (步长 0.2 m) + 格级细化, "
      "全局兜底 ±%.1f m / ±180° (连续失败 %d 帧后触发, 最少间隔 %.1f s), "
      "分数阈值=%.2f, 单帧上限=%.2f m, EMA alpha=%.2f, 滞回增益=%.2f",
      apply_pose_correction_ ?
        "本节点 2D 栅格配准 (估计并发布)" : "外部节点 (订阅 /pose_correction)",
      pose_correction_topic_.c_str(),
      pose_correction_local_range_m_,
      pose_correction_global_range_m_,
      pose_correction_global_trigger_fails_,
      pose_correction_global_period_sec_,
      pose_correction_score_min_,
      pose_correction_max_delta_m_,
      pose_correction_ema_alpha_,
      pose_correction_score_gain_);

    RCLCPP_INFO(
      get_logger(),
      "导航运动控制: 下发话题=%s (CAN 0xFF 对应 8 字节: "
      "bit0=control/bit1=shoot/tar_yaw/tar_pitch/vx12/vy12), "
      "里程计=%s, 全局路径=%s (%.1f s 未更新即进入等待态%s), "
      "局部路径=%s (超时 %.2f s 后回退全局), "
      "目标速度=%.2f m/s, 前视距离=%.2f m "
      "(自适应增益=%.2f, 上限=%.2f m), "
      "到点阈值=%.2f m, 减速起始距离=%.2f m%s, 下发频率=%.1f Hz, "
      "到达校验=%s (阈值=%.2f m, 最多重试=%d 次), "
      "A*兜底=%s (跟随速度=%.2f m/s, 硬性间距=%.2f m, 软性避让范围=%.2f m, "
      "失败则停车并保持重试)",
      motion_topic_.c_str(),
      odom_topic_.c_str(),
      global_plan_topic_.c_str(),
      motion_plan_timeout_,
      motion_plan_timeout_ > 0.0 ? "" : " [监督已关闭]",
      local_plan_topic_.c_str(),
      local_plan_timeout_,
      motion_target_speed_,
      motion_lookahead_,
      motion_lookahead_speed_gain_,
      motion_lookahead_max_,
      motion_goal_tolerance_,
      effectiveSlowRadius(
        motion_slow_radius_, motion_target_speed_, motion_goal_tolerance_),
      (motion_slow_radius_ > 0.0) ? " (手动指定)" : " (由目标速度自动决定)",
      motion_rate_hz_,
      (motion_goal_check_tolerance_ > 0.0) ? "已开启" :
      "未开启 (motion_goal_check_tolerance<=0)",
      motion_goal_check_tolerance_,
      motion_goal_check_max_retry_,
      astar_enabled_ ? "已开启" : "未开启 (astar_enabled=false)",
      resolveAstarSpeed(),
      astar_inflate_radius_,
      astar_cost_reach_);

    RCLCPP_INFO(
      get_logger(),
      "SocketCAN 直发: %s, 接口=%s, ID=0x%03X, 限频周期=%d ms "
      "(candump -t d %s,%03X:7FF 查看)",
      can_enabled_.load() ? "已开启" : "未开启 (can_enabled=false)",
      can_interface_.c_str(),
      can_id_,
      can_send_period_ms_,
      can_interface_.c_str(),
      can_id_);
  }

// ===================== 源码行 3631-3649 · createMotionTimer =====================
  void createMotionTimer()
  {
    if (motion_timer_) {
      motion_timer_->cancel();
    }

    double rate = 100.0;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      rate = motion_rate_hz_;
    }

    const double period_ms = 1000.0 / std::max(1.0, rate);

    motion_timer_ = create_wall_timer(
      std::chrono::milliseconds(
        std::max(1, static_cast<int>(std::lround(period_ms)))),
      std::bind(&RealtimePgmNode::motionTimerCallback, this));
  }

```

## `src/callbacks.cpp`

建议落点：class RealtimePgmNode (回调段)

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `odomCallback` | 1045-1147 | 函数 | 订阅 /Odometry: 解 yaw -> 位姿跳变检测并开启"拒绝窗口" -> 施加 map->odom 修正 -> 写车体位姿 |
| `planCallback` | 1150-1174 | 函数 | 缓存 Nav2 全局路径点 + 本机接收时刻 (新鲜度监督) |
| `localPlanCallback` | 1179-1201 | 函数 | 缓存局部路径(少于 2 点丢弃), 接收时刻用本机时钟 |
| `goalPoseCallback` | 1205-1215 | 函数 | 存网页 A* 终点 (x,y), 清零到达校验重试计数 |
| `renderTimerCallback` | 3363-3425 | 函数 | 渲染定时器: 消费 导出/导入/清史 请求 -> 快照点云与配置 -> 分派 renderPgm / renderHistoryPgm / renderDynamicObstaclePgm |
| `motionTimerCallback` | 3651-4133 | 函数 | 运动主状态机: 按钮请求 -> 追击接管 -> 占点接管 -> 取路径(局部优先) -> A* 兜底 -> 正常跟随 -> 到点校验重试 -> 发帧 |

```cpp
// ===================== 源码行 1045-1147 · odomCallback =====================
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // 位姿跳变检测 (LIO 发散/重定位): 两帧里程计之间位移或偏航突变往往是
    //   "点云突然全是障碍"的根因 —— 位姿一跳, 整帧 map 系点云被搬到错误位置。
    //   检出跳变后开启"拒绝窗口" pose_jump_until_: 窗口内的点云帧被 cloudCallback
    //   整体丢弃, 防止错位点云在历史缓冲里投票形成假障碍 (只丢帧, 不清历史)。
    {
      const double new_x = msg->pose.pose.position.x;
      const double new_y = msg->pose.pose.position.y;

      if (debug_prev_odom_valid_) {
        const double jump = std::hypot(
          new_x - debug_prev_odom_x_, new_y - debug_prev_odom_y_);

        double dyaw = yaw - debug_prev_odom_yaw_;

        while (dyaw > M_PI) {
          dyaw -= 2.0 * M_PI;
        }
        while (dyaw < -M_PI) {
          dyaw += 2.0 * M_PI;
        }

        if (jump > debug_odom_jump_m_ ||
          std::fabs(dyaw) > debug_odom_jump_yaw_)
        {
          if (pose_jump_reject_sec_ > 0.0) {
            const double stamp_sec =
              static_cast<double>(msg->header.stamp.sec) +
              static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
            pose_jump_until_sec_ = stamp_sec + pose_jump_reject_sec_;
            pose_jump_has_until_ = true;
            RCLCPP_WARN(
              get_logger(),
              "里程计位姿跳变: 位移=%.2f m, 偏航=%.1f deg, "
              "从 (%.2f, %.2f, %.1f deg) 到 (%.2f, %.2f, %.1f deg) "
              "(疑似 LIO 发散/重定位) —— 丢弃其后 %.2f s 内的点云帧",
              jump, dyaw * 180.0 / M_PI,
              debug_prev_odom_x_, debug_prev_odom_y_,
              debug_prev_odom_yaw_ * 180.0 / M_PI,
              new_x, new_y, yaw * 180.0 / M_PI,
              pose_jump_reject_sec_);
          } else {
            RCLCPP_WARN(
              get_logger(),
              "里程计位姿跳变: 位移=%.2f m, 偏航=%.1f deg, "
              "从 (%.2f, %.2f, %.1f deg) 到 (%.2f, %.2f, %.1f deg) "
              "(疑似 LIO 发散/重定位, 可能引发假障碍; 丢弃保护已关闭)",
              jump, dyaw * 180.0 / M_PI,
              debug_prev_odom_x_, debug_prev_odom_y_,
              debug_prev_odom_yaw_ * 180.0 / M_PI,
              new_x, new_y, yaw * 180.0 / M_PI);
          }
        }
      }

      debug_prev_odom_x_ = new_x;
      debug_prev_odom_y_ = new_y;
      debug_prev_odom_yaw_ = yaw;
      debug_prev_odom_valid_ = true;
    }

    // 位姿修正 (map->odom): 把里程计位姿变换到 map 系。
    //   统一走 SE(3) 矩阵复合: 本节点自带的 2D 栅格配准只给 (x, y, yaw),
    //   但修正量为恒等时结果与"直接用里程计"完全一致。
    const PoseCorrection correction = getPoseCorrection();
    double corrected_x = msg->pose.pose.position.x;
    double corrected_y = msg->pose.pose.position.y;
    double corrected_yaw = yaw;
    if (correction.valid) {
      const double px = msg->pose.pose.position.x;
      const double py = msg->pose.pose.position.y;
      const double pz = msg->pose.pose.position.z;
      corrected_x =
        correction.r[0] * px + correction.r[1] * py + correction.r[2] * pz +
        correction.t[0];
      corrected_y =
        correction.r[3] * px + correction.r[4] * py + correction.r[5] * pz +
        correction.t[1];

      // 偏航须按 "修正量 * 里程计姿态" 复合后取: 直接相加仅在纯 z 旋转下成立,
      // 而 3D 配准的修正量一般含有 roll/pitch。
      double r_odom[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      quaternionToMatrix(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, r_odom);
      double r_map[9];
      multiplyMatrix(correction.r, r_odom, r_map);
      corrected_yaw = yawFromMatrix(r_map);
    }

    std::lock_guard<std::mutex> lock(motion_mutex_);
    odom_x_ = corrected_x;
    odom_y_ = corrected_y;
    odom_yaw_ = corrected_yaw;
    odom_received_ = true;
    odom_stamp_ = msg->header.stamp;
  }

// ===================== 源码行 1150-1174 · planCallback =====================
  void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::vector<Point> plan;
    plan.reserve(msg->poses.size());

    for (const auto & pose : msg->poses) {
      plan.push_back(
        Point{
          static_cast<float>(pose.pose.position.x),
          static_cast<float>(pose.pose.position.y),
          0.0F});
    }

    if (plan.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(motion_mutex_);
    motion_plan_ = std::move(plan);
    motion_plan_received_ = true;
    motion_plan_stamp_ = msg->header.stamp;
    // 本机到达时刻: 用于"路径新鲜度"监督 (见 motionTimerCallback 的
    // motion_plan_timeout_)。用本机时钟而非 header.stamp, 避免依赖 Nav2 侧的时间基准。
    motion_plan_recv_time_ = now();
  }

// ===================== 源码行 1179-1201 · localPlanCallback =====================
  void localPlanCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::vector<Point> plan;
    plan.reserve(msg->poses.size());

    for (const auto & pose : msg->poses) {
      plan.push_back(
        Point{
          static_cast<float>(pose.pose.position.x),
          static_cast<float>(pose.pose.position.y),
          static_cast<float>(pose.pose.position.z)});
    }

    if (plan.size() < 2) {
      return;
    }

    std::lock_guard<std::mutex> lock(motion_mutex_);
    local_plan_ = std::move(plan);
    local_plan_received_ = true;
    // 到达时间用本机时钟: /local_plan 的 header.stamp 可能为 0, 不可用于超时判断
    local_plan_stamp_ = now();
  }

// ===================== 源码行 1205-1215 · goalPoseCallback =====================
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    goal_x_ = static_cast<float>(msg->pose.position.x);
    goal_y_ = static_cast<float>(msg->pose.position.y);
    goal_received_ = true;
    goal_recv_time_ = now();
    // 新目标点: 清零"到达后位置校验"重试计数
    motion_goal_check_retry_ = 0;
    motion_goal_check_replan_ = false;
  }

// ===================== 源码行 3363-3425 · renderTimerCallback =====================
  void renderTimerCallback()
  {
    const bool reset_history = history_reset_requested_.exchange(false);
    const bool dynamic_enabled = dynamic_obstacle_enabled_.load();
    const bool export_requested = export_map_requested_.exchange(false);
    const bool import_requested = import_map_requested_.exchange(false);

    // 导出/导入历史累计图: 独立于点云是否已到达, 也独立于动态障碍模式。
    if (export_requested) {
      exportHistoryMap();
    }

    if (import_requested) {
      importHistoryMap();
    }


    std::vector<Point> cloud_copy;
    std::vector<Point> dynamic_copy;
    std::deque<TimedCloud> history_copy;
    Config config_copy;

    // 刷新历史累计图: 清空本节点历史缓冲 + 同步整张清空 Nav2 红区, 之后随新
    //   历史图从零重建。放在最前面 (早于 received_cloud_ 检查): 清空红区不应
    //   依赖"当前是否收到点云"。异步服务调用, 不持任何互斥量。
    if (reset_history) {
      {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        history_.clear();
      }
      RCLCPP_INFO(get_logger(), "历史累计图已清空, 开始重新累积");
      clearCostmaps();
    }

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!received_cloud_) {
        return;
      }
      cloud_copy = latest_cloud_;
      pruneHistory(get_clock()->now());
      history_copy = history_;
      dynamic_copy = latest_dynamic_obstacles_;
    }

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_copy = config_;
      config_changed_ = false;
    }

    renderPgm(cloud_copy, config_copy);
    renderHistoryPgm(history_copy, config_copy);

    // 动态障碍栅格图 (仅网页绿色层显示): 仅在锁定时发布; 解锁后发一张空图让
    // 网页绿色层立即消失。注意: 供 Nav2 避障用的动态障碍点云不在这里发 ——
    // 它已在 cloudCallback 里即时发布 (见 publishDynamicObstacleCloud), 以
    // 保证红色代价区域实时跟随绿色障碍的出现与消失。
    if (dynamic_enabled || dynamic_image_published_) {
      renderDynamicObstaclePgm(dynamic_copy, config_copy);
      dynamic_image_published_ = dynamic_enabled;
    }
  }

// ===================== 源码行 3651-4133 · motionTimerCallback =====================
  void motionTimerCallback()
  {
    // 网页按钮请求: 一次触发, 在这里消费
    const bool start_requested = motion_start_requested_.exchange(false);
    const bool stop_requested = motion_stop_requested_.exchange(false);

    if (start_requested && !stop_requested) {
      startMotion();
    } else if (stop_requested) {
      stopMotion("");
    }

    // -------- 追击模式优先接管 (与占点/导航互斥) --------
    //   网页"选择追击目标"并点图后 chase_enabled 为 true, 此处每帧:
    //     1) 取最新位姿与合并占据栅格 (与导航/A* 同源);
    //     2) 重新定位目标障碍簇 (搜索半径内找不到则停车等待重新出现);
    //     3) 维持约 chase_radius 的环形跟随; 连线被挡则用 A* 绕行。
    //   全程受硬性间距 (astar_inflate_radius_) 约束, 保证不碰撞。
    {
      bool chase_on = false;

      {
        std::lock_guard<std::mutex> lock(chase_mutex_);
        chase_on = chase_enabled_;
      }

      if (chase_on) {
        bool ch_odom_ok = false;
        double ch_x = 0.0;
        double ch_y = 0.0;
        double ch_yaw = 0.0;

        {
          std::lock_guard<std::mutex> lock(motion_mutex_);
          ch_odom_ok = odom_received_;
          ch_x = odom_x_;
          ch_y = odom_y_;
          ch_yaw = odom_yaw_;
        }

        if (!ch_odom_ok) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "追击模式: 未收到里程计 (/Odometry), 停车等待");
          publishMotionFrame(false, 0.0, 0.0);
          return;
        }

        Config ch_config;
        int ch_width = 0;
        int ch_height = 0;

        {
          std::lock_guard<std::mutex> lock(config_mutex_);
          ch_config = config_;
        }

        if (!computeMapSize(ch_config, ch_width, ch_height)) {
          publishMotionFrame(false, 0.0, 0.0);
          return;
        }

        double ch_vx = 0.0;
        double ch_vy = 0.0;
        computeChaseCommand(
          ch_config, ch_width, ch_height, ch_x, ch_y, ch_yaw, ch_vx, ch_vy);

        const bool moving = std::hypot(ch_vx, ch_vy) > 1e-3;
        publishMotionFrame(moving, ch_vx, ch_vy);
        return;
      }
    }

    // -------- 占点模式优先接管 (与导航跟随互斥) --------
    //   网页"选择占点位置"并点图后 hold_point_enabled 为 true, 此处每帧:
    //     1) 取最新位姿与合并占据栅格 (与导航/A* 同源);
    //     2) 仲裁出"预测行进路线最短"的威胁障碍;
    //     3) 算出阻挡站位点并低速机动过去; 无威胁时回占点中心驻守。
    //   全程受硬性间距 (astar_inflate_radius_) 约束, 保证不碰撞。
    //   占点开启时完全接管运动指令, 因此 Nav2 的 /plan 与 A* 兜底都不参与。
    {
      bool hold_enabled = false;

      {
        std::lock_guard<std::mutex> lock(hold_point_mutex_);
        hold_enabled = hold_point_enabled_;
      }

      if (hold_enabled) {
        bool hp_odom_ok = false;
        double hp_x = 0.0;
        double hp_y = 0.0;
        double hp_yaw = 0.0;

        {
          std::lock_guard<std::mutex> lock(motion_mutex_);
          hp_odom_ok = odom_received_;
          hp_x = odom_x_;
          hp_y = odom_y_;
          hp_yaw = odom_yaw_;
        }

        if (!hp_odom_ok) {
          // 无位姿无法计算机动: 停车并保持占点态等待里程计
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "占点模式: 未收到里程计 (/Odometry), 停车等待");
          publishMotionFrame(false, 0.0, 0.0);
          return;
        }

        Config hp_config;
        int hp_width = 0;
        int hp_height = 0;

        {
          std::lock_guard<std::mutex> lock(config_mutex_);
          hp_config = config_;
        }

        if (!computeMapSize(hp_config, hp_width, hp_height)) {
          publishMotionFrame(false, 0.0, 0.0);
          return;
        }

        double hp_vx = 0.0;
        double hp_vy = 0.0;
        computeHoldPointCommand(
          hp_config, hp_width, hp_height, hp_x, hp_y, hp_yaw, hp_vx, hp_vy);

        publishHoldPointStatus();

        // control 位: 有速度才置 1, 原地驻守时下发 0 速 + control=0 更稳妥
        const bool moving = std::hypot(hp_vx, hp_vy) > 1e-3;
        publishMotionFrame(moving, hp_vx, hp_vy);
        return;
      }
    }

    bool active = false;
    bool odom_ok = false;
    bool plan_ok = false;
    bool local_plan_ok = false;
    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_yaw = 0.0;
    bool local_plan_fresh = false;
    bool plan_fresh = true;
    double plan_age = 0.0;
    double plan_timeout = 0.0;
    std::vector<Point> plan;

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active = motion_active_;
      odom_ok = odom_received_;
      plan_ok = motion_plan_received_;
      odom_x = odom_x_;
      odom_y = odom_y_;
      odom_yaw = odom_yaw_;
      // 全局路径新鲜度: Nav2 导航中会以 5 Hz 重规划 -> /plan 持续刷新;
      // 停止刷新即意味着导航已结束/中止, 此时必须停车而不是继续按过期路径跑
      plan_timeout = motion_plan_timeout_;
      if (plan_timeout > 0.0 && motion_plan_received_) {
        plan_age = (now() - motion_plan_recv_time_).seconds();
        plan_fresh = plan_age <= plan_timeout;
      }
      local_plan_ok = local_plan_received_ && local_plan_.size() >= 2;
      if (local_plan_ok && local_plan_timeout_ > 0.0) {
        local_plan_fresh = (now() - local_plan_stamp_).seconds() < local_plan_timeout_;
      } else {
        local_plan_fresh = local_plan_ok;
      }
      // 局部路径 (含避障) 新鲜时优先跟随; 否则回退全局路径。
      // 注意: 两者末端都是同一最终目标点, 故到点判定不受切换影响。
      plan = (local_plan_fresh) ? local_plan_ : motion_plan_;

      // "校验失败已改用 A* 重导航"解除条件: 车体已驶离到达半径 (远离那个偏掉的
      //   旧 Nav2 末点) -> 交还 Nav2 路径, 恢复正常跟随 (见 have_usable_plan)。
      if (motion_goal_check_replan_ && plan.size() >= 2) {
        const double c = std::cos(odom_yaw);
        const double s = std::sin(odom_yaw);
        const double bx = odom_x -
          (c * motion_base_to_lidar_x_ - s * motion_base_to_lidar_y_);
        const double by = odom_y -
          (s * motion_base_to_lidar_x_ + c * motion_base_to_lidar_y_);
        const double d_end = std::hypot(
          static_cast<double>(plan.back().x) - bx,
          static_cast<double>(plan.back().y) - by);
        if (d_end > motion_goal_tolerance_ * 1.5) {
          motion_goal_check_replan_ = false;
        }
      }
    }

    // 未开始运动: 持续下发 control=0 的静止帧, 保证底盘始终被明确约束
    if (!active) {
      publishMotionFrame(false, 0.0, 0.0);
      return;
    }

    if (!odom_ok) {
      stopMotion("未收到里程计 (/Odometry), 无法跟随路径");
      return;
    }

    // 无有效路径 (Nav2 未发 /plan, 或超时未刷新, 或点数不足):
    //   不再原地死等 —— 用自建 A* 重算一条 "当前位置 -> Web 目标点" 的兜底路径,
    //   以 astar_speed 低速跟随它逃出障碍区并继续前往目标 (这正是"从不小心进入
    //   障碍区/从障碍区开始导航"时 Nav2 /plan 会因起点落在障碍膨胀区内而规划失败、
    //   停止刷新的场景)。每帧重新规划 (按最新障碍与最新位姿), 故不会"一直沿一个
    //   方向走偏"。
    //   A* 失败 (无目标点/终点不可达/地图不可用) 时停车并保持重试: 每帧仍会重新
    //   调用 getAstarPath, 一旦障碍变化/收到 Web 目标点即可立即接管; 期间下发 0 速,
    //   由 Nav2 侧 (若已恢复) 或操作者重新下发目标兜底。
    //   一旦 Nav2 的 /plan 恢复刷新, 下一帧 have_usable_plan=true 即切回跟随 Nav2。
    //   例外: 若刚因"到达位置校验不通过"用 A* 重新导航 (motion_goal_check_replan_),
    //   则暂时无视 Nav2 路径 (其末端可能就是那个偏了的旧末点), 保持被 A* 接管,
    //   待车体驶离到达半径后自动解除 (见 motionTimerCallback 的到达判定处)。
    const bool have_usable_plan =
      plan_ok && plan_fresh && plan.size() >= 2 && !motion_goal_check_replan_;

    if (!have_usable_plan) {
      bool was_waiting = false;
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        was_waiting = motion_waiting_;
        if (!motion_waiting_) {
          motion_waiting_ = true;
        }
      }

      if (!was_waiting) {
        // 进入等待态只播报一次, 避免每帧刷屏
        if (!plan_fresh) {
          RCLCPP_WARN(
            get_logger(),
            "Nav2 已 %.1f s 未更新全局路径 (/plan, 阈值 %.1f s), "
            "进入兜底态 (自建 A* 重规划, 失败则停车并保持重试)",
            plan_age,
            plan_timeout);
        } else {
          RCLCPP_WARN(
            get_logger(),
            "暂未收到有效的 Nav2 全局路径 (/plan), 进入兜底态 (等待重新规划)");
        }
      }

      // -------- 自建 A* 重规划: 当前位置 -> Web 目标点 --------
      //   每帧评估是否重算 (符合"每帧根据最新障碍重新评估"), 用最新障碍与最新
      //   位姿; 实际搜索受 astar_replan_interval_ / astar_replan_move_dist_ 限频,
      //   避免 100 Hz 重算阻塞运动线程 (详见 getAstarPath)。
      //   规划出的路径以 astar_speed 低速跟随 (见 computeAstarFollowCommand)。
      Config astar_config;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        astar_config = config_;
      }

      int awidth = 0;
      int aheight = 0;
      std::vector<Point> astar_path;

      if (computeMapSize(astar_config, awidth, aheight) &&
        getAstarPath(
          astar_config, awidth, aheight, odom_x, odom_y, odom_yaw, astar_path))
      {
        // 发布兜底路径供网页黄色显示
        publishAstarPlan(astar_path);

        double avx = 0.0;
        double avy = 0.0;
        if (computeAstarFollowCommand(
            astar_path, odom_x, odom_y, odom_yaw, avx, avy))
        {
          // 到达兜底目标: 下发 0 速等待 Nav2 (或网页"停止运动")
          publishMotionFrame(false, 0.0, 0.0);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "A* 兜底路径已到末端, 0 速等待 Nav2 重规划 (/plan 恢复后自动跟随)");
        } else {
          publishMotionFrame(true, avx, avy);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "A* 兜底: 跟随自建路径 (%zu 点) 驶向 Web 目标 "
            "(control=1 vx=%.3f m/s vy=%.3f m/s, 巡航 %.2f m/s)",
            astar_path.size(), avx, avy, resolveAstarSpeed());
        }

        return;
      }

      // A* 失败: 清空网页上的兜底路径显示, 停车并保持重试
      //   (不退出运动态; 下一帧仍会重新 getAstarPath, 障碍变化/目标点到达
      //    后 A* 一旦成立立即恢复跟随; Nav2 的 /plan 恢复也会自动切回)。
      publishAstarPlan(std::vector<Point>{});
      publishMotionFrame(false, 0.0, 0.0);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "A* 兜底失败 (无 Web 目标点 / 终点不可达 / 地图不可用), 停车并保持重试 "
        "(Nav2 /plan 恢复或障碍消除后自动接管)");

      return;
    }

    // 有可用路径: 退出等待态, 恢复正常跟随
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      motion_waiting_ = false;
      // Nav2 已恢复供路径: 失效 A* 缓存, 下次兜底时按最新障碍重新规划
      astar_cache_valid_ = false;
    }

    double vx = 0.0;
    double vy = 0.0;
    const bool arrived = computeFollowCommand(
      plan, odom_x, odom_y, odom_yaw, vx, vy);

    // -------- 到达后的位置校验 + 校验不通过则用自建 A* 重新导航 --------
    //   到达判定 (arrived) 只看"路径末端距离", 而 Nav2 路径末点受其 tolerance
    //   (0.5 m) 影响, 可能偏离用户点的目标点 goal_x_/goal_y_。故这里额外复核
    //   "车体中心 -> 用户目标点" 的距离:
    //     通过 -> 正常停车;
    //     不通过且未超重试上限 -> 用自建 A* 从当前位置重新规划到目标并跟随;
    //     不通过且已超上限 -> 彻底停车并告警 (避免终点被封死时无限循环)。
    //   仅在"准备停车"这一刻校验一次 (不持续复核)。目标点未知 (未收到
    //   /nav_goal_pose) 或校验关闭 (motion_goal_check_tolerance <= 0) 时跳过。
    if (arrived) {
      bool goal_known = false;
      float goal_x = 0.0F;
      float goal_y = 0.0F;
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        goal_known = goal_received_;
        goal_x = goal_x_;
        goal_y = goal_y_;
      }

      // 车体中心 (雷达位姿 + 外参反推, 与 computeFollowCommand 同式)
      const double cos_yaw = std::cos(odom_yaw);
      const double sin_yaw = std::sin(odom_yaw);
      const double base_x = odom_x -
        (cos_yaw * motion_base_to_lidar_x_ - sin_yaw * motion_base_to_lidar_y_);
      const double base_y = odom_y -
        (sin_yaw * motion_base_to_lidar_x_ + cos_yaw * motion_base_to_lidar_y_);

      double goal_dist = 0.0;

      if (goal_known) {
        goal_dist = std::hypot(
          static_cast<double>(goal_x) - base_x,
          static_cast<double>(goal_y) - base_y);
      }

      const bool check_enabled =
        (goal_known && motion_goal_check_tolerance_ > 0.0);

      if (!check_enabled || goal_dist <= motion_goal_check_tolerance_) {
        // 校验通过 (或目标点未知 / 校验关闭): 正常停车
        if (check_enabled) {
          RCLCPP_INFO(
            get_logger(),
            "到达位置校验通过: 车体中心距目标点 %.3f m <= 阈值 %.3f m, 停车",
            goal_dist,
            motion_goal_check_tolerance_);
        }

        {
          std::lock_guard<std::mutex> lock(motion_mutex_);
          motion_goal_check_retry_ = 0;
        }

        stopMotion("");
        return;
      }

      // -------- 校验不通过 --------
      int retry_used = 0;
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        retry_used = motion_goal_check_retry_;
      }

      if (retry_used < motion_goal_check_max_retry_) {
        // 未超上限: 用自建 A* 从当前位置重新规划到用户目标点并继续跟随。
        //   这里不改变 motion_active_ (保持运动态), A* 缓存失效以强制按最新
        //   位置/障碍重算; 下一帧起 have_usable_plan 若仍成立会优先跟随 Nav2,
        //   但 A* 兜底会在 Nav2 未给出路径时接管驶向目标。
        {
          std::lock_guard<std::mutex> lock(motion_mutex_);
          motion_goal_check_retry_ = retry_used + 1;
          astar_cache_valid_ = false;
          motion_waiting_ = true;
          // 暂时无视 Nav2 路径, 由 A* 接管驶向目标 (见 have_usable_plan 判定)
          motion_goal_check_replan_ = true;
        }

        Config astar_config;
        {
          std::lock_guard<std::mutex> lock(config_mutex_);
          astar_config = config_;
        }

        int awidth = 0;
        int aheight = 0;
        std::vector<Point> astar_path;

        if (computeMapSize(astar_config, awidth, aheight) &&
          getAstarPath(
            astar_config, awidth, aheight, odom_x, odom_y, odom_yaw, astar_path))
        {
          publishAstarPlan(astar_path);

          double avx = 0.0;
          double avy = 0.0;

          if (computeAstarFollowCommand(
              astar_path, odom_x, odom_y, odom_yaw, avx, avy))
          {
            publishMotionFrame(false, 0.0, 0.0);
          } else {
            publishMotionFrame(true, avx, avy);
          }

          RCLCPP_WARN(
            get_logger(),
            "到达位置校验不通过: 车体中心距目标点 %.3f m > 阈值 %.3f m, "
            "用自建 A* 从当前位置重新导航到目标 (第 %d/%d 次重试)",
            goal_dist,
            motion_goal_check_tolerance_,
            retry_used + 1,
            motion_goal_check_max_retry_);

          return;
        }

        // A* 也规划不出: 保持运动态 0 速等待, 下一帧继续尝试 (与兜底态一致)
        publishAstarPlan(std::vector<Point>{});
        publishMotionFrame(false, 0.0, 0.0);

        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "到达位置校验不通过 (距目标 %.3f m > %.3f m), 但自建 A* 无法规划出通往 "
          "目标点的路径 (第 %d/%d 次重试), 停车等待",
          goal_dist,
          motion_goal_check_tolerance_,
          retry_used + 1,
          motion_goal_check_max_retry_);

        return;
      }

      // 已超重试上限: 彻底停车并告警, 避免终点被封死时无限循环
      RCLCPP_ERROR(
        get_logger(),
        "到达位置校验不通过且已重试 %d 次 (距目标点 %.3f m > 阈值 %.3f m), "
        "停止运动; 请检查目标点是否被障碍封死或手动重新下发目标",
        retry_used,
        goal_dist,
        motion_goal_check_tolerance_);

      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        motion_goal_check_retry_ = 0;
      }

      stopMotion("");
      return;
    }

    publishMotionFrame(true, vx, vy);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "运动帧: control=1 vx=%.3f m/s vy=%.3f m/s "
      "(vx=车头前方可负, vy=左手边可负, 路径 %zu 点, 来源=%s)",
      vx,
      vy,
      plan.size(),
      local_plan_fresh ? "局部(避障)" : "全局(回退)");
  }

```

## `src/parameter_callback.cpp`

建议落点：class RealtimePgmNode (参数段)

| 符号 | 源码行 | 类型 | 职责 |
|---|---|---|---|
| `parameterCallback` | 2580-3311 | 函数 | 参数热更新: 拷影子变量 -> 逐参数解析 -> 集中校验 -> 分锁提交 -> 按需重建定时器/静态基准 -> 汇总日志 |

```cpp
// ===================== 源码行 2580-3311 · parameterCallback =====================
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    Config new_config;
    int preview_max_points = cloud_preview_max_points_;
    int new_motion_control = motion_control_;
    double new_motion_vx_scale = motion_vx_scale_;
    double new_motion_vy_scale = motion_vy_scale_;
    double new_motion_rate_hz = motion_rate_hz_;
    double new_motion_target_speed = motion_target_speed_;
    double new_motion_lookahead = motion_lookahead_;
    double new_motion_lookahead_speed_gain = motion_lookahead_speed_gain_;
    double new_motion_lookahead_max = motion_lookahead_max_;
    double new_motion_yaw_kp = motion_yaw_kp_;
    double new_motion_goal_tolerance = motion_goal_tolerance_;
    double new_motion_goal_check_tolerance = motion_goal_check_tolerance_;
    int new_motion_goal_check_max_retry = motion_goal_check_max_retry_;
    double new_motion_max_lateral = motion_max_lateral_;
    double new_motion_plan_timeout = motion_plan_timeout_;
    double new_motion_slow_radius = motion_slow_radius_;
    double new_motion_min_speed = motion_min_speed_;
    double new_motion_base_to_lidar_x = motion_base_to_lidar_x_;
    double new_motion_base_to_lidar_y = motion_base_to_lidar_y_;
    bool new_astar_enabled = astar_enabled_;
    double new_astar_speed = astar_speed_;
    double new_astar_inflate_radius = astar_inflate_radius_;
    double new_astar_obstacle_penalty = astar_obstacle_penalty_;
    double new_astar_cost_reach = astar_cost_reach_;
    double new_astar_path_spacing = astar_path_spacing_;
    double new_astar_replan_interval = astar_replan_interval_;
    double new_astar_replan_move_dist = astar_replan_move_dist_;
    bool new_can_enabled = can_enabled_;
    double new_self_filtered_publish_rate = self_filtered_publish_rate_;
    int new_self_filtered_max_points = self_filtered_max_points_;
    int new_clear_ring_points = clear_ring_points_;
    double new_clear_ring_radius = clear_ring_radius_;
    bool new_hold_point_enabled = hold_point_enabled_;
    double new_hold_point_x = hold_point_x_;
    double new_hold_point_y = hold_point_y_;
    double new_hold_point_radius = hold_point_radius_;
    double new_hold_point_speed = hold_point_speed_;
    double new_hold_point_slow_radius = hold_point_slow_radius_;
    double new_hold_point_standoff_scale = hold_point_standoff_scale_;
    double new_hold_point_recenter_deadband = hold_point_recenter_deadband_;
    bool new_chase_enabled = chase_enabled_;
    double new_chase_pick_x = chase_pick_x_;
    double new_chase_pick_y = chase_pick_y_;
    double new_chase_radius = chase_radius_;
    double new_chase_deadband = chase_deadband_;
    double new_chase_speed = chase_speed_;
    double new_chase_lock_radius = chase_lock_radius_;
    double new_chase_track_radius = chase_track_radius_;

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      new_config = config_;
      preview_max_points = cloud_preview_max_points_;
    }

    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();

      try {
        if (name == "resolution") {
          new_config.resolution = parameter.as_double();
        } else if (name == "zmin") {
          new_config.zmin = parameter.as_double();
        } else if (name == "zmax") {
          new_config.zmax = parameter.as_double();
        } else if (name == "dilate") {
          new_config.dilate = parameter.as_int();
        } else if (name == "baseline_dilate") {
          new_config.baseline_dilate = parameter.as_int();
        } else if (name == "self_filter_size") {
          new_config.self_filter_size = parameter.as_double();
        } else if (name == "xmin") {
          new_config.xmin = parameter.as_double();
        } else if (name == "ymin") {
          new_config.ymin = parameter.as_double();
        } else if (name == "xmax") {
          new_config.xmax = parameter.as_double();
        } else if (name == "ymax") {
          new_config.ymax = parameter.as_double();
        } else if (name == "output") {
          new_config.output = parameter.as_string();
        } else if (name == "cloud_preview_max_points") {
          preview_max_points = parameter.as_int();
        } else if (name == "history_output") {
          new_config.history_output = parameter.as_string();
        } else if (name == "history_duration") {
          new_config.history_duration = parameter.as_double();
        } else if (name == "history_max_points_per_frame") {
          new_config.history_max_points_per_frame = parameter.as_int();
        } else if (name == "history_min_hits") {
          new_config.history_min_hits = parameter.as_int();
        } else if (name == "clear_history") {
          // 网页"刷新累计图": 置位请求, 由渲染线程清空历史点云
          if (parameter.as_bool()) {
            history_reset_requested_.store(true);
          }
        } else if (name == "export_map_dir") {
          export_map_dir_ = parameter.as_string();
        } else if (name == "export_map_name") {
          export_map_name_ = parameter.as_string();
        } else if (name == "export_map_request") {
          // 网页"导出地图": 置位请求, 由渲染线程写出 map.pgm/yaml/bin
          if (parameter.as_bool()) {
            export_map_requested_.store(true);
          }
        } else if (name == "import_map_dir") {
          import_map_dir_ = parameter.as_string();
        } else if (name == "import_map_request") {
          // 网页"导入地图": 置位请求, 由渲染线程读取导入目录恢复历史图/点云
          if (parameter.as_bool()) {
            import_map_requested_.store(true);
          }
        } else if (name == "motion_control") {
          new_motion_control = parameter.as_int();
        } else if (name == "motion_vx_scale") {
          new_motion_vx_scale = parameter.as_double();
        } else if (name == "motion_vy_scale") {
          new_motion_vy_scale = parameter.as_double();
        } else if (name == "motion_rate_hz") {
          new_motion_rate_hz = parameter.as_double();
        } else if (name == "motion_target_speed") {
          new_motion_target_speed = parameter.as_double();
        } else if (name == "motion_lookahead") {
          new_motion_lookahead = parameter.as_double();
        } else if (name == "motion_lookahead_speed_gain") {
          new_motion_lookahead_speed_gain = parameter.as_double();
        } else if (name == "motion_lookahead_max") {
          new_motion_lookahead_max = parameter.as_double();
        } else if (name == "motion_yaw_kp") {
          new_motion_yaw_kp = parameter.as_double();
        } else if (name == "motion_goal_tolerance") {
          new_motion_goal_tolerance = parameter.as_double();
        } else if (name == "motion_goal_check_tolerance") {
          new_motion_goal_check_tolerance = parameter.as_double();
        } else if (name == "motion_goal_check_max_retry") {
          new_motion_goal_check_max_retry = static_cast<int>(parameter.as_int());
        } else if (name == "motion_max_lateral") {
          new_motion_max_lateral = parameter.as_double();
        } else if (name == "motion_plan_timeout") {
          new_motion_plan_timeout = parameter.as_double();
        } else if (name == "motion_slow_radius") {
          new_motion_slow_radius = parameter.as_double();
        } else if (name == "motion_min_speed") {
          new_motion_min_speed = parameter.as_double();
        } else if (name == "motion_base_to_lidar_x") {
          new_motion_base_to_lidar_x = parameter.as_double();
        } else if (name == "motion_base_to_lidar_y") {
          new_motion_base_to_lidar_y = parameter.as_double();
        } else if (name == "astar_enabled") {
          new_astar_enabled = parameter.as_bool();
        } else if (name == "astar_speed") {
          new_astar_speed = parameter.as_double();
        } else if (name == "astar_inflate_radius") {
          new_astar_inflate_radius = parameter.as_double();
        } else if (name == "astar_obstacle_penalty") {
          new_astar_obstacle_penalty = parameter.as_double();
        } else if (name == "astar_cost_reach") {
          new_astar_cost_reach = parameter.as_double();
        } else if (name == "astar_path_spacing") {
          new_astar_path_spacing = parameter.as_double();
        } else if (name == "astar_replan_interval") {
          new_astar_replan_interval = parameter.as_double();
        } else if (name == "astar_replan_move_dist") {
          new_astar_replan_move_dist = parameter.as_double();
        } else if (name == "self_filtered_publish_rate") {
          new_self_filtered_publish_rate = parameter.as_double();
        } else if (name == "self_filtered_max_points") {
          new_self_filtered_max_points = parameter.as_int();
        } else if (name == "clear_ring_points") {
          new_clear_ring_points = parameter.as_int();
        } else if (name == "clear_ring_radius") {
          new_clear_ring_radius = parameter.as_double();
        } else if (name == "start_motion") {
          // 网页"开始运动": 置位请求, 由运动线程启动路径跟随
          if (parameter.as_bool()) {
            motion_start_requested_.store(true);
          }
        } else if (name == "stop_motion") {
          // 网页"停止运动": 置位请求, 由运动线程立即急停
          if (parameter.as_bool()) {
            motion_stop_requested_.store(true);
          }
        } else if (name == "hold_point_enabled") {
          new_hold_point_enabled = parameter.as_bool();
        } else if (name == "hold_point_x") {
          new_hold_point_x = parameter.as_double();
        } else if (name == "hold_point_y") {
          new_hold_point_y = parameter.as_double();
        } else if (name == "hold_point_radius") {
          new_hold_point_radius = parameter.as_double();
        } else if (name == "hold_point_speed") {
          new_hold_point_speed = parameter.as_double();
        } else if (name == "hold_point_slow_radius") {
          new_hold_point_slow_radius = parameter.as_double();
        } else if (name == "hold_point_standoff_scale") {
          new_hold_point_standoff_scale = parameter.as_double();
        } else if (name == "hold_point_recenter_deadband") {
          new_hold_point_recenter_deadband = parameter.as_double();
        } else if (name == "chase_enabled") {
          new_chase_enabled = parameter.as_bool();
        } else if (name == "chase_pick_x") {
          new_chase_pick_x = parameter.as_double();
        } else if (name == "chase_pick_y") {
          new_chase_pick_y = parameter.as_double();
        } else if (name == "chase_radius") {
          new_chase_radius = parameter.as_double();
        } else if (name == "chase_deadband") {
          new_chase_deadband = parameter.as_double();
        } else if (name == "chase_speed") {
          new_chase_speed = parameter.as_double();
        } else if (name == "chase_lock_radius") {
          new_chase_lock_radius = parameter.as_double();
        } else if (name == "chase_track_radius") {
          new_chase_track_radius = parameter.as_double();
        } else if (name == "dynamic_obstacle_enabled") {
          // 网页"锁定历史累计图": 锁定瞬间用当前历史图建立静态基准,
          // 此后新出现的点被判为动态障碍 (绿色、不入历史图)。
          const bool enabled = parameter.as_bool();
          if (enabled != dynamic_obstacle_enabled_.load()) {
            dynamic_obstacle_enabled_.store(enabled);
            if (enabled) {
              resetPoseCorrection();
              pose_correction_local_fail_count_.store(0);
              {
                std::lock_guard<std::mutex> lock(motion_mutex_);
                baseline_pose_x_ = odom_x_;
                baseline_pose_y_ = odom_y_;
                baseline_pose_yaw_ = odom_yaw_;
                baseline_pose_valid_ = odom_received_;
              }
              pose_correction_last_global_sec_.store(
                -std::numeric_limits<double>::infinity());
              buildStaticBaseline();
            } else {
              resetPoseCorrection();
              pose_correction_local_fail_count_.store(0);
              {
                std::lock_guard<std::mutex> lock(motion_mutex_);
                baseline_pose_valid_ = false;
              }
              std::lock_guard<std::mutex> lock(dynamic_mutex_);
              static_obstacle_mask_.clear();
              static_obstacle_tolerance_.clear();
              RCLCPP_INFO(
                get_logger(),
                "已解除历史累计图锁定: 恢复正常累积, 绿色动态障碍层将清空");
            }
          }
        } else if (name == "pose_correction_max_delta_m") {
          // 时序滤波参数: 直接更新成员 (不参与 Config 结构体)
          pose_correction_max_delta_m_ = parameter.as_double();
        } else if (name == "pose_correction_ema_alpha") {
          pose_correction_ema_alpha_ = parameter.as_double();
        } else if (name == "pose_correction_score_gain") {
          pose_correction_score_gain_ = parameter.as_double();
        } else if (name == "pose_correction_global_trigger_fails") {
          pose_correction_global_trigger_fails_ = parameter.as_int();
        } else if (name == "apply_pose_correction") {
          // 热切换: false = 修正交由外部节点 (relocalization_3d), 本节点不再
          //   估计/发布 /pose_correction, 仅订阅使用; true = 恢复内部 2D
          const bool enabled = parameter.as_bool();
          if (enabled != apply_pose_correction_) {
            apply_pose_correction_ = enabled;
            if (!enabled) {
              // 内部状态归零 (不发布); 下一帧外部修正量即覆盖
              resetPoseCorrection();
            }
            RCLCPP_INFO(
              get_logger(),
              "位姿修正来源已切换: %s",
              enabled ?
                "本节点 2D 栅格配准 (估计并发布 /pose_correction)" :
                "外部节点 (订阅 /pose_correction, 仅内部使用)");
          }
        } else if (name == "can_enabled") {
          // 热开关: true 时下一帧运动数据额外直发到 CAN 总线
          new_can_enabled = parameter.as_bool();
        } else if (name == "can_interface") {
          const std::string new_interface = parameter.as_string();
          if (new_interface != can_interface_) {
            can_interface_ = new_interface;
            closeCanSocket();
            RCLCPP_INFO(
              get_logger(),
              "CAN 接口已切换为 %s, socket 将在下次发送时重连",
              can_interface_.c_str());
          }
        } else if (name == "can_id") {
          // 标准帧 11 位 ID
          can_id_ = static_cast<canid_t>(parameter.as_int() & 0x7FF);
        } else if (name == "can_send_period_ms") {
          can_send_period_ms_ = static_cast<int>(std::clamp(parameter.as_int(), static_cast<int64_t>(1), static_cast<int64_t>(1000)));
        }
      } catch (const std::exception & e) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = e.what();
        return result;
      }
    }

    rcl_interfaces::msg::SetParametersResult result;

    if (new_config.resolution <= 0.0) {
      result.successful = false;
      result.reason = "resolution 必须大于 0";
      return result;
    }

    constexpr double kZminLimit = -10.0;
    constexpr double kZmaxLimit = 10.0;

    if (new_config.zmin < kZminLimit) {
      result.successful = false;
      result.reason = "zmin 最小支持 -10 米";
      return result;
    }

    if (new_config.zmax > kZmaxLimit) {
      result.successful = false;
      result.reason = "zmax 最大支持 10 米";
      return result;
    }

    if (new_config.zmin > new_config.zmax) {
      result.successful = false;
      result.reason = "zmin 不能大于 zmax";
      return result;
    }

    if (new_config.dilate < 0) {
      result.successful = false;
      result.reason = "dilate 不能小于 0";
      return result;
    }

    if (preview_max_points < 10) {
      result.successful = false;
      result.reason = "cloud_preview_max_points 不能小于 10";
      return result;
    }

    if (new_config.history_duration < 1.0) {
      result.successful = false;
      result.reason = "history_duration 不能小于 1 秒";
      return result;
    }

    if (new_config.history_max_points_per_frame < 100) {
      result.successful = false;
      result.reason = "history_max_points_per_frame 不能小于 100";
      return result;
    }

    if (new_config.history_min_hits < 1 ||
      new_config.history_min_hits > 255)
    {
      result.successful = false;
      result.reason = "history_min_hits 需在 1~255 之间";
      return result;
    }

    if (new_config.xmin >= new_config.xmax ||
      new_config.ymin >= new_config.ymax)
    {
      result.successful = false;
      result.reason = "地图范围必须满足 min < max";
      return result;
    }

    if (new_motion_vx_scale <= 0.0 || new_motion_vy_scale <= 0.0) {
      result.successful = false;
      result.reason = "motion_vx_scale / motion_vy_scale 必须大于 0";
      return result;
    }

    // 量化系数上限由协议量程决定 (12 位有符号量): 系数过大则 1 m/s 即溢出
    constexpr double kMotionScaleLimit = 2040.0;
    if (new_motion_vx_scale > kMotionScaleLimit ||
      new_motion_vy_scale > kMotionScaleLimit)
    {
      result.successful = false;
      result.reason = "motion_vx_scale / motion_vy_scale 不能大于 2040 "
        "(12 位有符号量程)";
      return result;
    }

    if (new_motion_rate_hz < 1.0 || new_motion_rate_hz > 200.0) {
      result.successful = false;
      result.reason = "motion_rate_hz 需在 1~200 之间";
      return result;
    }

    if (new_motion_target_speed < 0.0) {
      result.successful = false;
      result.reason = "motion_target_speed 不能为负";
      return result;
    }

    if (new_motion_lookahead <= 0.0) {
      result.successful = false;
      result.reason = "motion_lookahead 必须大于 0";
      return result;
    }

    if (new_motion_lookahead_speed_gain < 0.0) {
      result.successful = false;
      result.reason = "motion_lookahead_speed_gain 不能为负 (0 表示关闭自适应)";
      return result;
    }

    if (new_motion_goal_tolerance <= 0.0) {
      result.successful = false;
      result.reason = "motion_goal_tolerance 必须大于 0";
      return result;
    }

    // motion_goal_check_tolerance <= 0 表示关闭"到达后位置校验", 无需校验
    if (new_motion_goal_check_tolerance < 0.0 ||
      new_motion_goal_check_tolerance > 5.0)
    {
      result.successful = false;
      result.reason = "motion_goal_check_tolerance 需在 0 ~ 5 m 之间 (0 表示关闭校验)";
      return result;
    }

    if (new_motion_goal_check_max_retry < 0 ||
      new_motion_goal_check_max_retry > 100)
    {
      result.successful = false;
      result.reason = "motion_goal_check_max_retry 需在 0 ~ 100 之间 (0 表示不重试)";
      return result;
    }

    // motion_slow_radius <= 0 表示"自动 (由目标速度决定)", 此时无需校验;
    // 仅当手动指定正值时, 要求它不小于到点阈值 (否则减速段为空/反号)
    if (new_motion_slow_radius > 0.0 &&
      new_motion_slow_radius < new_motion_goal_tolerance)
    {
      result.successful = false;
      result.reason = "motion_slow_radius 需 <= 0 (自动, 由目标速度决定) "
        "或不小于 motion_goal_tolerance";
      return result;
    }

    if (new_motion_min_speed < 0.0 ||
      new_motion_min_speed > new_motion_target_speed)
    {
      result.successful = false;
      result.reason = "motion_min_speed 需在 0 ~ motion_target_speed 之间";
      return result;
    }

    // 路径新鲜度上限: <= 0 表示关闭监督, 正数上限 60 s 防止误填成毫秒
    if (new_motion_plan_timeout < 0.0 || new_motion_plan_timeout > 60.0) {
      result.successful = false;
      result.reason = "motion_plan_timeout 需在 0 ~ 60 秒之间 (0 表示关闭监督)";
      return result;
    }

    // A* 兜底规划器参数校验: <=0 合法 (表示跟随 motion_target_speed)
    if (new_astar_speed > kMotionMaxSpeed) {
      result.successful = false;
      result.reason = "astar_speed 需 <= 2 m/s (<=0 表示跟随 motion_target_speed)";
      return result;
    }

    if (new_astar_inflate_radius < 0.0 || new_astar_inflate_radius > 2.0) {
      result.successful = false;
      result.reason = "astar_inflate_radius 需在 0 ~ 2 m 之间";
      return result;
    }

    if (new_astar_obstacle_penalty < 0.0 || new_astar_obstacle_penalty > 100.0) {
      result.successful = false;
      result.reason = "astar_obstacle_penalty 需在 0 ~ 100 之间";
      return result;
    }

    if (new_astar_cost_reach < 0.0 || new_astar_cost_reach > 10.0) {
      result.successful = false;
      result.reason = "astar_cost_reach 需在 0 ~ 10 m 之间 (0 表示不产生邻近代价)";
      return result;
    }

    if (new_astar_path_spacing < 0.0 || new_astar_path_spacing > 5.0) {
      result.successful = false;
      result.reason = "astar_path_spacing 需在 0 ~ 5 m 之间 (0 表示不抽稀)";
      return result;
    }

    if (new_astar_replan_interval < 0.0 || new_astar_replan_interval > 10.0) {
      result.successful = false;
      result.reason = "astar_replan_interval 需在 0 ~ 10 秒之间 (0 表示每帧重算)";
      return result;
    }

    if (new_astar_replan_move_dist < 0.0 || new_astar_replan_move_dist > 5.0) {
      result.successful = false;
      result.reason = "astar_replan_move_dist 需在 0 ~ 5 m 之间";
      return result;
    }

    // 占点参数校验
    if (new_hold_point_radius <= 0.0 || new_hold_point_radius > 20.0) {
      result.successful = false;
      result.reason = "hold_point_radius 需在 0 ~ 20 m 之间";
      return result;
    }

    // <=0 合法: 表示跟随 motion_target_speed
    if (new_hold_point_speed > kMotionMaxSpeed) {
      result.successful = false;
      result.reason =
        "hold_point_speed 需 <= 2 m/s (<=0 表示跟随 motion_target_speed)";
      return result;
    }

    // 占点减速起始距离: <0 合法 (自动取常规的一半); =0 关闭; >0 必须 > 到点阈值
    if (new_hold_point_slow_radius > 0.0 &&
      new_hold_point_slow_radius <= motion_goal_tolerance_)
    {
      result.successful = false;
      result.reason = "hold_point_slow_radius 需 <0 (自动) 或 > 到点阈值 "
        "(motion_goal_tolerance), 否则减速段为空";
      return result;
    }

    if (new_hold_point_standoff_scale < 1.0 ||
      new_hold_point_standoff_scale > 10.0)
    {
      result.successful = false;
      result.reason = "hold_point_standoff_scale 需在 1 ~ 10 之间 (>=1 保证硬性间距)";
      return result;
    }

    if (new_hold_point_recenter_deadband < 0.0 ||
      new_hold_point_recenter_deadband > 2.0)
    {
      result.successful = false;
      result.reason = "hold_point_recenter_deadband 需在 0 ~ 2 m 之间";
      return result;
    }

    const bool rate_changed =
      std::fabs(new_motion_rate_hz - motion_rate_hz_) > 1e-6;

    // 基准图依赖栅格几何 (resolution/范围) 与膨胀量: 这些变了就必须重建
    int prev_baseline_dilate = 0;
    bool baseline_geometry_changed = false;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      prev_baseline_dilate = config_.baseline_dilate;
      baseline_geometry_changed =
        std::fabs(new_config.resolution - config_.resolution) > 1e-9 ||
        std::fabs(new_config.xmin - config_.xmin) > 1e-9 ||
        std::fabs(new_config.ymin - config_.ymin) > 1e-9 ||
        std::fabs(new_config.xmax - config_.xmax) > 1e-9 ||
        std::fabs(new_config.ymax - config_.ymax) > 1e-9;
    }

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_ = new_config;
      config_changed_ = true;
      cloud_preview_max_points_ = preview_max_points;
      self_filtered_publish_rate_ = new_self_filtered_publish_rate;
      self_filtered_max_points_ = new_self_filtered_max_points;
      clear_ring_points_ = new_clear_ring_points;
      clear_ring_radius_ = new_clear_ring_radius;
    }

    can_enabled_ = new_can_enabled;

    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      motion_control_ = new_motion_control;
      motion_vx_scale_ = new_motion_vx_scale;
      motion_vy_scale_ = new_motion_vy_scale;
      motion_rate_hz_ = new_motion_rate_hz;
      motion_target_speed_ = new_motion_target_speed;
      motion_lookahead_ = new_motion_lookahead;
      motion_lookahead_speed_gain_ = new_motion_lookahead_speed_gain;
      motion_lookahead_max_ = new_motion_lookahead_max;
      motion_yaw_kp_ = new_motion_yaw_kp;
      motion_goal_tolerance_ = new_motion_goal_tolerance;
      motion_goal_check_tolerance_ = new_motion_goal_check_tolerance;
      motion_goal_check_max_retry_ = new_motion_goal_check_max_retry;
      motion_max_lateral_ = new_motion_max_lateral;
      motion_plan_timeout_ = new_motion_plan_timeout;
      motion_slow_radius_ = new_motion_slow_radius;
      motion_min_speed_ = new_motion_min_speed;
      motion_base_to_lidar_x_ = new_motion_base_to_lidar_x;
      motion_base_to_lidar_y_ = new_motion_base_to_lidar_y;
      astar_enabled_ = new_astar_enabled;
      astar_speed_ = new_astar_speed;
      astar_inflate_radius_ = new_astar_inflate_radius;
      astar_obstacle_penalty_ = new_astar_obstacle_penalty;
      astar_cost_reach_ = new_astar_cost_reach;
      astar_path_spacing_ = new_astar_path_spacing;
      astar_replan_interval_ = new_astar_replan_interval;
      astar_replan_move_dist_ = new_astar_replan_move_dist;
      // 规划相关参数变化 -> 失效缓存, 下一帧按新参数重算
      astar_cache_valid_ = false;
    }

    // 占点参数提交: 关闭占点时立即回到普通导航逻辑 (运动线程每帧读取该标志)
    if (new_hold_point_enabled != hold_point_enabled_ ||
      std::fabs(new_hold_point_radius - hold_point_radius_) > 1e-9 ||
      std::fabs(new_hold_point_speed - hold_point_speed_) > 1e-9 ||
      std::fabs(new_hold_point_slow_radius - hold_point_slow_radius_) > 1e-9 ||
      std::fabs(new_hold_point_standoff_scale - hold_point_standoff_scale_) >
      1e-9 ||
      std::fabs(new_hold_point_recenter_deadband -
      hold_point_recenter_deadband_) > 1e-9)
    {
      std::lock_guard<std::mutex> lock(hold_point_mutex_);
      const bool was_enabled = hold_point_enabled_;
      hold_point_radius_ = new_hold_point_radius;
      hold_point_speed_ = new_hold_point_speed;
      hold_point_slow_radius_ = new_hold_point_slow_radius;
      hold_point_standoff_scale_ = new_hold_point_standoff_scale;
      hold_point_recenter_deadband_ = new_hold_point_recenter_deadband;
      hold_point_enabled_ = new_hold_point_enabled;

      if (new_hold_point_enabled != was_enabled) {
        // 切换占点开关: 清空威胁记忆, 避免沿用上一段的过期仲裁结果
        hold_point_threat_valid_ = false;
        RCLCPP_INFO(
          get_logger(),
          "占点模式 %s (占点中心=(%.2f, %.2f) m, 范围=%.2f m)",
          new_hold_point_enabled ? "已启用" : "已关闭",
          hold_point_x_, hold_point_y_, new_hold_point_radius);
      }
    }

    // 占点中心变更 (网页点击新的占点位置): 立即生效, 清空威胁记忆重新评估
    if (std::fabs(new_hold_point_x - hold_point_x_) > 1e-9 ||
      std::fabs(new_hold_point_y - hold_point_y_) > 1e-9)
    {
      std::lock_guard<std::mutex> lock(hold_point_mutex_);
      hold_point_x_ = new_hold_point_x;
      hold_point_y_ = new_hold_point_y;
      hold_point_threat_valid_ = false;
      RCLCPP_INFO(
        get_logger(),
        "占点中心已更新: (%.2f, %.2f) m",
        hold_point_x_, hold_point_y_);
    }

    // 追击参数提交: 开关/半径/速度等热更新
    if (new_chase_enabled != chase_enabled_ ||
      std::fabs(new_chase_radius - chase_radius_) > 1e-9 ||
      std::fabs(new_chase_deadband - chase_deadband_) > 1e-9 ||
      std::fabs(new_chase_speed - chase_speed_) > 1e-9 ||
      std::fabs(new_chase_lock_radius - chase_lock_radius_) > 1e-9 ||
      std::fabs(new_chase_track_radius - chase_track_radius_) > 1e-9)
    {
      std::lock_guard<std::mutex> lock(chase_mutex_);
      const bool was_enabled = chase_enabled_;
      chase_radius_ = new_chase_radius;
      chase_deadband_ = new_chase_deadband;
      chase_speed_ = new_chase_speed;
      chase_lock_radius_ = new_chase_lock_radius;
      chase_track_radius_ = new_chase_track_radius;
      chase_enabled_ = new_chase_enabled;

      if (new_chase_enabled != was_enabled) {
        RCLCPP_INFO(
          get_logger(),
          "追击模式 %s (环带半径=%.2f m, 锁定半径=%.2f m)",
          new_chase_enabled ? "已启用" : "已关闭",
          new_chase_radius, new_chase_lock_radius);
      }
    }

    // 追击目标点选变更 (网页点击新的目标位置): 标记需要重新锁定目标簇
    if (std::fabs(new_chase_pick_x - chase_pick_x_) > 1e-9 ||
      std::fabs(new_chase_pick_y - chase_pick_y_) > 1e-9)
    {
      std::lock_guard<std::mutex> lock(chase_mutex_);
      chase_pick_x_ = new_chase_pick_x;
      chase_pick_y_ = new_chase_pick_y;
      // 重新锁定: 清空旧目标记忆与轨迹, 由追击主逻辑按新点就近锁定目标簇
      chase_target_valid_ = false;
      chase_track_.clear();
      RCLCPP_INFO(
        get_logger(),
        "追击目标点已更新: (%.2f, %.2f) m, 将按该点就近重新锁定目标",
        chase_pick_x_, chase_pick_y_);
    }

    if (rate_changed) {
      createMotionTimer();
    }

    // 锁定状态下改了影响基准的配置 (分辨率/范围/基准膨胀) 时必须重建基准,
    // 否则基准仍按旧几何索引, 判定会整体错位 -> 障碍识别失效
    if (dynamic_obstacle_enabled_.load() &&
      (baseline_geometry_changed || new_config.baseline_dilate != prev_baseline_dilate))
    {
      buildStaticBaseline();
    }

    RCLCPP_INFO(
      get_logger(),
      "参数已热更新: resolution=%.4f, zmin=%.3f, zmax=%.3f, "
      "dilate=%d, "
      "baseline_dilate=%d, self_filter_size=%.2f, "
      "history_duration=%.1f, history_min_hits=%d, "
      "motion(control=%d, speed=%.2f m/s, lookahead=%.2f m, rate=%.1f Hz)",
      new_config.resolution,
      new_config.zmin,
      new_config.zmax,
      new_config.dilate,
      new_config.baseline_dilate,
      new_config.self_filter_size,
      new_config.history_duration,
      new_config.history_min_hits,
      new_motion_control,
      new_motion_target_speed,
      new_motion_lookahead,
      new_motion_rate_hz);

    result.successful = true;
    result.reason = "参数更新成功";
    return result;
  }

```

## 附录：成员变量区（20 组）的分配参考

`config/` 只放 yaml，所以成员变量分三种去向：① **可调参数** → `config/params.yaml` 的键；② **纯数据/结构** → `tools/types.hpp`；③ **运行期状态**（mutex / 队列 / 缓存 / 发布器）→ 对应 `io/` 或 `tasks/` 类的私有成员。逐组源码见 `05_node.md` 的附录。

| 成员变量组 | 源码行 | 建议落点 |
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
