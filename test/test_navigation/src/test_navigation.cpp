#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

// SocketCAN: 直接把运动帧写进 CAN 总线 (与 scripts 无关, 原生命令)
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
// 占点状态发布 (JSON 字符串): /hold_point_status, 供网页展示威胁/阻挡信息
#include <std_msgs/msg/string.hpp>
// Nav2 代价地图"整张清空"服务 (刷新历史累计图时同步清空红色代价区)
#include <nav2_msgs/srv/clear_entire_costmap.hpp>


class RealtimePgmNode : public rclcpp::Node
{
public:
  // 运动帧速度编码 (与接收方 CAN_ID=0xFF 协议保持一致):
  // 12 位有符号原始量, 接收方按 raw / 200.0f 还原为 m/s。
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

  struct Point
  {
    float x;
    float y;
    float z;
  };

  // 路径跟随的可视化快照 (供网页叠加绘制), 全部为 map 系量 (m / rad / m/s):
  //   base      = 当前车体中心位置
  //   yaw       = 车头航向
  //   target    = 由"最近点沿弧长推进 lookahead"得到的当前前视点
  //   vx / vy   = 下发到车体系的运动帧速度 (vx=前方, vy=左手边)
  //   valid     = 仅在运动跟随中为 true, 停车后即使置否
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

  // 带时间戳的点云缓存: 用于历史地图的滑动时间窗累积
  struct TimedCloud
  {
    rclcpp::Time stamp;
    std::shared_ptr<const std::vector<Point>> points;
  };

  RealtimePgmNode()
  : Node("test_navigation")
  {
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

private:
  // ==================== 调试日志 / 异常快照 ====================
  // 逐帧把点云统计写入 log/cloud_frames.csv, 异常帧 (点数突增/质心跳变) 额外
  //   存一份 PCD 快照到 log/。目的是"雷达突然全是障碍"这类问题能事后溯源:
  //   到底是点数暴增、点云整体位移 (LIO 跳变), 还是 z 范围异常。

  // 初始化日志目录与 CSV 文件 (表头)
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

  // 保存一帧点云为 PCD (ascii) 快照, 返回是否成功
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

  // 逐帧记录点云统计并做异常检测
  //   raw_points:   PointCloud2 原始点数 (width*height)
  //   valid_points: 解析且 isfinite 的点数
  //   self_filtered: 车体自过滤剔除数
  //   queued:       进入历史缓冲的点数 (0 表示不累积)
  //   cloud:        自过滤后的点云 (用于质心/包围盒与快照)
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

  // ---------------- 里程计 (雷达在 map 系的位姿) ----------------
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

  // ---------------- Nav2 全局路径 (map 系) ----------------
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

  // ---------------- Nav2 局部路径 (controller_server 的 /local_plan) ----------------
  // 与全局路径解析完全一致, 只是缓存到独立成员; 新鲜度由
  // motionTimerCallback 用 local_plan_timeout_ 判定, 超时回退全局路径。
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

  // ---------------- Web 目标点 (网页点击时发布, map 系) ----------------
  // 供自建 A* 兜底规划器作为终点。仅取位置 (x, y), 朝向不参与 A* 规划。
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

  // ---------- 位姿修正量 (map->odom) ----------
  // 语义: odom 系 -> map 系的刚体变换, p_map = R * p_odom + t。
  //   车体自身漂移 (含姿态倾斜与高度漂移) 全部由该 SE(3) 修正补偿, 因此不再局限于
  //   原有的"平面内 x/y/yaw"三自由度 —— 这是 3D 配准能修好"摇晃后歪掉/沉掉"的前提。
  struct PoseCorrection
  {
    double r[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};   // 行主序
    double t[3] = {0.0, 0.0, 0.0};
    bool valid = false;
  };

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

  // 纯 z 轴旋转的偏航角 (R = Rz(yaw))
  static double yawFromMatrix(const double r[9])
  {
    return std::atan2(r[3], r[0]);
  }

  // 角度归一化到 (-pi, pi]
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

  // 3x3 矩阵相乘 (行主序): out = a * b
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

  PoseCorrection getPoseCorrection()
  {
    std::lock_guard<std::mutex> lock(pose_correction_mutex_);
    return pose_correction_;
  }

  // 把修正量发布出去 (供 nav2_bridge.py 广播 map->odom TF 与网页显示)
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

  // builtin2d 路径: 纯 (x, y, yaw) -> SE(3), 并广播
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

  // 清理超出保留时长的历史点云
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

  // 动态障碍分离: 把"锁定时刻静态基准图上未被占据"的点判为临时障碍。
  //   输入 cloud 原地保留静态点 (即剔除动态障碍后的结果),
  //   输出 dynamic 为被判为动态障碍的原始点。
  //   判定依据是冻结的静态基准栅格 static_obstacle_mask_ (锁定那一刻由
  //   当前历史累积图按 history_min_hits 投票 + dilate 生成), 因此:
  //     - 基准图上已占据的格 -> 静态环境, 留在单帧图/历史图里;
  //     - 基准图上未占据 (含未观测) 的格 -> 新出现的点即临时障碍。
  //   调用方在分离前已拷贝了完整帧供绿色层使用 (dynamic_obstacles 只作避障点云),
  //   故这里就地剔除动态点对可视化/避障均无副作用。
  //   基准图为空 (尚未锁定) 时不做任何分离, 保证非锁定模式行为不变。
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

  // 基准扣除: 从"整帧点云"里剔除落在冻结静态基准 (static_obstacle_mask_) 上的点,
  //   剩余的点即为绿色层要显示的动态障碍候选。
  //   这是"绿色层与建图共用同一帧点云"的实现方式: 绿色层不再用 splitDynamicObstacles
  //   剥出的半边, 而是用整帧点云减去基准占据 —— 与建图看到的点云同源, 只把
  //   已确认为静态环境的格屏蔽掉。
  //   与 splitDynamicObstacles 的判据保持完全一致 (同一份 baseline / 同一套 z 与
  //   栅格几何), 唯一区别: 本函数不做原地剔除, 只输出"非基准"的点, 不改变输入。
  //   基准为空 (尚未锁定) 时直接返回整帧点云 (锁定前绿色层不发布, 无副作用)。
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

  // 由当前历史累积图生成"静态基准栅格" (锁定时刻调用一次)。
  //   与 renderHistoryPgm 同一套判定: 命中帧数 >= history_min_hits 即为占据,
  //   再按 dilate 膨胀 (膨胀让基准更宽容, 避免车体点/边缘点被误判为动态)。
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

  // 发布降采样后的点云预览 (供网页端绘制可交互侧视图)
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

  // 供 Nav2 obstacle_layer 消费的点云时间戳。
  //   Nav2 每条观测源订阅都挂着 tf2_ros::MessageFilter: 它要求"点云戳对应的 TF
  //   已经存在", 查不到就整帧丢弃并打印
  //     "Message Filter dropping message: frame 'base_link' ... for reason
  //      'Lookup would require extrapolation into the future' / 'earlier than all
  //      the data in the transform cache'"
  //   base_link 的 TF 由 nav2_bridge 用 /Odometry 的戳广播, 而 /Odometry 的戳是
  //   雷达一帧结束时刻, 通常比系统时间晚 0.1~0.3 s 才发出来 —— 实测最新 TF 只到
  //   now-0.16 s。若用 get_clock()->now() 打戳, 必定是"向未来外推" → 点云全被丢,
  //   表现为: 绿色动态障碍不参与规划 (marking 失效)、旧红区清不掉 (clearing 失效)。
  //   因此统一用"最新里程计戳 - 余量": 该时刻的 TF 一定已经在缓存里 (10 s 窗口内
  //   越早越稳), 又留出余量避免"我们的点云比 nav2_bridge 的 TF 先到"的竞态。
  //   余量 0.1 s 对应的位置误差 = 速度 × 0.1 s (0.5 m/s → 5 cm), 可忽略。
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

  // 把点云按 frame_id 原样发布 (不做降采样), 供 Nav2 obstacle_layer 订阅。
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

  // 把 map 系点云真实变换到 base_link 系, 供 Nav2 obstacle_layer 使用。
  // Nav2 的 ObservationBuffer 会把 PointCloud2 坐标当作 base_link 系数值, 再叠一次
  // base_link->global_frame 的 TF (机器人当前位姿)。因此必须在发布前先把 map 系坐标
  // 变换到 base_link 系, 否则机器人位姿被施加两次, 整张点云被搬到车体上并随航向旋转。
  // 车体中心由里程计位姿按外参反推, 与 computeVehicleFootprint 使用同一公式:
  //   base = lidar_odom - R(yaw) * motion_base_to_lidar
  // 返回 false 表示无里程计 (调用方应退回 map 帧)。
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

  // 发布"车体自过滤后的完整当前帧点云"到 self_filtered_topic_, 供 Nav2
  // obstacle_layer 的 pointcloud 源做【纯 clearing】(nav2_params.yaml 里该源
  // marking:false / clearing:true)。它是全场清除射线的唯一来源:
  // dynamic_obstacles 源在障碍消失后就没有点, 那个方向清不动; 本源的完整帧
  // 点云覆盖全 360°, 才能把旧红格彻底清掉。
  //   关键点 1: frame_id 必须是 base_link, 且点坐标必须真实变换到 base_link 系。
  //   Nav2 的 ObservationBuffer 会把 PointCloud2 的坐标当作 base_link 系数值, 再做一次
  //   base_link->global_frame 的 TF (机器人当前位姿)。若只改 frame_id 不改数值 (把 map
  //   系数值谎报成 base_link), 机器人位姿会被施加两次: 整张点云被搬到车体上并随航向
  //   旋转, 全局代价地图车周出现假障碍环, 局部代价地图只显示地图原点附近内容。
  //   关键点 2: 变换成 base_link 系后, Nav2 再做一次 TF 即回到 map 系正确位置; 且清除
  //   射线起点为车体当前位置, 能正确清掉绕车轨迹上的旧障碍。
  //   关键点 3: 限频 + 抽稀。整帧点云动辄上万点, 按点云帧率全量发布会给 TF 变换
  //   和 DDS 带来不必要压力; 限频到 self_filtered_publish_rate_ 即可满足清除需求。
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

  // 整张清空 Nav2 局部/全局代价地图 (调 ClearEntireCostmap 服务)。
  //   服务端是同步实现, 故全部异步调用 (async_send_request + 结果回调), 不在
  //   渲染线程里等待, 避免阻塞点云/渲染周期。服务端未就绪时给出 WARN 并跳过。
  //   与"刷新历史累计图"配合: 历史图清空后红区也整张清空, 再随新历史图从零累积。
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

  // ==================== 导航运动控制 ====================
  // 组装一帧 8 字节运动数据包并发布 (与接收方 CAN_ID=0xFF 的 Data[0..7] 对应):
  //   [0] bit0=control  bit1=shoot
  //   [1-2] tar_yaw(H/L)   [3-4] tar_pitch(H/L)   [5-7] vx/vy 12 位打包
  // 导航只使用 control / vx / vy 三个字段, 其余按协议占位填 0。
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

  // ---------------- SocketCAN 直发 ----------------
  // 关闭 socket (切换接口或发送失败重连时调用)
  void closeCanSocket()
  {
    if (can_socket_fd_ >= 0) {
      ::close(can_socket_fd_);
      can_socket_fd_ = -1;
    }
  }

  // 打开/重连 SocketCAN, 失败时返回 false (下个周期会再试)
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

  // 把 8 字节运动数据按 can_send_period_ms 限频直发到 CAN 总线
  // (运动帧本身就是 100 Hz / 10 ms 下发, 这里再做一次节流保证上限)
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

  // 保方向限幅: 若 max(|vx|, |vy|) 超过单轴上限, 则按同一比例等比缩小两轴,
  // 保持速度方向不变 (例如 (10.2, 5.1) -> (1.0, 0.5), 而非逐轴截断成 (1.0, 1.0))。
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

  // 减速起始距离 (m): slow_radius > 0 时用手动值, 否则由目标速度决定
  //   = max(到点阈值 + 0.05, kSlowRadiusTime * 目标速度)
  // 即默认速度越大, 越早开始减速; 下限保证 "阈值 ~ 减速起点" 区间不退化。
  static double effectiveSlowRadius(
    double slow_radius, double target_speed, double goal_tolerance)
  {
    if (slow_radius > 0.0) {
      return slow_radius;
    }

    const double speed = std::clamp(target_speed, 0.0, 5.0);
    return std::max(goal_tolerance + 0.05, kSlowRadiusTime * speed);
  }

  // 速度(m/s, 有符号) -> 12 位有符号原始量: 量化步长 1/scale (scale 默认 200, 单位 m/s)
  // 注意: 单轴上限已由 limitMotionSpeed() 在打包前统一处理, 这里只做量化与量程饱和。
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

  // 按 motion_rate_hz 重建下发定时器 (网页滑条热改频率时调用)
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

  // 路径跟随: 取路径前视点, 换算到车体系得到 vx/vy
  //   坐标系约定 (运动帧协议):
  //     vx > 0 = 车头前方 (可为负 => 后退)
  //     vy > 0 = 车辆左手边 (可为负 => 向右)。注意: 由雷达里程计投影出的
  //              横向分量是"右手边为正", 下发前需取反 (见步骤 5)。
  //   全向底盘 + 车头朝向保持不动: 速度矢量直接指向"车体中心 -> 前视点",
  //   故 (vx, vy) 即该单位矢量在车体系下的两个分量, 不引入任何转向增益。
  //   前视点选取: 最近点优先在车头前方半平面内搜索, 再从最近点弧长沿路径
  //   推进 lookahead; 前视点落入车体后半平面 (需倒车) 时速度钳到
  //   motion_min_speed, 不满速倒退。
  //   雷达为绕 X 轴 180 度倒装 (头朝地面), 水平朝向与车头一致。
  //   该翻转对位置的影响已由外参 motion_base_to_lidar_{x,y} 吸收 (步骤 0);
  //   对速度的影响是横向分量符号相反, 由步骤 5 的 left = -veh_right 处理。
  //   —— 即"翻转"需要两处配合纠正, 只改外参而漏改 vy 符号 (或反之) 都会出错。
  //   所有几何量 (到达判定/最近点/前视点) 统一以车体中心为参考点。
  // 构造"合并占据栅格" = 历史累计栅格 ∪ 绿色动态障碍栅格 (与 renderHistoryPgm /
  //   renderDynamicObstaclePgm 同口径), 并扣除车体自身方框。
  //   供"自建 A* 兜底规划器 (planAstarPath)"复用, 保证与网页显示的是同一份障碍图。
  //   occupancy 尺寸 = width * height, 非 0 表示占据。
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

  // ============================================================================
  // 自建 A* 兜底规划器
  // ============================================================================
  // 动机: Nav2 的 /plan 在"车体起点落在障碍膨胀区内"时会规划失败并停止刷新,
  //   此时用节点内自建的 A* 在"合并占据栅格" (历史累计 + 绿色动态, 见
  //   buildMergedOccupancy) 上重算一条 "当前车体位置 -> Web 页面选定目标点" 的
  //   路径, 以 astar_speed_ 低速跟随, 逃出障碍并继续前往目标; Nav2 的 /plan
  //   一旦恢复刷新即切回跟随 Nav2。
  //
  // 算法要点:
  //   - 障碍先按 astar_inflate_radius_ (0.50 m, 车体中心口径) 向外膨胀, 保证路径
  //     与障碍保持硬性最小间距 (与 Nav2 的 robot_radius 口径统一);
  //   - 代价项加入"靠近障碍惩罚" (astar_obstacle_penalty_): 越贴近障碍代价越高,
  //     影响范围为 astar_cost_reach_ (0.60 m), 引导路径走宽阔通路而非贴边
  //     (对应"A* + 靠障碍代价惩罚");
  //   - 起点允许位于障碍格内 (车在障碍里时): 起点格代价强制置 0 视为可通行,
  //     从而能"从障碍内起步, 向最近可行格找到出口";
  //   - 八连通 (允许斜向) + 对角穿越障碍格保护, 避免穿墙角;
  //   - 终点不可达时返回 false, 调用方停车并保持重试。
  //
  // 输出: path (map 系点列, 起点为车体中心, 末点为 Web 目标点附近最接近的可行格),
  //   已按 astar_path_spacing_ 抽稀。返回 true 表示规划成功且 path.size() >= 2。
  //   (终点固定为 Web 目标点 goal_x_/goal_y_; 任意终点的版本见 planAstarToGoal。)
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

  // A* 核心: 由"车体中心 (base_x, base_y)"规划一条在合并占据栅格上绕开障碍、
  //   保持硬性最小间距 (astar_inflate_radius_) 到达任意世界目标点 (goal_wx,
  //   goal_wy) 的路径。抽出自 planAstarPath, 供:
  //     - 导航兜底 (终点 = Web 目标点 goal_x_/goal_y_);
  //     - 追击模式绕障 (终点 = 站位点, 目标时刻变化)。
  //   起点允许位于障碍格内 (车在障碍里时强制放行), 终点被障碍覆盖时就近取可行格。
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

  // 带限频的自建 A* 规划入口 (运动线程每帧调用):
  //   决策是否真跑 A*:
  //     - 无缓存 -> 必须规划;
  //     - 车体中心较上次规划点移动 >= astar_replan_move_dist_ -> 必须规划;
  //     - 距上次规划 >= astar_replan_interval_ -> 必须规划 (吸收障碍更新);
  //     - 否则复用上次路径 (避免 100 Hz 重算阻塞运动线程)。
  //   astar_replan_interval_ <= 0 时表示不限频 (每帧都规划)。
  //   返回 true 表示 path 已填充有效路径 (>= 2 点); false 表示无可用路径。
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

  // 把自建 A* 路径发布到 astar_plan_topic_ (nav_msgs/Path, map 系), 供网页黄色显示。
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

  // ============================================================================
  // 占点 (hold point) 行为
  // ============================================================================
  // 语义: 网页先点"选择占点位置", 再点历史累计图上的目标位置 -> 该点成为
  //   "占点中心"。车辆自动前往并驻守中心; 同时每帧在占点范围内找出"到中心直线
  //   距离最近"的障碍, 抢先机动到"该障碍 -> 中心"连线上、尽可能贴近该障碍的
  //   位置, 用车体挡住 (物理阻挡)。不依赖任何障碍速度/运动预测, 纯几何位置仲裁。
  //
  // 安全约束 (与当前导航口径完全一致, 不另设一套):
  //   车体旋转轴中心 -> 障碍的硬性最小间距 = astar_inflate_radius_ (0.50 m),
  //   该值与 Nav2 的 robot_radius 同口径。任何机动若会使车体中心与障碍的距离
  //   小于该值, 则该机动被拒绝 (降级为停车/后退), 因此占点过程中不会发生碰撞。
  //
  // 数据结构来源: 障碍取自 buildMergedOccupancy 的合并占据栅格 (历史累计 +
  //   绿色动态障碍 + 车体扣除), 与导航/A* 用的是同一张图, 保证"占点看到的障碍"
  //   与"导航避开的障碍"完全一致。
  // ============================================================================

  // 障碍簇 (世界系): 用于威胁仲裁与阻挡站位点计算。
  //   仅保留质心坐标; 不做任何运动/速度预测, 威胁仲裁用"到占点中心的直线距离"
  //   最近者优先 (见 computeHoldPointCommand)。
  struct HoldObstacleCluster
  {
    double x{0.0};
    double y{0.0};
  };

  // 由合并占据栅格提取"障碍簇" (连通域质心), 用于威胁仲裁。
  //   只做粗粒度聚类: 网格步长 kClusterStep (格), 落在占点范围
  //   外扩一个安全边距的簇直接剔除 (范围外障碍不是威胁)。
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

  // 计算车体为阻挡某障碍应到达的"站位点":
  //   在"障碍 -> 占点中心"连线上, 距障碍 hold_point_standoff_scale_ * 硬性间距
  //   处。该点保证车体中心到障碍的距离 >= 硬性最小间距 (不碰撞), 同时正好卡在
  //   障碍通往中心的必经路径上 —— standoff_scale 越接近 1, 站位点越贴近敌人,
  //   阻挡越"顶格", 但不小于硬性间距。
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

  // 点对点硬性间距校验: 判断 (x,y) 到"除豁免障碍簇以外"的最近占据格距离是否
  //   >= safe_dist。用于阻挡站位点自检 —— 站位点本就刻意贴近目标敌人, 目标敌人
  //   自身的占据格不能算作"碰撞", 否则会与站位点定义自相矛盾 (必然失败退中心)。
  //   豁免方式: 只忽略落在 (obs_x, obs_y) 半径 exempt_radius 内的占据格 (即目标
  //   敌人本体), 其余障碍 (墙/其他敌人) 一律正常参与判定, 保证不撞墙/不撞旁人。
  //   地图外一律视为不可用 (返回 false)。
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

  // 沿线段做"硬性间距"校验 (带豁免): 在 [x0,y0] -> [x1,y1] 上等间隔采样, 每个
  //   采样点周围 safe_dist 内不得有"未被豁免"的占据格。豁免规则与 isHoldPointClear
  //   一致: 只忽略落在 (obs_x, obs_y) 半径 exempt_radius 内的占据格 (目标敌人本体)。
  //   用于机动前向预判, 与导航口径一致 (safe_dist = astar_inflate_radius_)。
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

  // 占点主逻辑: 每帧在运动定时器里调用 (motionTimerCallback)。
  //   输出 vx/vy (车体系速度) 与是否已接管 (true 时调用方不再走导航跟随)。
  //   返回 true 表示"占点模式已处理本帧运动指令", 调用方应直接下发并返回。
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

  // 发布占点可视化: Path 第 1 点 = 占点中心, 第 2 点 = 本帧期望到达点
  //   (阻挡站位点或中心自身)。网页据此画圈 + 连线。
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

  // 发布占点状态 JSON (供网页显示"正在阻挡/驻守/威胁距离"等)
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

  // ============================================================================
  // 追击 (chase) 模式
  // ============================================================================
  // 语义: 网页"锁定历史累计图"后, 点"选择追击目标"进入选点态, 再点历史图上的
  //   目标物体 -> 锁定距该点 chase_lock_radius_ 内最近的障碍簇 (复用
  //   extractHoldObstacles 粗聚类质心) 作为追击目标。此后每帧:
  //     1) 以上一帧目标位置为中心, 在 chase_track_radius_ 内重新定位最近障碍簇
  //        (跟踪同一物体, 防止被周围障碍抢占);
  //     2) 记录目标位置 (节流) 与目标簇 3D 点云, 形成轨迹缓存;
  //     3) 维持"车体中心到目标约 chase_radius_"的环形跟随: 太近退、太远进;
  //        若车体与目标连线被障碍挡住, 则用 A* 绕开障碍驶向站位点;
  //     4) 目标丢失 (搜索半径内找不到障碍簇) 时直接停车, 等待其重新出现;
  //        丢失后的重寻/搜索点逻辑已移除 (以后再实现)。
  //   控制复用 computeFollowCommand (与占点完全同一套纯方向跟踪), 安全口径与
  //   导航/占点一致: 硬性最小间距 = astar_inflate_radius_。
  //
  // 返回 true 表示"追击模式已处理本帧运动指令", 调用方应直接下发并返回。

  // 在当前合并占据栅格上, 以 (ref_x, ref_y) 为中心、radius 为半径, 取最近的障碍
  //   簇质心。found=false 表示范围内没有障碍簇。
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

  // 判断"车体中心 -> 目标"两点连线上是否有障碍 (硬性间距口径)。
  //   用于决定是否需要绕障: 连线通畅则直连站位点, 否则用 A* 规划绕行。
  //   目标本体允许落在连线末端 (豁免), 避免把目标自身判成"挡路障碍"。
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

  // 追击主逻辑: 每帧在运动定时器里调用 (motionTimerCallback)。
  //   输出 vx/vy (车体系速度) 与是否已接管 (true 时调用方不再走导航跟随)。
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

  // 记录目标簇附近 (target_x, target_y) 邻域内 (半径约 radius*1.5) 的 3D 点云,
  //   追加到目标轨迹缓存 (带位置与时间戳), 并按时长淘汰旧记录。
  //   3D 点云来源: 当前完整点云 latest_cloud_ (含静态+动态), 保留其真实 z 值。
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

  // 发布追击可视化: 第 1 点=当前目标, 第 2 点=期望站位点。
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

  // 发布追击状态 JSON (供网页显示目标位置/丢失/搜索状态)
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

  // 发布目标历史轨迹 (nav_msgs/Path, map 系): 供网页画目标运动轨迹线
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

  // 跟随自建 A* 兜底路径: 与 computeFollowCommand 完全同算法, 唯一区别是巡航
  //   速度用 astar_speed_ (<=0 时跟随 motion_target_speed_, 与 Nav2 正常速度一致)。
  //   返回 true 表示已到达兜底路径末端 (应停车等待 Nav2)。
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

  // 解析 A* 兜底速度: astar_speed_ <= 0 时跟随 motion_target_speed_ (与正常
  //   巡航速度一致), 否则用手动指定值。统一在此处解析, 避免多处口径不一致。
  double resolveAstarSpeed() const
  {
    const double base = (astar_speed_ > 0.0) ? astar_speed_ : motion_target_speed_;
    return std::clamp(base, 0.0, kMotionMaxSpeed);
  }

  // 返回 true 表示已到达终点 (应停车)
  //   cruise_speed_override: > 0 时用该速度作为巡航速度 (自建 A* 兜底低速跟随用);
  //     <= 0 时用 motion_target_speed_ (Nav2 路径正常跟随)。
  //   slow_radius_override: 覆盖"接近终点线性减速"的减速起始距离 (m):
  //     < 0  -> 用默认 (motion_slow_radius_ / 由目标速度自动决定);
  //     = 0  -> 完全关闭梯形减速 (全程巡航速度);
  //     > 0  -> 用该值作为减速起始距离。占点模式传入一个比常规更小的值,
  //             既大幅缩短减速段 (响应更快、不易因更新频率低而停不下来),
  //             又保留末段缓冲, 避免全速硬停。
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

  // 网页"开始运动": 校验里程计与全局路径后进入路径跟随
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

  // 计算地图栅格尺寸并校验上限
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

  // 车体自过滤方框 (map 系轴对齐 AABB): 由最新里程计位姿 + self_filter_size
  // 计算, 供"栅格级清除 (clearVehicleFootprint)"与"点级过滤 (点在框内即丢弃)"
  // 两处复用, 保证两者用的是同一套边界定义, 不会出现口径不一致。
  struct FootprintBounds
  {
    bool valid{false};      // false: 无里程计或已关闭自过滤, 不做任何过滤
    double xmin{0.0};
    double xmax{0.0};
    double ymin{0.0};
    double ymax{0.0};
  };

  // 车体中心由最新里程计位姿 (雷达位姿) 按外参反推, 与 computeFollowCommand
  // 步骤 0 使用完全相同的换算式, 保证与路径跟随使用同一个车体中心:
  //     base = lidar_odom - R(yaw) * motion_base_to_lidar
  // (motion_base_to_lidar_{x,y} 语义为"车体中心 -> 雷达原点", 故做减法)
  //   未收到里程计时返回 valid=false (宁可多算障碍也不误删真实障碍)。
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

  // 地图矩形范围裁剪 (开阔场景远距离回波滤波)。
  //   丢弃落在 [xmin,xmax) x [ymin,ymax) 之外的点。口径与 accumulateCloud /
  //   splitDynamicObstacles 的栅格索引一致: 采用半开区间 (含下界、不含上界),
  //   因为栅格索引 ix = floor((x - xmin)/res) 在 x == xmax 处会取到 width,
  //   越界。这样保证"裁剪后的点云"必定能进栅格图, 范围外告警自然消失。
  //   调用时机在点云校正之后、任何下游消费之前, 故单帧图/历史图/绿色动态层/
  //   发往 Nav2 的点云看到的都是同一份框内点云。返回被丢弃的点数。
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

  // 点级自过滤: 丢弃落在车体方框内的自身点云。
  //   与"栅格级清除"不同, 这是在点云进入历史缓冲之前直接剔除, 因此车体
  //   自身点永远不会被烧进历史图, 也就不会在车辆移动后留下"轨迹带"。
  //   (栅格级清除只能挡住当前时刻车体所在的一格方框, 历史位置仍会残留。)
  //   返回被丢弃的点数。
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

  // 车体自身占据格自过滤: 以车体中心为中心、边长 1.0 m 的正方形内,
  // 所有已占据栅格一律清零 (视为车体自身而非环境障碍)。
  //   动机: 雷达装在车上, 近距离必然扫到车体/支架/线缆; 这些点在 PGM 上
  //   表现为车体周围一圈黑色障碍。若不扣除, 会污染静态历史图 (永久烧进
  //   地图)、也会被误判成动态障碍 (车体自己变绿)。
  //   注意: 这里是"栅格级阻挡", 只作用于渲染中间产物; 治本的车体点剔除
  //   见 filterVehicleFootprintPoints (在点云入历史缓冲前执行)。
  //   未收到里程计时不做任何扣除 (宁可多算障碍也不误删真实障碍)。
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

  // 单帧点云投影到占据栅格 (z 范围过滤)
  // hit_counts 非空时进入历史累积模式: 借助 occupancy 作为"本帧已命中"标记,
  // 同一帧内同一栅格只计一次命中, 从而抑制单点噪声
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

  // 对占据栅格做圆形膨胀
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

  // 占据栅格 -> mono8 像素 (PGM 第 0 行在顶部, y 轴上下翻转)
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

  // 持久化历史地图: 最近 history_duration 秒点云按帧投票累积渲染,
  // 每个渲染周期重绘, 因此任何参数调整后都会自动更新
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

  // ==================== 历史累计图导出 / 导入 ====================
  // 导出产物 (目录 = export_map_dir_):
  //   map.pgm        成品历史累计图 (标准 P5 二进制, mono8: 0=占据, 205=未知)
  //   map.yaml       ROS map_server 风格元数据 (分辨率/原点/阈值)
  //   map_clouds.bin 原始历史点云队列 (自有二进制: 帧数/时间戳/点数/xyz float32)
  // 网页端把上述文件打包成 zip 供下载; 导入时反向操作。

  // 把"时间戳 + 点云队列"写成自有二进制 (小端):
  //   magic "RPGMCLD1" (8B) | uint32 frame_count | 每帧 { double stamp_sec; uint32
  //   point_count; point_count * 3 * float32 (x,y,z) }
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

  // 从自有二进制恢复历史点云队列 (与 writeHistoryCloudsBin 对应)
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

  // 写入 ROS map_server 风格 YAML 元数据:
  //   image / resolution / origin[x,y,yaw] / negate / occupied_thresh /
  //   free_thresh。图像语义与 map_server 一致: 0(黑)=占据, 255(白)=空闲,
  //   本节点未知用 205(灰), 故 occupied_thresh 取 0.2 (像素值<51 视为占据),
  //   free_thresh 取 0.8 — 但为避免把 205 误判为占据, 见下方说明。
  //   注: map_server 判定 occupied 是 "1 - pixel/255 < occupied_thresh";
  //   205/255=0.804 -> 1-0.804=0.196 < 0.2 会被判为占据, 故这里把
  //   occupied_thresh 设为 0.1 (仅像素值 < 25.5 视为占据), 205 归入未知(空闲侧)。
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

  // 把导入的成品图占据格叠加到当前历史图像素上 (尺寸一致时)。
  //   导入图像素值 0 = 占据; 仅当目标像素当前为未知 (205) 时才覆盖, 避免盖掉
  //   实时点云刚更新出的更"新鲜"的占据信息。
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

  // 把内存中的成品历史图 + 原始点云队列导出到 export_map_dir_
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

  // 从 import_map_dir_ 恢复历史图 (点云队列 -> history_; 成品图 -> imported_map_)
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

  // 动态障碍栅格图渲染 (仅供网页绿色层显示)。
  //   图像语义 (mono8): 0 = 动态障碍格, 205 = 非动态障碍 (网页据此画绿色)。
  //   注意: 用当前单帧累积 (无投票), 因此障碍消失后下一帧即从图中移除,
  //   这正是"可变障碍"需要的时效性。
  //
  //   【与点云发布解耦】本函数只负责画图, 运行在 render_period_ms 定时器上;
  //   动态障碍点云 (供 Nav2 避障) 改由 publishDynamicObstacleCloud 在点云回调
  //   里即时发布, 不再等渲染周期 —— 这是"红区实时跟随绿色障碍"的关键
  //   (原先点云跟着 200 ms 渲染节奏走, 叠加 renderPgm/renderHistoryPgm 串行
  //   耗时, 端到端延迟常达 200~400 ms)。
  //
  //   【车体扣除】动态栅格现按车体方框 (self_filter_size) 扣除一次: 车体自身点
  //   在进入本流程之前已被 filterVehicleFootprintPoints 点级剔除 (见 cloudCallback),
  //   这里再做栅格级扣除, 抹掉车体正下方一小块的地面/自身残留绿斑。
  //   注意: 该扣除只清车体当前位置方框内, 治不了"方框外随车整体漂移"的地面/
  //   墙面残留 (那属于基准覆盖不到的未观测区域被误判为动态, 需"未观测保护")。
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

  // 避障点云发布 (供 Nav2 obstacle_layer 的 dynamic_obstacles 源)。
  //   在 cloudCallback 里紧跟 splitDynamicObstacles 之后调用, 即"绿色障碍一出现
  //   /一消失就立刻告诉 Nav2", 不经过渲染定时器, 端到端延迟降到 1 个点云帧。
  //
  //   【与绿色层同源】输入就是绿色栅格层所用的那一份点云
  //   (green_layer_points = 自过滤整帧 − 基准占据), 而非"剥出的动态半边"。
  //   于是 Nav2 的红色代价区域与网页绿色层看到的障碍完全一致 —— 车体扣除、
  //   单帧判定都不再出现"绿了但没变红 / 红了但没变绿"的错位。
  //
  //   【高度带统一】z 过滤不再用独立的 marking_zmin/marking_zmax, 统一复用
  //   config_.zmin/zmax (即建图/绿色层同一高度带)。"避障带单独收窄"这个能力
  //   已移除, 要调就一起调。
  //
  //   【不会撞上锁定的累计历史图障碍】锁定后静态基准图 (锁定那一刻的历史累计图)
  //   里的障碍在这一帧点云中是有实际回波的 (它就是当时扫到的静态物), 因此天然
  //   落在整帧点云里, 会被正常发给 Nav2 marking。另外把栅格化的基准障碍
  //   (static_obstacle_mask_) 也补进点云 (每格取中心点), 双重保证这些"历史
  //   冻结障碍"无论此刻有没有回波都持续参与避障, 不会被当成空旷区撞上去。
  //
  //   与自过滤点云一致: 真实变换到 base_link 系后发布, 让 Nav2 的 clearing 射线
  //   从车体当前位置射出 (发 map 帧射线起点会退化为地图原点, 清除覆盖不全)。
  //   无里程计时退回 map 帧, marking 仍正确。
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

  // "清除射线"点云: 把一圈稀疏的射线终点发到 nav2_params.yaml 里一条
  //   【只开 clearing、不开 marking】的独立源 (clear_ring 源) 上。
  //   作用: obstacle_layer 从 base_link 原点向每个点连线, 射线穿过的格子置为
  //   空闲 —— 于是车周一圈范围内的旧红格 (绿色障碍已消失的位置) 被实时清掉,
  //   实现"红色代价区域跟随绿色障碍消失"。
  //   关键: 正因为那条源 marking:false, 这些环点自己不会变成障碍, 不会留下红环;
  //   它们纯粹充当"射线靶子"。对比把环点发到 marking:true 的 dynamic_obstacles
  //   源上会立刻在车周烧出一圈真红环, 那是错的。
  //   参数:
  //     - clear_ring_points: 环上点数 (每 360/N 度一个), <=0 关闭该机制;
  //     - clear_ring_radius: 环半径 (m), 决定清除覆盖到车周多远。
  //   环点定义在 base_link 系 (车体中心为原点), 与该源其余点云坐标系一致。
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

private:
  std::string cloud_topic_;
  int render_period_ms_{200};

  std::mutex cloud_mutex_;
  std::vector<Point> latest_cloud_;
  bool received_cloud_{false};
  std::size_t cloud_frames_{0};

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

  std::mutex config_mutex_;
  Config config_{};
  bool config_changed_{false};

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

  std::deque<TimedCloud> history_;
  // 网页"刷新累计图"请求标志, 由参数回调置位, 渲染线程消费
  std::atomic<bool> history_reset_requested_{false};

  std::string history_image_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    history_image_publisher_;

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

  // ---------- 清除射线点云 (让红色代价区域实时跟随绿色障碍消失) ----------
  // 每帧向 nav2_params.yaml 的 clear_ring 源 (marking:false / clearing:true)
  // 发一圈 base_link 系的稀疏点, 让 obstacle_layer 从车体原点向四周射线清除
  // 旧红格。见 publishClearRingCloud。
  std::string clear_ring_topic_;
  int clear_ring_points_{72};
  double clear_ring_radius_{1.2};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    clear_ring_publisher_;

  // ---------- 刷新历史图时整张清空 Nav2 红色代价区 ----------
  // 见 clearCostmaps: 异步调 ClearEntireCostmap 清空局部/全局代价地图。
  bool clear_costmap_on_history_reset_{true};
  std::string clear_local_costmap_service_;
  std::string clear_global_costmap_service_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
    clear_local_costmap_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
    clear_global_costmap_client_;

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

  // 网页"开始运动"/"停止运动"请求标志, 由参数回调置位, 运动线程消费
  std::atomic<bool> motion_start_requested_{false};
  std::atomic<bool> motion_stop_requested_{false};

  // 路径跟随可视化快照 (前视点/车体中心/速度矢量/车头航向), 供网页读取
  std::mutex lookahead_mutex_;
  LookaheadView lookahead_view_{};

  // ---------- SocketCAN 直发 (把运动帧原样写进 CAN 总线) ----------
  // 开启后无需任何桥接节点: 运动线程产出的 8 字节帧同时发布话题并 write() 到 can0
  std::atomic<bool> can_enabled_{false};
  std::string can_interface_{"can0"};
  canid_t can_id_{0xFF};
  int can_send_period_ms_{10};       // 直发限频周期 (10 ms => 100 Hz 上限)
  int can_socket_fd_{-1};
  std::atomic<uint64_t> can_sent_frames_{0};
  std::chrono::steady_clock::time_point can_last_send_{};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RealtimePgmNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
