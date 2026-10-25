#include <algorithm>
#include <array>
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
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>


class RelocalizationNode : public rclcpp::Node
{
public:

  struct Point
  {
    float x;
    float y;
    float z;
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

  RelocalizationNode()
  : Node("test_relocalization")
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
    //   (不入历史缓冲), 覆盖跳变后的位姿整定过程。<=0 关闭该保护。
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
    // 导出: 把历史图写成 map.pgm + map.yaml, 把原始历史点云队列写成
    //   map_clouds.bin (magic "RPGMCLD1") —— 后续重定位可将其作为先验地图。
    // 导入: 读取上述三个文件重建历史点云队列与导入底图。
    export_map_dir_ =
      declare_parameter<std::string>("export_map_dir", "/tmp/pgm_export");
    declare_parameter<bool>("export_map_request", false);
    import_map_dir_ =
      declare_parameter<std::string>("import_map_dir", "/tmp/pgm_import");
    declare_parameter<bool>("import_map_request", false);
    // 导出 zip 文件名, 缺失时默认 map_export.zip
    export_map_name_ =
      declare_parameter<std::string>("export_map_name", "map_export.zip");
    // 导出 PCD 的体素降采样边长 (m): 0 或负数表示不降采样
    export_pcd_voxel_ =
      declare_parameter<double>("export_pcd_voxel", 0.05);

    cloud_preview_topic_ =
      declare_parameter<std::string>(
        "cloud_preview_topic", "/realtime_pgm/cloud");

    cloud_preview_max_points_ =
      declare_parameter<int>("cloud_preview_max_points", 15000);

    history_image_topic_ =
      declare_parameter<std::string>(
        "history_image_topic", "/realtime_pgm/image_history");

    // ---- 锁定历史累计图 (冻结静态基准) ----
    // 语义继承自主节点"锁定历史累计图":
    //   1) 锁定后历史累计图停止吸收新点云 (冻结为"静态环境"基准);
    //   2) 以冻结历史图为基准做二维栅格配准, 输出 map->odom 修正量并发布
    //      /pose_correction (estimatePoseCorrection) —— 本包重定位功能的
    //      基线实现, 后续全局重定位/绑架恢复在此之上继续开发。
    dynamic_obstacle_enabled_ =
      declare_parameter<bool>("dynamic_obstacle_enabled", false);

    // ---- 锁定历史图后的位姿校正 (map->odom) ----
    // 锁定后以冻结历史图为基准做二维配准, 输出 map->odom 修正量并发布
    //   /pose_correction。
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
    // 是否启用本节点自带的 2D 栅格配准修正 (正确性逊于 3D GICP):
    //   true  = 旧行为: 锁定后自己算修正量并作用到整帧点云
    //   false = 修正交由 3D 重定位节点 (relocalization_3d) 承担, 本节点只出
    //           原始点云, 显示层的修正量由网页按 3D 的 /pose_correction 施加。
    //   省 CPU, 且避免本节点错误的 2D 解把点云搬到错误位置 (与箭头不一致)。
    apply_pose_correction_ =
      declare_parameter<bool>("apply_pose_correction", true);

    parameter_callback_handle_ =
      add_on_set_parameters_callback(
      std::bind(
        &RelocalizationNode::parameterCallback,
        this,
        std::placeholders::_1));

    // 渲染结果同步发布为 mono8 图像, 供查看窗口显示
    image_topic_ =
      declare_parameter<std::string>("image_topic", "/realtime_pgm/image");

    image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(image_topic_, 1);

    // 雷达位姿里程计 (FAST-LIO): 位姿缓存/初值传播/跳变检测
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/Odometry");

    // "车体中心 -> 雷达原点" 的平移 (车体系): 用于由雷达位姿反推车体中心
    //   (computeVehicleFootprint 的自过滤方框)。
    base_to_lidar_x_ =
      declare_parameter<double>("base_to_lidar_x", -0.239);
    base_to_lidar_y_ =
      declare_parameter<double>("base_to_lidar_y", 0.158);

    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &RelocalizationNode::odomCallback,
        this,
        std::placeholders::_1));

    cloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &RelocalizationNode::cloudCallback,
        this,
        std::placeholders::_1));

    // 位姿修正量发布 (map->odom SE(3))
    pose_correction_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_correction_topic_, rclcpp::QoS(10));

    // 降采样点云预览, 供网页查看器绘制可交互点云侧视图
    cloud_preview_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_preview_topic_, rclcpp::SensorDataQoS());

    // 持久化历史地图图像 (最近 history_duration 秒点云按帧投票累积)
    history_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(history_image_topic_, 1);

    render_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(render_period_ms_),
      std::bind(
        &RelocalizationNode::renderTimerCallback,
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
      "重定位节点启动: topic=%s, odom=%s, output=%s, render_period=%d ms, "
      "cloud_preview=%s (max %d pts), history_image=%s (keep %.0f s)",
      cloud_topic_.c_str(),
      odom_topic_.c_str(),
      initial_config.output.c_str(),
      render_period_ms_,
      cloud_preview_topic_.c_str(),
      cloud_preview_max_points_,
      history_image_topic_.c_str(),
      initial_config.history_duration);

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

    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_x_ = corrected_x;
    odom_y_ = corrected_y;
    odom_yaw_ = corrected_yaw;
    odom_received_ = true;
    odom_stamp_ = msg->header.stamp;
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

    // 本节点是修正量的发布者, 广播恒等变换让桥接/网页归零。
    publishPoseCorrection(c);
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

    // 3D 重定位接管时本节点不做任何修正: 既省掉整帧栅格配准的 CPU,
    //   也避免本节点错误的 2D 解把点云搬偏 (网页按 3D 修正量统一施加)。
    if (!apply_pose_correction_) {
      return;
    }

    // 先跑本节点自带的 2D 栅格配准算出修正量, 再把它作用到整帧点云。
    estimatePoseCorrection(cloud);

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
    //   history_duration (默认 300 s)。
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
    //     - 单帧图/历史图 (accumulateCloud) 与后续重定位配准看到的都是同一份
    //       "框内点云", 不会出现"图上有点、配准看不到"的口径不一致。
    const std::size_t clipped = clipCloudToMapBounds(new_cloud);

    if (clipped > 0) {
      RCLCPP_DEBUG(
        get_logger(),
        "地图范围裁剪: 丢弃 %zu 个 xmin..xmax/ymin..ymax 之外的点 (剩余 %zu 点)",
        clipped,
        new_cloud.size());
    }

    // ---- 发布降采样点云预览 (供网页查看器绘制绿色点云) ----
    //   必须放在车体点级自过滤之前: 预览保留车体自身点 (与建图用的"自过滤后
    //   点云"有意区分); 但 z 口径与栅格图一致 —— 函数内部按 zmin/zmax 筛选,
    //   保证网页上看到的点云与底图黑色占据格来自同一批点。
    //   函数内部按 cloud_preview_max_points 抽稀, 且无订阅者时立即返回 (零开销)。
    publishCloudPreview(new_cloud);

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
      "基准占据格 %zu; 此后历史图冻结, 位姿配准以该基准为参照",
      width,
      height,
      config.resolution,
      history_copy.size(),
      baseline_hits);
  }

  // 发布降采样后的点云预览 (供网页端绘制点云)
  //   与栅格图保持同一 z 口径: 只发布落在 [zmin, zmax] 内的点。
  //   否则车体/地面/天花板等 z 范围外的点也会画到网页上 —— 它们不参与建图,
  //   与底图的黑色占据格对不上, 用户会误以为"点云没按 zmin/zmax 筛选"。
  //   筛选口径与 accumulateCloud 完全一致 (z < zmin || z > zmax 即丢弃)。
  void publishCloudPreview(const std::vector<Point> & points)
  {
    if (!cloud_preview_publisher_ ||
      cloud_preview_publisher_->get_subscription_count() == 0)
    {
      return;
    }

    double zmin = 0.0;
    double zmax = 0.0;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      zmin = config_.zmin;
      zmax = config_.zmax;
    }

    // 先用下标筛出落入 z 区间的点 (避免整帧点云拷贝), 再按下标抽稀
    std::vector<std::size_t> kept;
    kept.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i) {
      const double z = points[i].z;

      if (z < zmin || z > zmax) {
        continue;
      }

      kept.push_back(i);
    }

    const std::size_t total = kept.size();

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

    for (std::size_t k = 0; k < total; k += stride) {
      const Point & point = points[kept[k]];
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud_preview_publisher_->publish(preview_msg);
  }

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    Config new_config;
    int preview_max_points = cloud_preview_max_points_;

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
        } else if (name == "export_pcd_voxel") {
          export_pcd_voxel_ = parameter.as_double();
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
        } else if (name == "dynamic_obstacle_enabled") {
          // 网页"锁定历史累计图": 锁定瞬间用当前历史图建立静态基准,
          // 此后历史图冻结, 并以该基准做 map->odom 位姿配准 (重定位基线)。
          const bool enabled = parameter.as_bool();
          if (enabled != dynamic_obstacle_enabled_.load()) {
            dynamic_obstacle_enabled_.store(enabled);
            if (enabled) {
              resetPoseCorrection();
              pose_correction_local_fail_count_.store(0);
              {
                std::lock_guard<std::mutex> lock(odom_mutex_);
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
                std::lock_guard<std::mutex> lock(odom_mutex_);
                baseline_pose_valid_ = false;
              }
              std::lock_guard<std::mutex> lock(dynamic_mutex_);
              static_obstacle_mask_.clear();
              static_obstacle_tolerance_.clear();
              RCLCPP_INFO(
                get_logger(),
                "已解除历史累计图锁定: 恢复正常累积");
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
          // 热切换: 关闭时把修正量归零 (本节点不再输出/作用任何修正)
          const bool enabled = parameter.as_bool();
          if (enabled != apply_pose_correction_) {
            apply_pose_correction_ = enabled;
            if (!enabled) {
              resetPoseCorrection();
            }
            RCLCPP_INFO(
              get_logger(), "本节点 2D 位姿修正已%s (修正量由 3D 节点承担)",
              enabled ? "启用" : "停用");
          }
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
    }

    // 锁定状态下改了影响基准的配置 (分辨率/范围/基准膨胀) 时必须重建基准,
    // 否则基准仍按旧几何索引, 配准会整体错位 -> 重定位失效
    if (dynamic_obstacle_enabled_.load() &&
      (baseline_geometry_changed || new_config.baseline_dilate != prev_baseline_dilate))
    {
      buildStaticBaseline();
    }

    RCLCPP_INFO(
      get_logger(),
      "参数已热更新: resolution=%.4f, zmin=%.3f, zmax=%.3f, "
      "dilate=%d, baseline_dilate=%d, self_filter_size=%.2f, "
      "history_duration=%.1f, history_min_hits=%d",
      new_config.resolution,
      new_config.zmin,
      new_config.zmax,
      new_config.dilate,
      new_config.baseline_dilate,
      new_config.self_filter_size,
      new_config.history_duration,
      new_config.history_min_hits);

    result.successful = true;
    result.reason = "参数更新成功";
    return result;
  }

  void renderTimerCallback()
  {
    const bool reset_history = history_reset_requested_.exchange(false);
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
    std::deque<TimedCloud> history_copy;
    Config config_copy;

    // 刷新历史累计图: 清空本节点历史缓冲, 之后随新历史图从零重建。
    //   放在最前面 (早于 received_cloud_ 检查)。
    if (reset_history) {
      {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        history_.clear();
      }
      RCLCPP_INFO(get_logger(), "历史累计图已清空, 开始重新累积");
    }

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!received_cloud_) {
        return;
      }
      cloud_copy = latest_cloud_;
      pruneHistory(get_clock()->now());
      history_copy = history_;
    }

    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_copy = config_;
      config_changed_ = false;
    }

    renderPgm(cloud_copy, config_copy);
    renderHistoryPgm(history_copy, config_copy);
  }

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

  // 车体中心由最新里程计位姿 (雷达位姿) 按外参反推:
  //     base = lidar_odom - R(yaw) * base_to_lidar
  // (base_to_lidar_{x,y} 语义为"车体中心 -> 雷达原点", 故做减法)
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
      std::lock_guard<std::mutex> lock(odom_mutex_);
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
      (cos_yaw * base_to_lidar_x_ - sin_yaw * base_to_lidar_y_);
    const double base_y = odom_y -
      (sin_yaw * base_to_lidar_x_ + cos_yaw * base_to_lidar_y_);

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
  //   调用时机在点云校正之后、任何下游消费之前, 故单帧图/历史图看到的都是
  //   同一份框内点云。返回被丢弃的点数。
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

  // 把历史点云队列合并 -> 体素降采样 -> 写成 PCD v0.7 (binary, x/y/z float32)。
  //   内容口径与 map_clouds.bin 一致 (全部高度点, 不做 z 裁剪), 仅做体素抽稀
  //   以控制体积; 供离线查看 (CloudCompare/pcl_viewer) 或外部工具直接当作
  //   先验点云地图使用。本节点刻意不依赖 PCL, 故手写 PCD 头与二进制数据段。
  //   voxel_size <= 0 表示不降采样 (原样写出, 文件可能很大)。
  bool writeHistoryCloudsPcd(
    const std::deque<TimedCloud> & history, double voxel_size,
    const std::string & path)
  {
    // 体素哈希: 每格保留首个点。key 由体素索引按 21 位/轴打包
    //   (偏移 +2^20, 覆盖 ±50 km @0.05 m, 远超本工程地图范围)。
    std::unordered_map<std::int64_t, std::array<float, 3>> voxels;
    std::size_t total_points = 0;

    if (voxel_size > 0.0) {
      const double inv = 1.0 / voxel_size;
      auto packKey = [](std::int64_t ix, std::int64_t iy, std::int64_t iz) {
          const std::int64_t offset = 1LL << 20;
          const std::int64_t mask = (1LL << 21) - 1;
          return (((ix + offset) & mask) << 42) |
                 (((iy + offset) & mask) << 21) |
                 ((iz + offset) & mask);
        };

      for (const auto & timed : history) {
        if (!timed.points) {
          continue;
        }
        total_points += timed.points->size();
        for (const auto & p : *timed.points) {
          const auto ix = static_cast<std::int64_t>(std::floor(p.x * inv));
          const auto iy = static_cast<std::int64_t>(std::floor(p.y * inv));
          const auto iz = static_cast<std::int64_t>(std::floor(p.z * inv));
          voxels.emplace(packKey(ix, iy, iz), std::array<float, 3>{p.x, p.y, p.z});
        }
      }
    }

    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
      RCLCPP_ERROR(get_logger(), "无法创建导出 PCD: %s", path.c_str());
      return false;
    }

    if (voxel_size > 0.0) {
      const std::size_t count = voxels.size();
      file << "# .PCD v0.7 - Point Cloud Data file format\n"
        << "VERSION 0.7\n"
        << "FIELDS x y z\n"
        << "SIZE 4 4 4\n"
        << "TYPE F F F\n"
        << "COUNT 1 1 1\n"
        << "WIDTH " << count << "\n"
        << "HEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << count << "\n"
        << "DATA binary\n";

      for (const auto & kv : voxels) {
        file.write(
          reinterpret_cast<const char *>(kv.second.data()),
          static_cast<std::streamsize>(sizeof(float) * 3));
      }
      RCLCPP_INFO(
        get_logger(), "PCD 已导出: %zu 点 (原始 %zu 点, 体素 %.3f m) -> %s",
        count, total_points, voxel_size, path.c_str());
    } else {
      // 不降采样: 需要先把总量数出来才能写头
      std::size_t count = 0;
      for (const auto & timed : history) {
        if (timed.points) {
          count += timed.points->size();
        }
      }
      file << "# .PCD v0.7 - Point Cloud Data file format\n"
        << "VERSION 0.7\n"
        << "FIELDS x y z\n"
        << "SIZE 4 4 4\n"
        << "TYPE F F F\n"
        << "COUNT 1 1 1\n"
        << "WIDTH " << count << "\n"
        << "HEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << count << "\n"
        << "DATA binary\n";

      for (const auto & timed : history) {
        if (!timed.points) {
          continue;
        }
        for (const auto & p : *timed.points) {
          const float xyz[3] = {p.x, p.y, p.z};
          file.write(
            reinterpret_cast<const char *>(xyz),
            static_cast<std::streamsize>(sizeof(xyz)));
        }
      }
      RCLCPP_INFO(
        get_logger(), "PCD 已导出: %zu 点 (未降采样) -> %s",
        count, path.c_str());
    }

    if (!file.good()) {
      RCLCPP_ERROR(get_logger(), "写入导出 PCD 失败: %s", path.c_str());
      return false;
    }
    return true;
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

    // ---- 4) 合并后的 PCD (全部高度点, 体素降采样; 供离线查看/外部工具) ----
    const std::string pcd_path = dir + "/map.pcd";
    if (!writeHistoryCloudsPcd(history_copy, export_pcd_voxel_, pcd_path)) {
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
      // 导入后若已处于锁定态, 用导入的历史图重建静态基准
      //   (重定位配准将切换到以导入的先验地图为参照)。
      if (dynamic_obstacle_enabled_.load()) {
        buildStaticBaseline();
      }
      RCLCPP_INFO(get_logger(), "历史累计图导入完成");
    } else {
      RCLCPP_ERROR(get_logger(), "历史累计图导入失败: 目录中无有效文件 (%s)",
        dir.c_str());
    }
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
      "位姿修正 (锁定历史图后生效): topic=%s, 评分容差=基准膨胀+1 格, "
      "局部搜索 ±%.1f m / ±15° (步长 0.2 m) + 格级细化, "
      "全局兜底 ±%.1f m / ±180° (连续失败 %d 帧后触发, 最少间隔 %.1f s), "
      "分数阈值=%.2f, 单帧上限=%.2f m, EMA alpha=%.2f, 滞回增益=%.2f",
      pose_correction_topic_.c_str(),
      pose_correction_local_range_m_,
      pose_correction_global_range_m_,
      pose_correction_global_trigger_fails_,
      pose_correction_global_period_sec_,
      pose_correction_score_min_,
      pose_correction_max_delta_m_,
      pose_correction_ema_alpha_,
      pose_correction_score_gain_);
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
  // 导出 PCD 的体素降采样边长 (m), <=0 表示不降采样
  double export_pcd_voxel_{0.05};
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

  // ---------- 锁定历史累计图 (静态基准): 重定位配准的参照 ----------
  // 静态基准栅格: 锁定那一刻由当前历史累积图投票生成, 作为"静态环境"依据
  std::mutex dynamic_mutex_;
  std::vector<uint8_t> static_obstacle_mask_;
  // 评分容差掩码: 基准占据掩码再膨胀 1 格 (容差 = dilate + 1 格)。
  //   原实现容差仅 ±1 格而搜索步长 0.2/1.0 m, 采样点几乎全部错过基准,
  //   分数退化为噪声 -> 修正量乱跳。预膨胀后评分仍是单次查表。
  std::vector<uint8_t> static_obstacle_tolerance_;
  std::atomic<bool> dynamic_obstacle_enabled_{false};

  // ---------- 位姿修正 (map->odom, SE(3)) ----------
  std::string pose_correction_topic_{"/pose_correction"};
  double pose_correction_global_period_sec_{1.0};
  double pose_correction_global_range_m_{15.0};
  double pose_correction_local_range_m_{1.0};
  double pose_correction_score_min_{0.12};
  // 单帧校正变化上限 (m): 超过即判"跳飞"沿用上次值。默认由 1.5 收紧到 0.3,
  //   避免一次估计错误就把整帧点云搬走 1.5 m (观感上即"猛飘一下")。
  double pose_correction_max_delta_m_{0.3};
  // 时序滤波: EMA 平滑系数 (1 = 不平滑) 与滞回增益阈值
  //   (新解评分须比当前解高出该值才接受, 否则保持当前修正量不动)
  double pose_correction_ema_alpha_{0.5};
  double pose_correction_score_gain_{0.05};
  // 全局兜底搜索触发条件: 局部搜索连续失败达到该帧数才做 ±15 m/±180° 全搜
  int pose_correction_global_trigger_fails_{5};
  // 本节点 2D 修正总开关 (false = 交由 3D 重定位节点承担)
  bool apply_pose_correction_{true};
  std::atomic<int> pose_correction_local_fail_count_{0};
  std::atomic<double> pose_correction_last_global_sec_{
    -std::numeric_limits<double>::infinity()};
  std::mutex pose_correction_mutex_;
  double baseline_pose_x_{0.0};
  double baseline_pose_y_{0.0};
  double baseline_pose_yaw_{0.0};
  bool baseline_pose_valid_{false};
  // 位姿修正量 (map->odom, SE(3)): 由本节点自带的 2D 栅格配准提供
  PoseCorrection pose_correction_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_correction_publisher_;

  // ---------- 里程计 (FAST-LIO /Odometry): 位姿缓存与初值传播 ----------
  std::string odom_topic_;
  // "车体中心 -> 雷达原点" 的平移 (车体系), 车体自过滤方框用
  double base_to_lidar_x_{-0.239};
  double base_to_lidar_y_{0.158};
  std::mutex odom_mutex_;
  double odom_x_{0.0};               // 最新雷达位姿 (map 系, 已叠加修正量)
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool odom_received_{false};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RelocalizationNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
