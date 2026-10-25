// 3D 重定位节点 (GICP scan-to-map):
//   先验地图: 网页"锁定基准"时导出的 map_clouds.bin (RPGMCLD1 格式,
//     历史点云队列, 点已在 map 系)。本节点按 mtime 轮询, 文件被覆盖即
//     自动重载并重建配准目标 —— 重新锁定 = 重新导出 = 重定位重新开始。
//   配准: 当前帧 /cloud_registered (FAST-LIO odom 系) 对先验全图做 GICP,
//     直接得到 T_map_odom (与 2D 节点的 /pose_correction 语义一致:
//     p_map = R * p_odom + t)。降频运行 (rate_hz, 网页可热调)。
//   输出 (混合式): /pose_correction (PoseStamped, 与 2D 版格式一致,
//     网页/下游零改动) + map->odom TF + /relocalization_3d/fitness
//     (Float64, 置信度参考, 每次尝试都发)。
//   门控: enabled 参数 (网页锁定/解锁时同步设置)。禁用时发布一次恒等
//     修正, 使下游退回原始里程计。
//   先验地图格式: 按扩展名自动识别 —— .pcd 用 pcl::io 直接读取
//     (可用 stl2pcd.py 由官方场地 STL 采样生成, 不依赖实地建图),
//     其它扩展名按 RPGMCLD1 (网页导出的历史点云队列) 解析。
//   初始位姿: init_x / init_y / init_yaw 三个参数提供首次配准的基座
//     (默认原点), 供 AMCL 等粗定位模块在上电时注入; 收敛后沿用它跟踪。
//   已知限制: 未注入初始位姿时仍假设在先验地图原点附近 (多 yaw 假设只能
//     纠正朝向误差), 大位移绑架恢复待后续 ScanContext 全局检索补齐。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

#include <tf2_ros/transform_broadcaster.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

class Relocalization3DNode : public rclcpp::Node
{
public:
  Relocalization3DNode()
  : Node("relocalization_3d")
  {
    // ---- 输入 ----
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/cloud_registered");
    // ---- 先验地图 (mtime 轮询自动重载) ----
    prior_map_path_ = declare_parameter<std::string>(
      "prior_map_path", "/tmp/pgm_export/map_clouds.bin");
    map_poll_period_sec_ = declare_parameter<double>(
      "map_poll_period_sec", 1.0);
    // ---- 门控 / 频率 ----
    enabled_ = declare_parameter<bool>("enabled", false);
    rate_hz_ = declare_parameter<double>("rate_hz", 2.0);
    // ---- 点云预处理 ----
    source_voxel_ = declare_parameter<double>("source_voxel", 0.2);
    target_voxel_ = declare_parameter<double>("target_voxel", 0.2);
    source_max_points_ = declare_parameter<int>("source_max_points", 8000);
    // 范围过滤: 丢弃距离超过该值的点。Gazebo gpu_ray 与真实雷达在"没打到东西"
    //   时会发最大量程的假点 (40 m), 这些点没有对应物, 会把 fitness 拉爆
    source_range_max_ = declare_parameter<double>("source_range_max", 20.0);
    // ---- GICP ----
    gicp_max_corr_dist_ = declare_parameter<double>(
      "gicp_max_corr_dist", 2.0);
    gicp_max_iter_ = declare_parameter<int>("gicp_max_iter", 50);
    // 接受门限 (口径: 只统计 max_corr_dist 内的对应点)
    //   gicp_fitness_max  = 截断均方对应距离 (m^2)
    //   fitness_inlier_min = 内点比例下限 (对应点数 / 源点数)
    gicp_fitness_max_ = declare_parameter<double>("gicp_fitness_max", 2.0);
    fitness_inlier_min_ = declare_parameter<double>("fitness_inlier_min", 0.3);
    // 单次修正跳变上限 (m): 防止误匹配把位姿搬飞。
    //   实际门限 = max(correction_max_delta_m, correction_max_speed_mps * 帧间隔),
    //   否则低配准频率下正常行驶(1 Hz + 1 m/s 就超 1 m)会被大面积误拒
    correction_max_delta_m_ = declare_parameter<double>(
      "correction_max_delta_m", 1.0);
    correction_max_speed_mps_ = declare_parameter<double>(
      "correction_max_speed_mps", 2.0);
    // 里程计补偿: 仅在"点云在传感器系、输出是绝对位姿"时打开
    //   (仿真 /livox/lidar 直连)。点云已注册到 odom 系时保持 false
    compensate_odom_ = declare_parameter<bool>("compensate_odom", false);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    // EMA 平滑系数 (1 = 不平滑)
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.5);
    // ---- 初始位姿猜测 (上电 / 全局定位交接时注入) ----
    // 只在 need_init_ 的首次配准生效: 以 (init_x, init_y, init_yaw) 为基座
    // 叠加 init_yaw_hypotheses 个 yaw 假设做 GICP; 收敛后沿用结果跟踪
    init_x_ = declare_parameter<double>("init_x", 0.0);
    init_y_ = declare_parameter<double>("init_y", 0.0);
    init_yaw_ = declare_parameter<double>("init_yaw", 0.0);
    // 初始 yaw 假设个数: 8 = 每 45° 试一次 (无粗定位时用);
    // 已有可信的 init_yaw (如 AMCL 收敛结果) 时设 1, 初始化耗时可降到 1/8
    init_yaw_hypotheses_ = declare_parameter<int>("init_yaw_hypotheses", 8);
    // ---- 输出 ----
    pose_correction_topic_ = declare_parameter<std::string>(
      "pose_correction_topic", "/pose_correction");
    fitness_topic_ = declare_parameter<std::string>(
      "fitness_topic", "/relocalization_3d/fitness");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>(
      "odom_frame", "camera_init");
    broadcast_tf_ = declare_parameter<bool>("broadcast_tf", true);
    // TF 广播频率: 配准频率(默认 1 Hz)远低于点云(10 Hz), 若只在配准时发 TF,
    //   RViz 会报 "Lookup would require extrapolation into the future" 并闪断
    tf_publish_rate_ = declare_parameter<double>("tf_publish_rate", 10.0);
    // 2D 定位: 把 map->odom 的 z 锁 0 (先验图的 z 原点可能与地面差十几厘米)
    correction_z_locked_ = declare_parameter<bool>("correction_z_locked", true);
    // 先验地图可视化话题 (latched, RViz 里可直接叠加对照)
    prior_map_topic_ = declare_parameter<std::string>(
      "prior_map_topic", "/relocalization_3d/prior_map");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        CloudT::Ptr cloud(new CloudT);
        pcl::fromROSMsg(*msg, *cloud);
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = cloud;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto & p = msg->pose.pose.position;
        const auto & q = msg->pose.pose.orientation;
        const double yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x_ = p.x;
        odom_y_ = p.y;
        odom_yaw_ = yaw;
        has_odom_ = true;
      });

    correction_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_correction_topic_, 10);
    fitness_pub_ = create_publisher<std_msgs::msg::Float64>(
      fitness_topic_, 10);
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 先验地图可视化: latched (transient_local), 后加入的 RViz 也能收到
    prior_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      prior_map_topic_, rclcpp::QoS(1).transient_local());
    recreateTfTimer();

    map_poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(map_poll_period_sec_),
      [this]() { pollPriorMap(); });
    recreateAlignTimer();

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "enabled") {
            setEnabled(p.as_bool());
          } else if (p.get_name() == "rate_hz") {
            rate_hz_ = std::max(0.1, p.as_double());
            recreate_align_requested_ = true;
          } else if (p.get_name() == "prior_map_path") {
            prior_map_path_ = p.as_string();
            known_mtime_.reset();
          } else if (p.get_name() == "gicp_fitness_max") {
            gicp_fitness_max_ = p.as_double();
          } else if (p.get_name() == "fitness_inlier_min") {
            fitness_inlier_min_ = p.as_double();
          } else if (p.get_name() == "source_range_max") {
            source_range_max_ = std::max(1.0, p.as_double());
          } else if (p.get_name() == "correction_max_delta_m") {
            correction_max_delta_m_ = p.as_double();
          } else if (p.get_name() == "correction_max_speed_mps") {
            correction_max_speed_mps_ = p.as_double();
          } else if (p.get_name() == "compensate_odom") {
            compensate_odom_ = p.as_bool();
          } else if (p.get_name() == "tf_publish_rate") {
            tf_publish_rate_ = p.as_double();
            recreateTfTimer();
          } else if (p.get_name() == "correction_z_locked") {
            correction_z_locked_ = p.as_bool();
          } else if (p.get_name() == "ema_alpha") {
            ema_alpha_ = p.as_double();
          }
        }
        return result;
      });

    RCLCPP_INFO(
      get_logger(),
      "3D 重定位节点启动: 先验地图 %s (mtime 轮询 %.1fs), "
      "配准频率 %.2f Hz, 等待 enabled=true (网页锁定基准)",
      prior_map_path_.c_str(), map_poll_period_sec_, rate_hz_);
  }

private:
  // ---------------- 先验地图: mtime 轮询 + RPGMCLD1 加载 ----------------

  void pollPriorMap()
  {
    std::error_code ec;
    if (!std::filesystem::exists(prior_map_path_, ec)) {
      return;
    }
    const auto mtime =
      std::filesystem::last_write_time(prior_map_path_, ec);
    if (ec) {
      return;
    }
    if (known_mtime_ && *known_mtime_ == mtime) {
      return;   // 文件未变
    }
    if (loadPriorMap(prior_map_path_)) {
      known_mtime_ = mtime;
    }
  }

  // RPGMCLD1: magic(8B) | uint32 帧数 | 每帧 { double 时间戳;
  //   uint32 点数; 点数 * 3 * float32 (x,y,z, 已是 map 系) }
  bool loadPriorMap(const std::string & path)
  {
    // 按扩展名分派:
    //   .pcd / .ply 等 → 通用点云（stl2pcd.py 合成的先验图，点已在 map 系）
    //   其它           → RPGMCLD1（网页导出的历史点云队列）
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    CloudT::Ptr merged(new CloudT);
    if (ext == ".pcd") {
      if (pcl::io::loadPCDFile<PointT>(path, *merged) != 0 || merged->empty()) {
        RCLCPP_WARN(get_logger(), "PCD 先验地图加载失败: %s", path.c_str());
        return false;
      }
      RCLCPP_INFO(
        get_logger(), "先验地图(PCD) 已读取: %zu 点 (%s)",
        merged->size(), path.c_str());
    } else {
      std::ifstream file(path, std::ios::binary);
      if (!file) {
        RCLCPP_WARN(get_logger(), "先验地图打不开: %s", path.c_str());
        return false;
      }
      char magic[8];
      file.read(magic, 8);
      if (!file.good() || std::memcmp(magic, "RPGMCLD1", 8) != 0) {
        RCLCPP_WARN(get_logger(), "先验地图格式不匹配: %s", path.c_str());
        return false;
      }
      uint32_t frame_count = 0;
      file.read(reinterpret_cast<char *>(&frame_count), 4);
      if (!file.good() || frame_count == 0 || frame_count > 100000) {
        RCLCPP_WARN(get_logger(), "先验地图帧数异常: %u", frame_count);
        return false;
      }

      merged->reserve(frame_count * 2000);
    for (uint32_t f = 0; f < frame_count; ++f) {
      double stamp = 0.0;
      uint32_t n = 0;
      file.read(reinterpret_cast<char *>(&stamp), 8);
      file.read(reinterpret_cast<char *>(&n), 4);
      if (!file.good() || n > 1000000) {
        RCLCPP_WARN(get_logger(), "先验地图第 %u 帧损坏, 中止加载", f);
        return false;
      }
      std::vector<float> xyz(n * 3);
      file.read(reinterpret_cast<char *>(xyz.data()), n * 3 * 4);
      if (!file.good()) {
        RCLCPP_WARN(get_logger(), "先验地图第 %u 帧数据不完整", f);
        return false;
      }
      for (uint32_t i = 0; i < n; ++i) {
        merged->emplace_back(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
      }
    }
    }   // end else: RPGMCLD1

    CloudT::Ptr target(new CloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(
      static_cast<float>(target_voxel_), static_cast<float>(target_voxel_),
      static_cast<float>(target_voxel_));
    voxel.setInputCloud(merged);
    voxel.filter(*target);
    if (target->size() < 500) {
      RCLCPP_WARN(
        get_logger(), "先验地图点数过少 (%zu), 暂不使用", target->size());
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      target_cloud_ = target;
      // 目标点 KD 树: 供质量评估做最近邻查询 (与地图同生命周期)
      target_tree_.reset(new pcl::KdTreeFLANN<PointT>);
      target_tree_->setInputCloud(target);
      map_ready_ = true;
      need_init_ = true;   // 新地图 -> 下次配准走多 yaw 初始化
    }
    publishPriorMap();   // latched 发布一份, RViz 可直接叠加对照
    RCLCPP_INFO(
      get_logger(), "先验地图已加载: %zu 点(原始) -> %zu 点(体素 %.2f m)",
      merged->size(), target->size(), target_voxel_);
    return true;
  }

  // ---------------- 门控 ----------------

  void setEnabled(bool value)
  {
    if (value == enabled_) {
      return;
    }
    enabled_ = value;
    if (!enabled_) {
      // 禁用: 发布一次恒等修正, 让下游退回原始里程计
      corr_x_ = corr_y_ = corr_z_ = corr_yaw_ = 0.0;
      has_correction_ = false;
      publishCorrection(get_clock()->now());
      RCLCPP_INFO(get_logger(), "3D 重定位已停用 (解锁), 输出恒等修正");
    } else {
      RCLCPP_INFO(get_logger(), "3D 重定位已启用 (锁定)");
    }
  }

  // ---------------- 配准 ----------------

  void recreateAlignTimer()
  {
    align_timer_.reset();
    align_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz_)),
      [this]() { alignOnce(); });
  }

  void alignOnce()
  {
    if (recreate_align_requested_) {
      recreate_align_requested_ = false;
      recreateAlignTimer();
      return;
    }
    if (!enabled_) {
      return;
    }

    // 帧间隔: 跳变门限按速率自适应, 用 steady_clock 避免 /clock 跳变影响
    const auto now_tp = std::chrono::steady_clock::now();
    if (last_align_tp_.time_since_epoch().count() != 0) {
      dt_ = std::chrono::duration<double>(now_tp - last_align_tp_).count();
    }
    last_align_tp_ = now_tp;

    CloudT::Ptr source;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      source = latest_cloud_;
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!map_ready_) {
        return;
      }
    }
    if (!source || source->empty()) {
      return;
    }

    // 预处理 1: 范围过滤 —— 丢弃最大量程/无回波假点 (Gazebo gpu_ray 在 40 m
    //   处补点, 真实雷达空旷处同样有), 否则它们把 fitness 拉爆
    CloudT::Ptr ranged(new CloudT);
    ranged->reserve(source->size());
    const double rmax2 = source_range_max_ * source_range_max_;
    for (const auto & p : source->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      if (static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
        static_cast<double>(p.z) * p.z > rmax2)
      {
        continue;
      }
      ranged->push_back(p);
    }
    if (ranged->size() < 100) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "范围过滤(<=%.1fm)后有效点仅 %zu, 跳过本次", source_range_max_,
        ranged->size());
      return;
    }

    // 预处理 2: 体素降采样 + 点数上限 (stride)
    CloudT::Ptr filtered(new CloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(
      static_cast<float>(source_voxel_), static_cast<float>(source_voxel_),
      static_cast<float>(source_voxel_));
    voxel.setInputCloud(ranged);
    voxel.filter(*filtered);
    const size_t cap = static_cast<size_t>(std::max(100, source_max_points_));
    if (filtered->size() > cap) {
      CloudT::Ptr capped(new CloudT);
      capped->reserve(cap);
      const size_t stride = (filtered->size() + cap - 1) / cap;
      for (size_t i = 0; i < filtered->size(); i += stride) {
        capped->push_back((*filtered)[i]);
      }
      filtered = capped;
    }

    // 初始猜测: 沿用上次接受的修正; 新地图初始化时叠加 init_yaw_hypotheses
    //   个 yaw 假设 (只能纠正朝向误差, 大位移绑架恢复待 ScanContext 补齐)
    bool init_mode;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      init_mode = need_init_;
    }
    std::vector<double> yaw_guesses;
    if (init_mode) {
      const int n_hyp = std::max(1, init_yaw_hypotheses_);
      for (int k = 0; k < n_hyp; ++k) {
        // n_hyp=8 时即每 45° 一个假设 (k * M_PI_4)
        yaw_guesses.push_back(2.0 * M_PI * k / n_hyp);
      }
    } else {
      yaw_guesses.push_back(0.0);
    }
    // 初始化时以 init_* 为基座（外部注入的粗定位/先验位姿），跟踪时沿用上次修正
    const double base_x = init_mode ? init_x_ : corr_x_;
    const double base_y = init_mode ? init_y_ : corr_y_;
    const double base_z = init_mode ? 0.0 : corr_z_;
    const double base_yaw = init_mode ? init_yaw_ : corr_yaw_;

    Eigen::Matrix4f best_tf = Eigen::Matrix4f::Identity();
    double best_fitness = std::numeric_limits<double>::max();
    double best_inlier = 0.0;
    bool converged = false;
    for (const double dyaw : yaw_guesses) {
      Eigen::Matrix4f guess = makeTf(base_x, base_y, base_z, base_yaw + dyaw);
      CloudT aligned;
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setMaximumIterations(gicp_max_iter_);
      gicp.setMaxCorrespondenceDistance(gicp_max_corr_dist_);
      gicp.setTransformationEpsilon(1e-4);
      gicp.setInputSource(filtered);
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        gicp.setInputTarget(target_cloud_);
      }
      gicp.align(aligned, guess);
      if (!gicp.hasConverged()) {
        continue;
      }
      // 质量评估 (自算, 不用 getFitnessScore):
      //   只统计 max_corr_dist 内的对应点 -> 截断均方 + 内点比例,
      //   避免远端/无对应点把均值拉爆 (PCL 的 getFitnessScore 会对全部源点求均值)
      double mse = 0.0;
      double inlier = 0.0;
      evaluateFitness(aligned, mse, inlier);
      // 先比内点比例 (更抗初值偏差), 内点接近时再比截断均方
      const bool better = (inlier > best_inlier + 1e-3) ||
        (std::abs(inlier - best_inlier) <= 1e-3 && mse < best_fitness);
      if (better) {
        best_inlier = inlier;
        best_fitness = mse;
        best_tf = gicp.getFinalTransformation();
        converged = true;
      }
    }

    // 置信度话题: 每次尝试都发 (未收敛发 -1), 网页可据此时实观察
    std_msgs::msg::Float64 fitness_msg;
    fitness_msg.data = converged ? best_fitness : -1.0;
    fitness_pub_->publish(fitness_msg);

    if (!converged) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "GICP 配准被拒绝: 未收敛");
      return;
    }
    if (best_inlier < fitness_inlier_min_ || best_fitness > gicp_fitness_max_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "GICP 配准被拒绝: 内点 %.2f (下限 %.2f) 截断均方 %.3f m^2 (门限 %.3f)",
        best_inlier, fitness_inlier_min_, best_fitness, gicp_fitness_max_);
      return;
    }

    double nx, ny, nz, nyaw;
    decomposeTf(best_tf, nx, ny, nz, nyaw);
    if (correction_z_locked_) {
      nz = 0.0;   // 2D 定位: 高度不参与修正 (先验图 z 原点常与地面有偏差)
    }

    // 跳变保护: 与上次修正差超门限则拒绝 (初始化阶段豁免)
    //   门限 = max(绝对上限, 速率上限 * 实际帧间隔): 低配准频率或快车时自适应放宽
    if (!init_mode && has_correction_) {
      double pred_x = corr_x_;
      double pred_y = corr_y_;
      {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (compensate_odom_ && has_odom_ && has_odom_at_corr_) {
          // 传感器系点云时输出是绝对位姿, 车辆自位移会带进解里, 先用 odom
          // 增量抵扣, 否则每帧都在"跳变"
          const double c = std::cos(corr_yaw_);
          const double s = std::sin(corr_yaw_);
          const double dx_odom = odom_x_ - odom_at_corr_x_;
          const double dy_odom = odom_y_ - odom_at_corr_y_;
          pred_x += c * dx_odom - s * dy_odom;
          pred_y += s * dx_odom + c * dy_odom;
        }
      }
      const double limit = std::max(
        correction_max_delta_m_,
        correction_max_speed_mps_ * std::max(dt_, 0.1));
      const double jump = std::hypot(nx - pred_x, ny - pred_y);
      if (jump > limit) {
        RCLCPP_WARN(
          get_logger(),
          "修正跳变 %.2f m 超门限 %.2f m (dt=%.2fs), 拒绝本次 "
          "(%.2f,%.2f)->(%.2f,%.2f)",
          jump, limit, dt_, pred_x, pred_y, nx, ny);
        return;
      }
    }

    // EMA 平滑 (初始化直接采用, 不做平滑)
    if (init_mode || !has_correction_) {
      corr_x_ = nx; corr_y_ = ny; corr_z_ = nz; corr_yaw_ = nyaw;
    } else {
      const double a = std::min(1.0, std::max(0.0, ema_alpha_));
      corr_x_ += a * (nx - corr_x_);
      corr_y_ += a * (ny - corr_y_);
      corr_z_ += a * (nz - corr_z_);
      corr_yaw_ += a * wrapAngle(nyaw - corr_yaw_);
    }
    has_correction_ = true;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (has_odom_) {
        odom_at_corr_x_ = odom_x_;
        odom_at_corr_y_ = odom_y_;
        has_odom_at_corr_ = true;
      }
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      need_init_ = false;
    }

    publishCorrection(get_clock()->now());
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "3D 配准成功: fitness=%.3f m^2 (内点 %.2f), map->odom=(%.2f, %.2f, %.1f deg)%s",
      best_fitness, best_inlier, corr_x_, corr_y_, corr_yaw_ * 180.0 / M_PI,
      init_mode ? " [初始化]" : "");
  }

  // ---------------- 质量评估 ----------------

  // 对 aligned 中每个点在目标图上找最近邻, 只统计 max_corr_dist 内的对应点:
  //   mean_sq = 截断均方对应距离 (m^2), inlier = 内点比例 (对应点数/源点数)
  // 与 PCL getFitnessScore() 的区别: 后者对全部源点求均值, 无对应物的远端点
  //   会把均值拉爆 (Gazebo 40 m 无回波假点 -> fitness 183, 正确结果被误杀)
  void evaluateFitness(const CloudT & aligned, double & mean_sq, double & inlier)
  {
    mean_sq = std::numeric_limits<double>::max();
    inlier = 0.0;
    if (aligned.empty()) {
      return;
    }
    const double max_d2 = gicp_max_corr_dist_ * gicp_max_corr_dist_;
    double sum = 0.0;
    size_t matched = 0;
    pcl::Indices idx(1);
    std::vector<float> d2(1);
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!target_tree_ || !target_cloud_) {
      return;
    }
    for (const auto & p : aligned.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      if (target_tree_->nearestKSearch(p, 1, idx, d2) <= 0) {
        continue;
      }
      if (d2[0] > max_d2) {
        continue;
      }
      sum += static_cast<double>(d2[0]);
      ++matched;
    }
    inlier = static_cast<double>(matched) / static_cast<double>(aligned.size());
    if (matched > 0) {
      mean_sq = sum / static_cast<double>(matched);
    }
  }

  // ---------------- 输出 ----------------

  void publishCorrection(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = corr_x_;
    pose.pose.position.y = corr_y_;
    pose.pose.position.z = corr_z_;
    pose.pose.orientation.z = std::sin(corr_yaw_ * 0.5);
    pose.pose.orientation.w = std::cos(corr_yaw_ * 0.5);
    correction_pub_->publish(pose);
  }

  // TF 独立广播: 由 tf_timer_ 按 tf_publish_rate 驱动, 与配准频率解耦。
  //   未解锁/未初始化时也照发 (此时是恒等修正, 语义上就是 map==odom),
  //   否则 RViz 在两条 TF 之间查不到变换会闪断。
  void publishTf()
  {
    if (!broadcast_tf_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = corr_x_;
    tf.transform.translation.y = corr_y_;
    tf.transform.translation.z = corr_z_;
    tf.transform.rotation.z = std::sin(corr_yaw_ * 0.5);
    tf.transform.rotation.w = std::cos(corr_yaw_ * 0.5);
    tf_broadcaster_->sendTransform(tf);
  }

  void recreateTfTimer()
  {
    tf_timer_.reset();
    if (!broadcast_tf_ || tf_publish_rate_ <= 0.0) {
      return;   // 关闭 TF 广播 (例如下游自己合成 map->odom)
    }
    tf_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tf_publish_rate_),
      [this]() { publishTf(); });
  }

  // 先验地图 -> PointCloud2 (latched), 供 RViz 叠加对照
  void publishPriorMap()
  {
    if (!prior_map_pub_) {
      return;
    }
    CloudT::Ptr target;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      target = target_cloud_;
    }
    if (!target || target->empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*target, msg);
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = map_frame_;
    prior_map_pub_->publish(msg);
  }

  // ---------------- 小工具 ----------------

  static Eigen::Matrix4f makeTf(double x, double y, double z, double yaw)
  {
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    const double c = std::cos(yaw), s = std::sin(yaw);
    tf(0, 0) = c; tf(0, 1) = -s; tf(0, 3) = x;
    tf(1, 0) = s; tf(1, 1) = c;  tf(1, 3) = y;
    tf(2, 3) = z;
    return tf;
  }

  static void decomposeTf(
    const Eigen::Matrix4f & tf, double & x, double & y, double & z,
    double & yaw)
  {
    x = tf(0, 3);
    y = tf(1, 3);
    z = tf(2, 3);
    yaw = std::atan2(tf(1, 0), tf(0, 0));
  }

  static double wrapAngle(double a)
  {
    while (a > M_PI) { a -= 2.0 * M_PI; }
    while (a < -M_PI) { a += 2.0 * M_PI; }
    return a;
  }

  // ---- 参数 / 配置 ----
  std::string cloud_topic_;
  std::string prior_map_path_;
  double map_poll_period_sec_{1.0};
  bool enabled_{false};
  double rate_hz_{0.5};
  double source_voxel_{0.2};
  double target_voxel_{0.2};
  int source_max_points_{8000};
  double source_range_max_{20.0};
  double gicp_max_corr_dist_{2.0};
  int gicp_max_iter_{50};
  double gicp_fitness_max_{2.0};
  double fitness_inlier_min_{0.3};
  double correction_max_delta_m_{1.0};
  double correction_max_speed_mps_{2.0};
  bool compensate_odom_{false};
  std::string odom_topic_;
  double ema_alpha_{0.5};
  // 首次配准的初始位姿基座 (由 init_x/init_y/init_yaw 参数注入)
  double init_x_{0.0};
  double init_y_{0.0};
  double init_yaw_{0.0};
  // 首次配准尝试的 yaw 假设个数 (8 = 每 45° 一次; 有可信 init_yaw 时设 1)
  int init_yaw_hypotheses_{8};
  std::string pose_correction_topic_;
  std::string fitness_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  bool broadcast_tf_{true};
  double tf_publish_rate_{10.0};
  bool correction_z_locked_{true};
  std::string prior_map_topic_;

  // ---- 状态 ----
  std::mutex cloud_mutex_;
  CloudT::Ptr latest_cloud_;
  std::mutex map_mutex_;
  CloudT::Ptr target_cloud_;
  pcl::KdTreeFLANN<PointT>::Ptr target_tree_;
  bool map_ready_{false};
  bool need_init_{true};
  std::optional<std::filesystem::file_time_type> known_mtime_;
  std::atomic<bool> recreate_align_requested_{false};
  // 当前接受的 map->odom 修正 (T_map_odom)
  double corr_x_{0.0}, corr_y_{0.0}, corr_z_{0.0}, corr_yaw_{0.0};
  bool has_correction_{false};
  // 跳变速率门限用的实际帧间隔 / 上次配准时刻
  double dt_{0.0};
  std::chrono::steady_clock::time_point last_align_tp_{};
  // 里程计 (仅 compensate_odom=true 时参与跳变预测)
  std::mutex odom_mutex_;
  double odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0};
  bool has_odom_{false};
  // 上次接受修正时锁存的里程计位姿
  double odom_at_corr_x_{0.0}, odom_at_corr_y_{0.0};
  bool has_odom_at_corr_{false};

  // ---- ROS 接口 ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    correction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr map_poll_timer_;
  rclcpp::TimerBase::SharedPtr align_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relocalization3DNode>());
  rclcpp::shutdown();
  return 0;
}
