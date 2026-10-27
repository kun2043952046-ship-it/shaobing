# 04 节点装配与回调（src/）

来源: `src/test_navigation.cpp`。这些函数与 ROS 强绑定（创建器/回调/系统调用），留给 `src/` 的节点类。
迁入目录: 工作区顶层 `Navigation/src/`。建议拆成 `main.cpp` + `realtime_pgm_node.cpp` + `callbacks.cpp` + `parameter_callback.cpp` + `can_socket.cpp`，类声明放 `src/realtime_pgm_node.hpp`。

## 1. 节点装配与主函数（src/）

构造函数装配区 + main + 启动期初始化。

### `constructor_assembly`

- **源码行**: `605-815` (211 行)
- **目标位置**: `src/realtime_pgm_node.cpp`
- **职责**: 构造函数装配区: 创建 6 个订阅 / 12 个发布器 / 2 个定时器 / 2 个服务客户端 / 参数回调句柄
- **依赖 / 备注**: block(605-815); 参数声明区(118-604)见 01_config.md

```cpp
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
```

### `initDebugLog`

- **源码行**: `824-864` (41 行)
- **目标位置**: `src/realtime_pgm_node.cpp`
- **职责**: 启动期一次性初始化: 建 debug_log_dir_ 目录, 打开 cloud_frames.csv 写表头
- **依赖 / 备注**: 读 debug_* 参数

```cpp
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

### `printConfig`

- **源码行**: `7471-7567` (97 行)
- **目标位置**: `src/realtime_pgm_node.cpp`
- **职责**: 把全部生效配置打印到日志 (含话题名与 CAN 状态)
- **依赖 / 备注**: 依赖几乎所有参数成员; 拆分后建议改为 config 结构体的 to_string

```cpp
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
```

### `createMotionTimer`

- **源码行**: `3631-3649` (19 行)
- **目标位置**: `src/realtime_pgm_node.cpp`
- **职责**: 按 motion_rate_hz_ 取消并重建 wall timer (频率热改)
- **依赖 / 备注**: 写 motion_timer_

```cpp
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

### `main`

- **源码行**: `7927-7937` (11 行)
- **目标位置**: `src/main.cpp`
- **职责**: rclcpp::init -> 构造 RealtimePgmNode -> spin -> shutdown
- **依赖 / 备注**: 唯一自由函数, 原样搬运

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RealtimePgmNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
```

## 2. 订阅回调（src/）

回调本身保持很薄：解析消息 + 写成员 + 交给 tasks。

### `odomCallback`

- **源码行**: `1045-1147` (103 行)
- **目标位置**: `src/callbacks.cpp`
- **职责**: 订阅 /Odometry: 解 yaw -> 位姿跳变检测并开启"拒绝窗口" -> 施加 map->odom 修正 -> 写车体位姿
- **依赖 / 备注**: 写 odom_* / pose_jump_*, 加 motion_mutex_

```cpp
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
```

### `planCallback`

- **源码行**: `1150-1174` (25 行)
- **目标位置**: `src/callbacks.cpp`
- **职责**: 缓存 Nav2 全局路径点 + 本机接收时刻 (新鲜度监督)
- **依赖 / 备注**: 写 motion_plan_*

```cpp
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
```

### `localPlanCallback`

- **源码行**: `1179-1201` (23 行)
- **目标位置**: `src/callbacks.cpp`
- **职责**: 缓存局部路径(少于 2 点丢弃), 接收时刻用本机时钟
- **依赖 / 备注**: 写 local_plan_*

```cpp
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
```

### `goalPoseCallback`

- **源码行**: `1205-1215` (11 行)
- **目标位置**: `src/callbacks.cpp`
- **职责**: 存网页 A* 终点 (x,y), 清零到达校验重试计数
- **依赖 / 备注**: 写 goal_x_/goal_y_/motion_goal_check_*

```cpp
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
```

## 3. 定时器回调 / 运动状态机

motionTimerCallback 是全文件最长的函数（1483 行），排查/改动成本最高，建议按文末"二次拆解"清单拆段。

### `renderTimerCallback`

- **源码行**: `3363-3425` (63 行)
- **目标位置**: `src/callbacks.cpp`
- **职责**: 渲染定时器: 消费 导出/导入/清史 请求 -> 快照点云与配置 -> 分派 renderPgm / renderHistoryPgm / renderDynamicObstaclePgm
- **依赖 / 备注**: 定时器回调

```cpp
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
```

### `motionTimerCallback`

- **源码行**: `3651-4133` (483 行)
- **目标位置**: `src/callbacks.cpp (流程段建议拆到 tasks/motion_pipeline)`
- **职责**: 运动主状态机: 按钮请求 -> 追击接管 -> 占点接管 -> 取路径(局部优先) -> A* 兜底 -> 正常跟随 -> 到点校验重试 -> 发帧
- **依赖 / 备注**: 可拆为 chaseStep(3669-3722) / holdPointStep(3731-3788) / astarFallbackStep(3899-3954) / goalCheckStep(3978-4119)

```cpp
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

## 4. 参数回调

2580-3311 共 732 行, 建议只保留"解析 -> 校验 -> 提交"三段, 参数表外置到 config。

### `parameterCallback`

- **源码行**: `2580-3311` (732 行)
- **目标位置**: `src/parameter_callback.cpp`
- **职责**: 参数热更新: 拷影子变量 -> 逐参数解析 -> 集中校验 -> 分锁提交 -> 按需重建定时器/静态基准 -> 汇总日志
- **依赖 / 备注**: 参数分组见 01_config.md

```cpp
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

## 5. SocketCAN 直发

系统调用层, 建议留在 src（或整体搬 tools/can_bus 并把 publish 部分留节点）。

### `openCanSocket`

- **源码行**: `3487-3536` (50 行)
- **目标位置**: `src/can_socket.cpp`
- **职责**: socket / ioctl(SIOCGIFINDEX) / bind 建 SocketCAN 链路, 失败打错并返回 false (下次重试)
- **依赖 / 备注**: 读 can_interface_/can_id_/can_send_period_ms_

```cpp
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
```

### `closeCanSocket`

- **源码行**: `3478-3484` (7 行)
- **目标位置**: `src/can_socket.cpp`
- **职责**: 关闭 CAN fd 并置 -1
- **依赖 / 备注**: 写 can_socket_fd_

```cpp
  void closeCanSocket()
  {
    if (can_socket_fd_ >= 0) {
      ::close(can_socket_fd_);
      can_socket_fd_ = -1;
    }
  }
```

### `canFrameFromMotion`

- **源码行**: `3540-3586` (47 行)
- **目标位置**: `src/can_socket.cpp`
- **职责**: 按 can_send_period_ms_ 节流组 can_frame 直写总线; 失败关 socket 待重连, 成功累加计数
- **依赖 / 备注**: 读写 can_last_send_ / can_sent_frames_

```cpp
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
