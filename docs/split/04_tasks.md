# 04 业务流程（tasks/）

**约定**：`tasks/` 是"做什么" —— 编排 io 的收发与 tools 的计算，持有状态与状态机；自己不做数学、不发消息、不写文件。
迁入顶层 `Navigation/tasks/`，每小节一个类：`CloudPipeline` / `DynamicSplitter` / `PoseCorrector` / `MapRenderer` / `MapIo` / `AstarPlanner` / `HoldPointTask` / `ChaseTask` / `MotionController` / `DebugLogger`。

## 1. 点云流水线 / 动态障碍分离

cloudCallback 是全文件最强耦合点（4 把锁），建议先抽 tools/io 再搬它。

### `cloudCallback`

- **源码行**: `1715-1949` (235 行)
- **目标位置**: `tasks/cloud_pipeline.cpp`
- **职责**: 点云主流水线: 字段校验 -> 解析+isfinite -> 跳变拒绝窗丢弃 -> 位姿修正 -> 地图裁剪 -> 车体点级自过滤 -> 发自过滤点云 -> 动态障碍分离/绿层/清除环 -> 历史降采样 -> 调试日志 -> 入队
- **依赖 / 备注**: 订阅回调; 涉及 cloud/dynamic/config/motion 四把锁; 调用 8 个 tools 函数

```cpp
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
```

### `pruneHistory`

- **源码行**: `1952-1973` (22 行)
- **目标位置**: `tasks/cloud_pipeline.cpp`
- **职责**: 按 history_duration 淘汰历史队列队首; 动态障碍模式下直接跳过
- **依赖 / 备注**: 读写 history_

```cpp
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

### `logCloudFrame`

- **源码行**: `917-1042` (126 行)
- **目标位置**: `tasks/debug_log.cpp`
- **职责**: 逐帧统计质心/包围盒, 判定点数突增与质心跳变, 写 CSV, 异常时落 PCD 快照
- **依赖 / 备注**: 读多个 debug_* 阈值成员

```cpp
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

### `splitDynamicObstacles`

- **源码行**: `1985-2077` (93 行)
- **目标位置**: `tasks/dynamic_split.cpp`
- **职责**: 用冻结的静态基准掩码把"新出现的临时障碍点"从整帧中原地摘出
- **依赖 / 备注**: 读写 dynamic_mutex_ 下的 static_obstacle_mask_

```cpp
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
```

### `subtractBaseline`

- **源码行**: `2087-2156` (70 行)
- **目标位置**: `tasks/dynamic_split.cpp`
- **职责**: 同判据但不改输入, 输出"整帧 - 基准占据"的动态点 (绿色层数据源)
- **依赖 / 备注**: 与 splitDynamicObstacles 共用判据

```cpp
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
```

### `buildStaticBaseline`

- **源码行**: `2161-2320` (160 行)
- **目标位置**: `tasks/dynamic_split.cpp`
- **职责**: 锁定瞬间: 历史累积图按 history_min_hits 投票 + 并入成品图 + 扣车体 + baseline_dilate 膨胀 -> 基准栅格; 再膨胀 1 格得到配准评分容差掩码
- **依赖 / 备注**: 写 static_obstacle_mask_ / static_obstacle_tolerance_

```cpp
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

## 2. 位姿修正（map->odom 2D 配准）

流程与状态在 tasks；评分内核在 tools/registration，发布在 io/ros_pose_pub。

### `getPoseCorrection`

- **源码行**: `1335-1339` (5 行)
- **目标位置**: `tasks/pose_correction.cpp`
- **职责**: 加锁拷贝当前 map->odom 修正量
- **依赖 / 备注**: 读 pose_correction_mutex_

```cpp
  PoseCorrection getPoseCorrection()
  {
    std::lock_guard<std::mutex> lock(pose_correction_mutex_);
    return pose_correction_;
  }
```

### `setPoseCorrection2D`

- **源码行**: `1362-1378` (17 行)
- **目标位置**: `tasks/pose_correction.cpp`
- **职责**: (x,y,yaw) -> SE(3) 写入成员并广播
- **依赖 / 备注**: 写 pose_correction_, 经 publishPoseCorrection

```cpp
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
```

### `resetPoseCorrection`

- **源码行**: `1380-1396` (17 行)
- **目标位置**: `tasks/pose_correction.cpp`
- **职责**: 置恒等修正; 仅 apply_pose_correction_ 为真时广播
- **依赖 / 备注**: 写 pose_correction_

```cpp
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
```

### `estimatePoseCorrection`

- **源码行**: `1411-1688` (278 行)
- **目标位置**: `tasks/pose_correction.cpp (+ tools/registration.hpp 评分内核)`
- **职责**: 2D 栅格配准: 粗搜(±range/步长) + 亚格细化 + 单帧速率限幅 + 滞回/EMA/死区; 失败计数达阈值触发全局重搜; 成功则 setPoseCorrection2D
- **依赖 / 备注**: 读 config_ + static_obstacle_mask_/tolerance_; 写原子失败计数; 内核 score_at 在 tools

```cpp
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
```

### `correctCloudForPose`

- **源码行**: `1690-1713` (24 行)
- **目标位置**: `tasks/pose_correction.cpp`
- **职责**: 按 apply_pose_correction_ 二选一(自估 / 外部修正)后把整帧点云变换到 map 系
- **依赖 / 备注**: 读 pose_correction_ / dynamic_obstacle_enabled_

```cpp
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

## 3. 建图渲染

渲染编排（栅格运算在 tools，写 PGM 在 io）。

### `renderPgm`

- **源码行**: `6554-6655` (102 行)
- **目标位置**: `tasks/map_render.cpp`
- **职责**: 单帧建图流程: 算尺寸 -> 累积点云 -> 扣车体 -> 膨胀 -> 转像素 -> 发 Image -> 写 PGM 文件
- **依赖 / 备注**: 发布; 文件 IO

```cpp
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
```

### `renderHistoryPgm`

- **源码行**: `6659-6827` (169 行)
- **目标位置**: `tasks/map_render.cpp`
- **职责**: 历史多帧投票累积渲染 -> 占据比例暴涨检测(WARN) -> 发 Image -> 写 PGM
- **依赖 / 备注**: 读 debug_prev_occ_* 统计

```cpp
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
```

### `renderDynamicObstaclePgm`

- **源码行**: `7279-7322` (44 行)
- **目标位置**: `tasks/map_render.cpp`
- **职责**: 动态障碍栅格 -> 绿色层 Image 发布
- **依赖 / 备注**: 写 dynamic_image_published_

```cpp
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

## 4. 地图导入导出

流程编排 + 状态重建（字节读写已下沉 io/file_*）。

### `exportHistoryMap`

- **源码行**: `7046-7155` (110 行)
- **目标位置**: `tasks/map_io.cpp`
- **职责**: 导出编排: 渲染历史图 + 写 PGM/YAML/BIN
- **依赖 / 备注**: 读 history_ / config_ / export_map_dir_

```cpp
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
```

### `importHistoryMap`

- **源码行**: `7158-7261` (104 行)
- **目标位置**: `tasks/map_io.cpp`
- **职责**: 导入编排: 读 PGM/BIN -> 重建 history_ 与 imported_map_, 清 Nav2 代价图
- **依赖 / 备注**: 调用 buildStaticBaseline / clearCostmaps

```cpp
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
```

### `mergeImportedMap`

- **源码行**: `7026-7043` (18 行)
- **目标位置**: `tasks/map_io.cpp`
- **职责**: 导入图占据格叠加到当前像素(仅覆盖未知格)
- **依赖 / 备注**: 读写 imported_map_*

```cpp
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

## 5. A* 规划

缓存/限频/目标解析留 tasks，搜索内核建议下沉 tools/astar。

### `buildMergedOccupancy`

- **源码行**: `4154-4214` (61 行)
- **目标位置**: `tasks/astar_planner.cpp`
- **职责**: 合并占据栅格 = 历史累计 ∪ 动态障碍(同一 accumulateCloud/applyDilate 口径), 再扣车体方框; A* / 占点 / 追击共用
- **依赖 / 备注**: 读 history_ / latest_dynamic_obstacles_

```cpp
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
```

### `planAstarPath`

- **源码行**: `4239-4265` (27 行)
- **目标位置**: `tasks/astar_planner.cpp`
- **职责**: A* 入口: 把雷达位姿反推为车体中心, 以 Web 目标 goal_x_/goal_y_ 为终点调用 planAstarToGoal
- **依赖 / 备注**: 读 astar_enabled_ / goal_received_

```cpp
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
```

### `planAstarToGoal`

- **源码行**: `4273-4602` (330 行)
- **目标位置**: `tasks/astar_planner.cpp (+ tools/astar.cpp 内核)`
- **职责**: A* 核心: 建图 -> 膨胀 -> 障碍代价场 -> 八连通搜索 -> 回溯 -> 弧长抽稀
- **依赖 / 备注**: 内核可拆: inflateGrid / findNearestFreeCell / buildObstacleCostField / astarSearch / backtrackCells / cellsToWorld / resampleByArcLength

```cpp
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
```

### `getAstarPath`

- **源码行**: `4612-4669` (58 行)
- **目标位置**: `tasks/astar_planner.cpp`
- **职责**: 带限频的规划入口: 按时间或位移判断是否重规划, 否则复用缓存
- **依赖 / 备注**: 读写 astar_cache_valid_ / astar_cached_path_ / astar_last_plan_*

```cpp
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

## 6. 占点行为

业务状态机 + 复用 tools 的聚类/间距判定。

### `computeHoldPointCommand`

- **源码行**: `5017-5217` (201 行)
- **目标位置**: `tasks/hold_point.cpp`
- **职责**: 占点主流程: 读参数 -> 建图 -> 聚簇 -> 仲裁最近威胁 -> 算站位点 -> 硬性间距自检 -> 跟随到站 -> 机动安全兜底 -> 记录状态 -> 发可视化
- **依赖 / 备注**: 读写 hold_point_*; 复用 motion_* 参数; 调 tools 的 extractHoldObstacles / computeHoldBlockPose / isHoldPointClearPath

```cpp
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

## 7. 追击行为

业务状态机 + 目标轨迹记录。

### `computeChaseCommand`

- **源码行**: `5371-5633` (263 行)
- **目标位置**: `tasks/chase.cpp`
- **职责**: 追击主流程: 锁定/跟踪目标 -> 节流记录轨迹 -> 算环带站位点 -> 连线检查 -> A* 或直连跟随 -> 安全兜底 -> 记录 + 发布
- **依赖 / 备注**: 读写 chase_*; 调 extractHoldObstacles / findNearestObstacleCluster / chaseLineClear / planAstarToGoal

```cpp
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
```

### `recordTargetCloud`

- **源码行**: `5638-5671` (34 行)
- **目标位置**: `tasks/chase.cpp`
- **职责**: 把目标邻域(半径约 radius*1.5)内的 3D 点云连同时间戳追加到 chase_track_, 按时长淘汰
- **依赖 / 备注**: 读 latest_cloud_, 写 chase_track_

```cpp
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

## 8. 运动控制 / 路径跟随

几何内核下沉 tools/pure_pursuit，类只保留状态机。

### `computeFollowCommand`

- **源码行**: `5823-6035` (213 行)
- **目标位置**: `tasks/motion_control.cpp (+ tools/pure_pursuit.cpp 几何内核)`
- **职责**: 跟随核心: 外参反推车体中心 -> 到达判定 -> 前视半平面最近点 -> 横向偏差保护 -> 梯形减速 -> 速度自适应 lookahead -> 弧长推进取前视点 -> 投影成车体系 (vx,vy) -> 写前视可视化
- **依赖 / 备注**: 内核可拆: baseFromLidar / nearestOnPlan / trapezoidSpeed / lookaheadByArc / projectToBody

```cpp
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
```

### `computeAstarFollowCommand`

- **源码行**: `5792-5804` (13 行)
- **目标位置**: `tasks/motion_control.cpp`
- **职责**: A* 兜底路径跟随薄壳: 解析巡航速度后转调 computeFollowCommand
- **依赖 / 备注**: 读 astar_speed_ / motion_target_speed_

```cpp
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
```

### `startMotion`

- **源码行**: `6038-6120` (83 行)
- **目标位置**: `tasks/motion_control.cpp`
- **职责**: 开始运动: 校验里程计与 /plan (无 plan 但有目标点时允许, 交给 A* 兜底), 置运动态
- **依赖 / 备注**: 读写 motion_active_ / motion_waiting_ / astar_cache_valid_

```cpp
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
```

### `stopMotion`

- **源码行**: `6122-6150` (29 行)
- **目标位置**: `tasks/motion_control.cpp`
- **职责**: 停车: 清运动/等待/重规划标志, 作废前视可视化, 下发 0 速帧
- **依赖 / 备注**: 调用 publishMotionFrame

```cpp
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
