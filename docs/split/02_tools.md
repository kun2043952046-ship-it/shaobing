# 02 纯计算（tools/）

**约定**：`tools/` 只做数学、几何、编解码 —— 不 include ROS、不碰 socket/文件、不加锁。
迁入顶层 `Navigation/tools/`，统一 `namespace nav::tools`，编译成 `nav_tools` 静态库。

> 文件读写（PCD/BIN/YAML）已划到 `io/file_*`；ROS 发布全部在 `io/ros_*`。

## 1. 数据类型与协议常量

结构体与 constexpr 从节点类里搬出，成为 tools 的公共类型；本层零 ROS、零 IO。

### `consts`

- **源码行**: `46-55` (10 行)
- **目标位置**: `tools/constants.hpp`
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
- **目标位置**: `tools/types.hpp`
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
- **目标位置**: `tools/types.hpp`
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
- **目标位置**: `tools/types.hpp`
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

### `PoseCorrection`

- **源码行**: `1233-1238` (6 行)
- **目标位置**: `tools/types.hpp`
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
- **目标位置**: `tools/types.hpp`
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
- **目标位置**: `tools/types.hpp`
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
- **目标位置**: `tools/types.hpp`
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

## 2. 几何 / 坐标变换

全部只依赖入参；建议 `namespace nav::tools`。

### `hasField`

- **源码行**: `1217-1227` (11 行)
- **目标位置**: `tools/ros_utils.hpp / ros_utils.cpp`
- **职责**: 检查 PointCloud2 是否含指定字段名 (static, 无成员访问)
- **依赖 / 备注**: 入参 sensor_msgs::msg::PointCloud2, 纯查询

```cpp
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

### `quaternionToMatrix`

- **源码行**: `1240-1265` (26 行)
- **目标位置**: `tools/tf_math.hpp (inline)`
- **职责**: 四元数 -> 行主序 3x3 旋转矩阵, 含归一化与退化时取单位阵
- **依赖 / 备注**: static; 入参 x,y,z,w; 出参 double r[9]

```cpp
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
```

### `matrixToQuaternion`

- **源码行**: `1267-1298` (32 行)
- **目标位置**: `tools/tf_math.hpp (inline)`
- **职责**: 3x3 旋转矩阵 -> 四元数 (trace 分支法)
- **依赖 / 备注**: static; 出参 x,y,z,w

```cpp
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
```

### `yawFromMatrix`

- **源码行**: `1301-1304` (4 行)
- **目标位置**: `tools/tf_math.hpp (inline)`
- **职责**: 从行主序 3x3 取纯 z 旋转偏航角 atan2(r[3], r[0])
- **依赖 / 备注**: static; 仅两元素

```cpp
  static double yawFromMatrix(const double r[9])
  {
    return std::atan2(r[3], r[0]);
  }
```

### `wrapAngle`

- **源码行**: `1307-1318` (12 行)
- **目标位置**: `tools/tf_math.hpp (inline)`
- **职责**: 角度归一化到 (-pi, pi]
- **依赖 / 备注**: static; 纯标量

```cpp
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
```

### `multiplyMatrix`

- **源码行**: `1321-1333` (13 行)
- **目标位置**: `tools/tf_math.hpp (inline)`
- **职责**: 3x3 行主序矩阵相乘
- **依赖 / 备注**: static; 纯标量

```cpp
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
```

### `transformPointByCorrection`

- **源码行**: `1398-1409` (12 行)
- **目标位置**: `tools/tf_math.hpp (inline)`
- **职责**: 单点刚体变换 p_map = R * p + t
- **依赖 / 备注**: const; 入参 Point + PoseCorrection

```cpp
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

### `score_at`

- **源码行**: `1469-1505` (37 行)
- **目标位置**: `tools/registration.hpp`
- **职责**: 位姿命中率评分 (estimatePoseCorrection 内部的 lambda): 采样点变换后查栅格, 出界计入分母
- **依赖 / 备注**: block(1469-1505); 捕获 samples/config/mask/width/height

```cpp
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

### `transformToBaseLink`

- **源码行**: `2454-2494` (41 行)
- **目标位置**: `tools/tf_math.cpp`
- **职责**: map -> base_link: 先用 odom + 雷达外参反推车体中心, 再 R(-yaw)*(p-base)
- **依赖 / 备注**: 读 odom_x_/odom_y_/odom_yaw_/odom_received_/motion_base_to_lidar_x,y_; 改造成入参即纯函数

```cpp
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

## 3. 运动帧编解码（12 位有符号量协议）

纯位运算与量化，可单独做单元测试。

### `limitMotionSpeed`

- **源码行**: `3590-3601` (12 行)
- **目标位置**: `tools/motion_codec.hpp`
- **职责**: 速度保方向等比限幅: 峰值超 kMotionMaxSpeed 时两轴同比缩放
- **依赖 / 备注**: static

```cpp
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
```

### `effectiveSlowRadius`

- **源码行**: `3606-3615` (10 行)
- **目标位置**: `tools/motion_codec.hpp`
- **职责**: 减速起始距离: 手动值优先, 否则 max(阈值+0.05, kSlowRadiusTime*目标速度)
- **依赖 / 备注**: static

```cpp
  static double effectiveSlowRadius(
    double slow_radius, double target_speed, double goal_tolerance)
  {
    if (slow_radius > 0.0) {
      return slow_radius;
    }

    const double speed = std::clamp(target_speed, 0.0, 5.0);
    return std::max(goal_tolerance + 0.05, kSlowRadiusTime * speed);
  }
```

### `encodeMotionRaw`

- **源码行**: `3619-3628` (10 行)
- **目标位置**: `tools/motion_codec.hpp`
- **职责**: m/s -> 12 位有符号原始量: 就近量化 + ±2040 饱和 + &0x0FFF
- **依赖 / 备注**: static

```cpp
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
```

### `resolveAstarSpeed`

- **源码行**: `5808-5812` (5 行)
- **目标位置**: `tools/motion_codec.hpp`
- **职责**: A* 兜底速度解析: <=0 时回落到 motion_target_speed_, 并夹到 kMotionMaxSpeed
- **依赖 / 备注**: const

```cpp
  double resolveAstarSpeed() const
  {
    const double base = (astar_speed_ > 0.0) ? astar_speed_ : motion_target_speed_;
    return std::clamp(base, 0.0, kMotionMaxSpeed);
  }
```

## 4. 栅格几何与像素

占据栅格/像素层面的纯运算，拆分收益最大的一块。

### `computeMapSize`

- **源码行**: `6153-6191` (39 行)
- **目标位置**: `tools/grid_geom.cpp`
- **职责**: 由 Config 的地图范围/分辨率算 width/height, 并校验像素总数上限
- **依赖 / 备注**: 准纯 (仅 RCLCPP_ERROR_THROTTLE 日志, 建议改为返回错误码)

```cpp
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
```

### `computeVehicleFootprint`

- **源码行**: `6210-6253` (44 行)
- **目标位置**: `tools/grid_geom.cpp`
- **职责**: 由里程计位姿 + self_filter_size 算出世界系轴对齐车体方框 FootprintBounds
- **依赖 / 备注**: 读 odom_* + config_.self_filter_size + 外参, 入参化后可纯

```cpp
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
```

### `clipCloudToMapBounds`

- **源码行**: `6262-6294` (33 行)
- **目标位置**: `tools/grid_geom.cpp`
- **职责**: 按半开区间 [xmin,xmax) x [ymin,ymax) 就地裁剪点云, 返回丢弃点数
- **依赖 / 备注**: 读 config_ 四个边界

```cpp
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
```

### `filterVehicleFootprintPoints`

- **源码行**: `6301-6327` (27 行)
- **目标位置**: `tools/grid_geom.cpp`
- **职责**: 点级自过滤: 丢弃落在车体方框(闭区间)内的本体点
- **依赖 / 备注**: 调用 computeVehicleFootprint

```cpp
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
```

### `clearVehicleFootprint`

- **源码行**: `6337-6390` (54 行)
- **目标位置**: `tools/grid.cpp`
- **职责**: 栅格级清除: 车体方框内占据格一律置空
- **依赖 / 备注**: 准纯, 仅 RCLCPP_DEBUG

```cpp
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
```

### `accumulateCloud`

- **源码行**: `6395-6459` (65 行)
- **目标位置**: `tools/grid.cpp`
- **职责**: 单帧点云投影入占据栅格 (z 过滤), 支持"同帧只计一次"的命中计数模式
- **依赖 / 备注**: 零成员零 ROS, 最干净的抽取对象

```cpp
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
```

### `applyDilate`

- **源码行**: `6462-6519` (58 行)
- **目标位置**: `tools/grid.cpp`
- **职责**: 占据栅格圆形(方形核)膨胀指定格数
- **依赖 / 备注**: 零成员零 ROS

```cpp
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
```

### `occupancyToPixels`

- **源码行**: `6522-6552` (31 行)
- **目标位置**: `tools/grid.cpp`
- **职责**: 占据栅格 -> mono8 像素 (0=占据, 205=未知), 行序按 PGM 约定翻转
- **依赖 / 备注**: 零成员零 ROS

```cpp
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

## 5. 栅格查询 / 聚类 / 间距判定

占点与追击共用，属通用几何算法。

### `extractHoldObstacles`

- **源码行**: `4726-4818` (93 行)
- **目标位置**: `tools/grid_cluster.cpp`
- **职责**: 按固定步长扫描占据栅格, 在线聚类(加权桶)求障碍簇质心, 只保留范围内且间距达标的簇
- **依赖 / 备注**: 含局部 struct Bucket; 读 astar_inflate_radius_

```cpp
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
```

### `computeHoldBlockPose`

- **源码行**: `4825-4855` (31 行)
- **目标位置**: `tools/grid_geom.cpp`
- **职责**: 沿"障碍 -> 中心"连线取距障碍 standoff 的阻挡站位点
- **依赖 / 备注**: const; standoff 入参化即纯

```cpp
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

### `isHoldPointClear`

- **源码行**: `4863-4928` (66 行)
- **目标位置**: `tools/grid_clearance.cpp`
- **职责**: 点级硬性间距自检: 圆盘邻域内不得有未豁免的占据格
- **依赖 / 备注**: const; 参数全传入, 已是纯栅格查询

```cpp
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
```

### `isHoldPointClearPath`

- **源码行**: `4934-5012` (79 行)
- **目标位置**: `tools/grid_clearance.cpp`
- **职责**: 线段级硬性间距自检: 等间隔采样(上限 4000 点)后逐点判定
- **依赖 / 备注**: const; 复用 isHoldPointClear 判据

```cpp
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
```

### `findNearestObstacleCluster`

- **源码行**: `5318-5347` (30 行)
- **目标位置**: `tools/grid_cluster.cpp`
- **职责**: 在 (ref_x,ref_y) 为圆心、radius 为半径内取最近障碍簇质心
- **依赖 / 备注**: 调用 extractHoldObstacles

```cpp
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

### `chaseLineClear`

- **源码行**: `5352-5367` (16 行)
- **目标位置**: `tools/grid_clearance.cpp`
- **职责**: 车体 -> 目标连线是否通畅(豁免目标本体), 决定直连还是 A* 绕行
- **依赖 / 备注**: 读 astar_inflate_radius_

```cpp
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
