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
  }// 四元数 -> 行主序 3x3 旋转矩阵, 含归一化与退化时取单位阵
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
  }// 3x3 旋转矩阵 -> 四元数
  static double yawFromMatrix(const double r[9])
  {
    return std::atan2(r[3], r[0]);
  }//从行主序 3x3 取纯 z 旋转偏航角 atan2(r[3], r[0])
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
  }//3x3 行主序矩阵相乘
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
  }//单点刚体变换 p_map = R * p + t
  2D位姿命中率评分 (estimatePoseCorrection 内部的 lambda): 采样点变换后查栅格, 出界计入分母
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
      }
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
   static void limitMotionSpeed(double & vx, double & vy)
  {
    const double peak = std::max(std::fabs(vx), std::fabs(vy));

    if (peak <= kMotionMaxSpeed || peak <= 0.0) {
      return;
    }

    const double ratio = kMotionMaxSpeed / peak;
    vx *= ratio;
    vy *= ratio;
  }//速度保方向等比限幅: 峰值超 kMotionMaxSpeed 时两轴同比缩放
  static double effectiveSlowRadius(
    double slow_radius, double target_speed, double goal_tolerance)
  {
    if (slow_radius > 0.0) {
      return slow_radius;
    }

    const double speed = std::clamp(target_speed, 0.0, 5.0);
    return std::max(goal_tolerance + 0.05, kSlowRadiusTime * speed);
  }//减速起始距离: 手动值优先, 否则 max(阈值+0.05, kSlowRadiusTime*目标速度)
    static double effectiveSlowRadius(
    double slow_radius, double target_speed, double goal_tolerance)
  {
    if (slow_radius > 0.0) {
      return slow_radius;
    }

    const double speed = std::clamp(target_speed, 0.0, 5.0);
    return std::max(goal_tolerance + 0.05, kSlowRadiusTime * speed);
  }
  // 减速起始距离: 手动值优先, 否则 max(阈值+0.05, kSlowRadiusTime*目标速度)
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
  // 12 位有符号原始量: 就近量化 + ±2040 饱和 + &0x0FFF
  
