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