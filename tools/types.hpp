  struct Point
  {
    float x;
    float y;
    float z;
  };
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
    struct PoseCorrection
  {
    double r[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};   // 行主序 计算位姿的矩阵
    double t[3] = {0.0, 0.0, 0.0};
    bool valid = false;
  };
    struct HoldObstacleCluster
  {
    double x{0.0};
    double y{0.0};
  };
    struct FootprintBounds//车体自过滤 AABB: valid / xmin / xmax / ymin / ymax
  {
    bool valid{false};      // false: 无里程计或已关闭自过滤, 不做任何过滤
    double xmin{0.0};
    double xmax{0.0};
    double ymin{0.0};
    double ymax{0.0};
  };
    struct ChaseTrackPoint//追击坐标
  {
    rclcpp::Time stamp;
    double x{0.0};
    double y{0.0};
    std::vector<Point> cloud;   // 记录该时刻目标簇的 3D 点云 (map 系)
  };