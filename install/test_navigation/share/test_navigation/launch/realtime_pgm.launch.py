# 一键启动: Livox 驱动 + FAST-LIO + 实时 PGM 绘制节点
#
# 用法一（完整链路: 驱动 + FAST-LIO + 实时PGM）:
#   ros2 launch test_navigation realtime_pgm.launch.py
#
# 用法二（覆盖参数）:
#   ros2 launch test_navigation realtime_pgm.launch.py \
#     resolution:=0.05 zmin:=-2.25 zmax:=0.22 \
#     dilate:=2 \
#     history_duration:=300.0 history_min_hits:=2
#
# 用法三（FAST-LIO 已单独运行, 只启动 PGM 节点）:
#   ros2 launch test_navigation realtime_pgm.launch.py with_driver:=false
#
# 用法四（不启动查看窗口, 无桌面/SSH 环境使用）:
#   ros2 launch test_navigation realtime_pgm.launch.py with_viewer:=false
#
# 用法五（把运动帧直接写进 CAN 总线, 不需要任何桥接节点/脚本）:
#   # 先拉起 CAN 接口 (经典 CAN 1 Mbps, 以 can0 为例)
#   sudo ip link set can0 down
#   sudo ip link set can0 type can bitrate 1000000
#   sudo ip link set can0 up
#
#   ros2 launch test_navigation realtime_pgm.launch.py can_enabled:=true
#
#   # 另开终端验证本节点发出的 0xFF 报文:
#   candump -t d can0,0FF:7FF
#
# 注意: 下方 fast_lio / livox_ros_driver2 的 launch 路径以 Ericsii/FAST_LIO_ROS2
# 和官方 livox_ros_driver2 为准, 如果你用的是别的 fork, 按其 README 改这两处。
#
# 一键执行（云端机器上）:
# cd /home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation
# source /opt/ros/humble/setup.bash
# source /home/lele/source_code_from_git/ws_livox/install/setup.bash   # 驱动 + fast_lio
# source install/setup.bash                                            # 本包
#
# ros2 launch test_navigation realtime_pgm.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
# RewrittenYaml: 在 launch 阶段把参数文件里的某些值改写成"运行时才能确定的
# 绝对路径"(如本包 config/ 下的行为树 XML), 既保证 Nav2 能打开文件, 又不必把
# 机器相关路径硬编码进 yaml。nav2_bringup 自己也是这么处理 map yaml 的。
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    with_viewer = LaunchConfiguration('with_viewer')
    robot_length = LaunchConfiguration('robot_length')
    robot_width = LaunchConfiguration('robot_width')
    lidar_to_base_x = LaunchConfiguration('lidar_to_base_x')
    lidar_to_base_y = LaunchConfiguration('lidar_to_base_y')


    # 链路开关
    with_driver = LaunchConfiguration('with_driver')
    config_file = LaunchConfiguration('config_file')

    # Nav2 开关与参数文件
    with_nav2 = LaunchConfiguration('with_nav2')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # 3D 重定位节点 (test_relocalization 包) 开关: 锁定历史累计图后接管
    #   map->odom 位姿修正 (见 apply_pose_correction 参数)
    with_relocalization_3d = LaunchConfiguration('with_relocalization_3d')

    # 位姿诊断记录 (diag_logger.py): 记录 /Odometry 姿态与 /pose_correction
    # 修正量到 CSV, 用于定位"摇晃后地图上小车位置漂移/无法重定位"一类问题。
    with_diag = LaunchConfiguration('with_diag')
    diag_log_file = LaunchConfiguration('diag_log_file')
    diag_odom_jump_m = LaunchConfiguration('diag_odom_jump_m')
    diag_odom_jump_yaw_deg = LaunchConfiguration('diag_odom_jump_yaw_deg')

    # PGM 节点参数
    cloud_topic = LaunchConfiguration('cloud_topic')
    resolution = LaunchConfiguration('resolution')
    zmin = LaunchConfiguration('zmin')
    zmax = LaunchConfiguration('zmax')
    dilate = LaunchConfiguration('dilate')
    baseline_dilate = LaunchConfiguration('baseline_dilate')
    self_filter_size = LaunchConfiguration('self_filter_size')
    xmin = LaunchConfiguration('xmin')
    ymin = LaunchConfiguration('ymin')
    xmax = LaunchConfiguration('xmax')
    ymax = LaunchConfiguration('ymax')
    output_file = LaunchConfiguration('output')
    history_duration = LaunchConfiguration('history_duration')
    history_max_points_per_frame = LaunchConfiguration('history_max_points_per_frame')
    history_min_hits = LaunchConfiguration('history_min_hits')
    history_output = LaunchConfiguration('history_output')
    render_period_ms = LaunchConfiguration('render_period_ms')

    # 调试日志 / 异常快照 (逐帧点云统计 CSV + 异常帧 PCD 快照)
    debug_log_enabled = LaunchConfiguration('debug_log_enabled')
    debug_log_dir = LaunchConfiguration('debug_log_dir')
    debug_pcd_enabled = LaunchConfiguration('debug_pcd_enabled')
    debug_pcd_max_files = LaunchConfiguration('debug_pcd_max_files')
    debug_cloud_count_jump_ratio = LaunchConfiguration(
        'debug_cloud_count_jump_ratio')
    debug_cloud_count_jump_delta = LaunchConfiguration(
        'debug_cloud_count_jump_delta')
    debug_centroid_jump_m = LaunchConfiguration('debug_centroid_jump_m')
    debug_odom_jump_m = LaunchConfiguration('debug_odom_jump_m')
    debug_odom_jump_yaw_deg = LaunchConfiguration('debug_odom_jump_yaw_deg')
    pose_jump_reject_sec = LaunchConfiguration('pose_jump_reject_sec')
    debug_occ_ratio_jump = LaunchConfiguration('debug_occ_ratio_jump')

    # 动态障碍 (可变障碍): 锁定历史累计图后, 新出现的障碍单独成层
    dynamic_obstacle_topic = LaunchConfiguration('dynamic_obstacle_topic')
    dynamic_obstacle_image_topic = LaunchConfiguration(
        'dynamic_obstacle_image_topic')

    # 自过滤点云 (供 Nav2 obstacle_layer 的清除射线)
    self_filtered_topic = LaunchConfiguration('self_filtered_topic')
    self_filtered_publish_rate = LaunchConfiguration('self_filtered_publish_rate')
    self_filtered_max_points = LaunchConfiguration('self_filtered_max_points')

    # 清除射线点云 (让红色代价区域实时跟随绿色障碍消失)
    clear_ring_topic = LaunchConfiguration('clear_ring_topic')
    clear_ring_points = LaunchConfiguration('clear_ring_points')
    clear_ring_radius = LaunchConfiguration('clear_ring_radius')

    # 刷新历史累计图时整张清空 Nav2 红色代价区
    clear_costmap_on_history_reset = LaunchConfiguration(
        'clear_costmap_on_history_reset')
    clear_local_costmap_service = LaunchConfiguration(
        'clear_local_costmap_service')
    clear_global_costmap_service = LaunchConfiguration(
        'clear_global_costmap_service')

    # 导航运动控制 (网页"开始运动" -> 8 字节运动帧)
    motion_topic = LaunchConfiguration('motion_topic')
    local_plan_topic = LaunchConfiguration('local_plan_topic')
    local_plan_timeout = LaunchConfiguration('local_plan_timeout')
    motion_control = LaunchConfiguration('motion_control')
    motion_vx_scale = LaunchConfiguration('motion_vx_scale')
    motion_vy_scale = LaunchConfiguration('motion_vy_scale')
    motion_rate_hz = LaunchConfiguration('motion_rate_hz')
    motion_target_speed = LaunchConfiguration('motion_target_speed')
    motion_lookahead = LaunchConfiguration('motion_lookahead')
    motion_lookahead_speed_gain = LaunchConfiguration(
        'motion_lookahead_speed_gain')
    motion_lookahead_max = LaunchConfiguration('motion_lookahead_max')
    motion_yaw_kp = LaunchConfiguration('motion_yaw_kp')
    motion_goal_tolerance = LaunchConfiguration('motion_goal_tolerance')
    motion_goal_check_tolerance = LaunchConfiguration(
        'motion_goal_check_tolerance')
    motion_goal_check_max_retry = LaunchConfiguration(
        'motion_goal_check_max_retry')
    motion_max_lateral = LaunchConfiguration('motion_max_lateral')
    motion_plan_timeout = LaunchConfiguration('motion_plan_timeout')
    motion_slow_radius = LaunchConfiguration('motion_slow_radius')
    motion_min_speed = LaunchConfiguration('motion_min_speed')
    motion_base_to_lidar_x = LaunchConfiguration('motion_base_to_lidar_x')
    motion_base_to_lidar_y = LaunchConfiguration('motion_base_to_lidar_y')
    # 自建 A* 兜底规划器 (/plan 不可用时用自建 A* 规划"逃离+前往目标"路径)
    astar_enabled = LaunchConfiguration('astar_enabled')
    astar_speed = LaunchConfiguration('astar_speed')
    astar_inflate_radius = LaunchConfiguration('astar_inflate_radius')
    astar_obstacle_penalty = LaunchConfiguration('astar_obstacle_penalty')
    astar_cost_reach = LaunchConfiguration('astar_cost_reach')
    astar_path_spacing = LaunchConfiguration('astar_path_spacing')
    astar_replan_interval = LaunchConfiguration('astar_replan_interval')
    astar_replan_move_dist = LaunchConfiguration('astar_replan_move_dist')
    goal_pose_topic = LaunchConfiguration('goal_pose_topic')
    astar_plan_topic = LaunchConfiguration('astar_plan_topic')

    # 占点 (hold point) 行为
    hold_point_enabled = LaunchConfiguration('hold_point_enabled')
    hold_point_radius = LaunchConfiguration('hold_point_radius')
    hold_point_speed = LaunchConfiguration('hold_point_speed')
    hold_point_slow_radius = LaunchConfiguration('hold_point_slow_radius')
    hold_point_standoff_scale = LaunchConfiguration('hold_point_standoff_scale')
    hold_point_recenter_deadband = LaunchConfiguration(
        'hold_point_recenter_deadband')
    hold_point_status_topic = LaunchConfiguration('hold_point_status_topic')
    hold_point_viz_topic = LaunchConfiguration('hold_point_viz_topic')

    # SocketCAN 直发 (运动帧同时写进 CAN 总线, 不需要额外桥接节点)
    can_enabled = LaunchConfiguration('can_enabled')
    can_interface = LaunchConfiguration('can_interface')
    can_id = LaunchConfiguration('can_id')
    can_send_period_ms = LaunchConfiguration('can_send_period_ms')

    # 1) Livox 驱动 (CustomMsg 格式, FAST-LIO 必须用 msg_ 系列 launch)
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('livox_ros_driver2'),
            'launch_ROS2', 'msg_MID360_launch.py'])),
        condition=IfCondition(with_driver))

    # 2) FAST-LIO 建图 (MID360 直接用 avia.yaml: lidar_type=1 即 Livox CustomMsg)
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'])),
        launch_arguments={'config_file': config_file}.items(),
        condition=IfCondition(with_driver))

    # 3) 实时 PGM 绘制节点（常驻运行, Ctrl+C 一键停止全部节点）
    pgm_node = Node(
        package='test_navigation',
        executable='test_navigation_node',
        output='screen',
        parameters=[{
            'cloud_topic': cloud_topic,
            'resolution': ParameterValue(resolution, value_type=float),
            'zmin': ParameterValue(zmin, value_type=float),
            'zmax': ParameterValue(zmax, value_type=float),
            'dilate': ParameterValue(dilate, value_type=int),
            # 静态基准图膨胀量 (-1 = 沿用 dilate): 调大能避免"障碍贴住静态
            # 物表面时被误判为静态"(表现为近距离绿色消失)
            'baseline_dilate': ParameterValue(baseline_dilate, value_type=int),
            # 车体自身自过滤: 边长 1.0 m 的正方形内占据格视为车体并去除
            'self_filter_size': ParameterValue(
                self_filter_size, value_type=float),
            'xmin': ParameterValue(xmin, value_type=float),
            'ymin': ParameterValue(ymin, value_type=float),
            'xmax': ParameterValue(xmax, value_type=float),
            'ymax': ParameterValue(ymax, value_type=float),
            'output': output_file,
            'history_output': history_output,
            'history_duration': ParameterValue(history_duration, value_type=float),
            'history_max_points_per_frame': ParameterValue(history_max_points_per_frame, value_type=int),
            'history_min_hits': ParameterValue(history_min_hits, value_type=int),
            'render_period_ms': ParameterValue(render_period_ms, value_type=int),

            # 调试日志 / 异常快照: 逐帧把点云统计 (点数/质心/包围盒/自过滤数)
            # 写入 <debug_log_dir>/cloud_frames.csv; 异常帧 (点数突增/质心跳变/
            # 里程计跳变) 额外存 PCD 快照, 便于"雷达突然全是障碍"事后溯源。
            'debug_log_enabled': ParameterValue(
                debug_log_enabled, value_type=bool),
            'debug_log_dir': debug_log_dir,
            'debug_pcd_enabled': ParameterValue(
                debug_pcd_enabled, value_type=bool),
            'debug_pcd_max_files': ParameterValue(
                debug_pcd_max_files, value_type=int),
            'debug_cloud_count_jump_ratio': ParameterValue(
                debug_cloud_count_jump_ratio, value_type=float),
            'debug_cloud_count_jump_delta': ParameterValue(
                debug_cloud_count_jump_delta, value_type=int),
            'debug_centroid_jump_m': ParameterValue(
                debug_centroid_jump_m, value_type=float),
            'debug_odom_jump_m': ParameterValue(
                debug_odom_jump_m, value_type=float),
            'debug_odom_jump_yaw_deg': ParameterValue(
                debug_odom_jump_yaw_deg, value_type=float),
            # 位姿跳变"拒绝窗口"时长 (s): 检出跳变后该时长内的点云帧一律丢弃
            'pose_jump_reject_sec': ParameterValue(
                pose_jump_reject_sec, value_type=float),
            'debug_occ_ratio_jump': ParameterValue(
                debug_occ_ratio_jump, value_type=float),
            'pose_correction_topic': '/pose_correction',
            'pose_correction_global_period_sec': ParameterValue(
                LaunchConfiguration('pose_correction_global_period_sec'), value_type=float),
            'pose_correction_global_range_m': ParameterValue(
                LaunchConfiguration('pose_correction_global_range_m'), value_type=float),
            'pose_correction_local_range_m': ParameterValue(
                LaunchConfiguration('pose_correction_local_range_m'), value_type=float),
            'pose_correction_score_min': ParameterValue(
                LaunchConfiguration('pose_correction_score_min'), value_type=float),
            'pose_correction_max_delta_m': ParameterValue(
                LaunchConfiguration('pose_correction_max_delta_m'), value_type=float),
            'pose_correction_ema_alpha': ParameterValue(
                LaunchConfiguration('pose_correction_ema_alpha'), value_type=float),
            'pose_correction_score_gain': ParameterValue(
                LaunchConfiguration('pose_correction_score_gain'), value_type=float),
            'pose_correction_global_trigger_fails': ParameterValue(
                LaunchConfiguration('pose_correction_global_trigger_fails'),
                value_type=int),
            # 修正来源: false(默认) = 交由外部 3D 重定位节点 (relocalization_3d)
            #   承担 —— 本节点不估计/不发布 /pose_correction, 仅订阅并施加到点云;
            #   true = 使用本节点自带的 2D 栅格配准 (自己估计并发布)。
            'apply_pose_correction': ParameterValue(
                LaunchConfiguration('apply_pose_correction'), value_type=bool),

            # 动态障碍 (可变障碍): 网页"锁定历史累计图"置位
            # dynamic_obstacle_enabled 后生效; 点云供 Nav2 obstacle_layer
            # 订阅实现避障, 栅格图供网页画绿色层
            'dynamic_obstacle_topic': dynamic_obstacle_topic,
            'dynamic_obstacle_image_topic': dynamic_obstacle_image_topic,

            # 自过滤点云 (frame=base_link): 供 Nav2 obstacle_layer 的 pointcloud
            # 源开启 clearing, 从车体当前位置射出清除射线, 清掉旧红色残留格。
            'self_filtered_topic': self_filtered_topic,
            'self_filtered_publish_rate': ParameterValue(
                self_filtered_publish_rate, value_type=float),
            'self_filtered_max_points': ParameterValue(
                self_filtered_max_points, value_type=int),

            # 清除射线点云: 每帧向"只 clearing 不 marking"的 clear_ring 源发
            # 一圈 base_link 系稀疏点, 让 Nav2 从车体原点向四周射线清除旧红格,
            # 使红色代价区域实时跟随绿色障碍消失。话题名必须与 nav2_params.yaml
            # 里 clear_ring.topic 完全一致。
            'clear_ring_topic': clear_ring_topic,
            'clear_ring_points': ParameterValue(clear_ring_points, value_type=int),
            'clear_ring_radius': ParameterValue(clear_ring_radius, value_type=float),

            # 刷新历史累计图时整张清空 Nav2 红色代价区 (调 ClearEntireCostmap 服务),
            # 之后随新历史图从零累积。服务名须与 nav2_params.yaml 一致。
            'clear_costmap_on_history_reset': ParameterValue(
                clear_costmap_on_history_reset, value_type=bool),
            'clear_local_costmap_service': clear_local_costmap_service,
            'clear_global_costmap_service': clear_global_costmap_service,

            # 导航运动控制: 网页"开始运动"后按 Nav2 路径跟随,
            # 向 motion_topic 发布 8 字节运动帧 (control/shoot/tar_yaw/tar_pitch/vx/vy)
            'motion_topic': motion_topic,
            # 避障: 跟随 Nav2 局部路径 (/local_plan, 含动态避障);
            # 超过 local_plan_timeout 秒未更新则回退全局路径 /plan
            'odom_topic': odom_topic,
            'local_plan_topic': local_plan_topic,
            'local_plan_timeout': ParameterValue(local_plan_timeout, value_type=float),
            'motion_control': ParameterValue(motion_control, value_type=int),
            'motion_vx_scale': ParameterValue(motion_vx_scale, value_type=float),
            'motion_vy_scale': ParameterValue(motion_vy_scale, value_type=float),
            'motion_rate_hz': ParameterValue(motion_rate_hz, value_type=float),
            'motion_target_speed': ParameterValue(motion_target_speed, value_type=float),
            'motion_lookahead': ParameterValue(motion_lookahead, value_type=float),
            'motion_lookahead_speed_gain': ParameterValue(
                motion_lookahead_speed_gain, value_type=float),
            'motion_lookahead_max': ParameterValue(
                motion_lookahead_max, value_type=float),
            'motion_yaw_kp': ParameterValue(motion_yaw_kp, value_type=float),
            'motion_goal_tolerance': ParameterValue(motion_goal_tolerance, value_type=float),
            # 到达后位置校验: 车体中心 -> 用户目标点 距离超阈值则用自建 A* 重导航
            'motion_goal_check_tolerance': ParameterValue(
                motion_goal_check_tolerance, value_type=float),
            'motion_goal_check_max_retry': ParameterValue(
                motion_goal_check_max_retry, value_type=int),
            'motion_max_lateral': ParameterValue(motion_max_lateral, value_type=float),
            # 全局路径新鲜度: /plan 超过该时间未更新 -> 判定 Nav2 已停止重规划, 停车
            'motion_plan_timeout': ParameterValue(
                motion_plan_timeout, value_type=float),
            'motion_slow_radius': ParameterValue(motion_slow_radius, value_type=float),
            'motion_min_speed': ParameterValue(motion_min_speed, value_type=float),
            'motion_base_to_lidar_x': ParameterValue(motion_base_to_lidar_x, value_type=float),
            'motion_base_to_lidar_y': ParameterValue(motion_base_to_lidar_y, value_type=float),

            # 自建 A* 兜底规划器: /plan 不可用 (Nav2 因起点在障碍膨胀区内规划失败)
            # 时, 用节点内 A* 规划"当前位置 -> Web 目标点"的路径低速跟随;
            # A* 失败则停车并保持重试。
            'astar_enabled': ParameterValue(astar_enabled, value_type=bool),
            'astar_speed': ParameterValue(astar_speed, value_type=float),
            'astar_inflate_radius': ParameterValue(
                astar_inflate_radius, value_type=float),
            'astar_obstacle_penalty': ParameterValue(
                astar_obstacle_penalty, value_type=float),
            'astar_cost_reach': ParameterValue(
                astar_cost_reach, value_type=float),
            'astar_path_spacing': ParameterValue(
                astar_path_spacing, value_type=float),
            # A* 重算限频: 避免 100 Hz 重算阻塞运动线程 (0 = 每帧重算)
            'astar_replan_interval': ParameterValue(
                astar_replan_interval, value_type=float),
            'astar_replan_move_dist': ParameterValue(
                astar_replan_move_dist, value_type=float),
            'goal_pose_topic': goal_pose_topic,
            'astar_plan_topic': astar_plan_topic,

            # 占点 (hold point) 行为: 网页"选择占点位置"后车辆前往驻守并
            # 用车体阻挡靠近的障碍; 安全间距复用 astar_inflate_radius (0.50 m,
            # 与 Nav2 robot_radius 同口径)。
            'hold_point_enabled': ParameterValue(
                hold_point_enabled, value_type=bool),
            'hold_point_radius': ParameterValue(
                hold_point_radius, value_type=float),
            'hold_point_speed': ParameterValue(
                hold_point_speed, value_type=float),
            'hold_point_slow_radius': ParameterValue(
                hold_point_slow_radius, value_type=float),
            'hold_point_standoff_scale': ParameterValue(
                hold_point_standoff_scale, value_type=float),
            'hold_point_recenter_deadband': ParameterValue(
                hold_point_recenter_deadband, value_type=float),
            'hold_point_status_topic': hold_point_status_topic,
            'hold_point_viz_topic': hold_point_viz_topic,

            # SocketCAN 直发: 开启后运动帧除发布话题外, 还会原样 write() 到 CAN 总线
            # 接口需先拉起: sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
            # 用 candump -t d can0,0FF:7FF 即可看到本节点发出的 8 字节运动帧
            'can_enabled': ParameterValue(can_enabled, value_type=bool),
            'can_interface': can_interface,
            'can_id': ParameterValue(can_id, value_type=int),
            'can_send_period_ms': ParameterValue(can_send_period_ms, value_type=int),
        }])
    # 4) 实时 PGM 查看窗口（需要桌面环境; 无显示器时用 with_viewer:=false 关掉）
    # 注意: 下方 viewer 的参数默认值必须与 scripts/pgm_viewer.py 中
    #       declare_parameter 的默认值保持一致(单一数据源), 否则网页显示值
    #       与代码里看到的值会对不上。
    viewer = Node(
        package='test_navigation',
        executable='pgm_viewer.py',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'odom_topic': odom_topic,
            'target_node': '/test_navigation',
            # 3D 重定位节点 (锁定/解锁时同步 enabled, 与 reloc viewer 同机制)
            'target_node_3d': '/relocalization_3d',
            'robot_length': ParameterValue(robot_length, value_type=float),
            'robot_width': ParameterValue(robot_width, value_type=float),
            'lidar_to_base_x': ParameterValue(lidar_to_base_x, value_type=float),
            'lidar_to_base_y': ParameterValue(lidar_to_base_y, value_type=float),
            # 运动帧回显: 网页订阅该话题显示 control / vx / vy
            'motion_cmd_topic': motion_topic,
            'motion_vx_scale': ParameterValue(motion_vx_scale, value_type=float),
            'motion_vy_scale': ParameterValue(motion_vy_scale, value_type=float),
            # 动态障碍绿色层: 订阅 C++ 节点发布的动态障碍栅格图
            'dynamic_image_topic': dynamic_obstacle_image_topic,
            # 自建 A* 兜底路径 (黄色显示) 与目标点转发话题
            'astar_plan_topic': astar_plan_topic,
            'goal_pose_topic': goal_pose_topic,
            # 车体自过滤正方形边长: 网页据此画绿色自过滤框
            'self_filter_size': ParameterValue(
                self_filter_size, value_type=float),
            # 占点状态/可视化话题 (C++ 节点发布, 网页绘制占点圈/连线)
            'hold_point_status_topic': hold_point_status_topic,
            'hold_point_viz_topic': hold_point_viz_topic,
        }],
        condition=IfCondition(with_viewer))

    # 5) Nav2 桥接节点: 实时占据图 -> /map, /Odometry -> TF(map/odom/base_link)
    #    (始终启动, 不受 with_nav2 影响; Nav2 未启动时只是没人订阅 /map)
    nav2_bridge = Node(
        package='test_navigation',
        executable='nav2_bridge.py',
        output='screen',
        parameters=[{
            'odom_topic': odom_topic,
            'history_image_topic': '/realtime_pgm/image_history',
            'map_topic': '/map',
            'target_node': '/test_navigation',
            'pose_correction_topic': '/pose_correction',
            'lidar_to_base_x': ParameterValue(lidar_to_base_x, value_type=float),
            'lidar_to_base_y': ParameterValue(lidar_to_base_y, value_type=float),
        }])

    # 6) 3D 重定位节点 (test_relocalization 包, GICP scan-to-map):
    #    锁定历史累计图后接管位姿修正 —— 主节点 apply_pose_correction=false,
    #    仅订阅它发布的 /pose_correction 并施加到点云 (动态障碍/绿层/Nav2 点云
    #    全部随之一致); TF map->odom 仍由 nav2_bridge.py 统一广播, 故这里
    #    broadcast_tf=false, 避免两个 TF 源打架。
    #    先验地图: 网页"锁定"时自动导出到 export_dir 的 map_clouds.bin
    #    (RPGMCLD1, 与 relocalization_3d 的 prior_map_path 默认值一致),
    #    节点按 mtime 轮询自动重载; enabled 由网页锁定/解锁同步设置。
    relocalization_3d = Node(
        package='test_relocalization',
        executable='relocalization_3d',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('test_relocalization'),
                'config', 'relocalization.yaml']),
            {'broadcast_tf': ParameterValue(False, value_type=bool)},
        ],
        condition=IfCondition(with_relocalization_3d))

    # 7) 位姿诊断记录节点: 把 /Odometry 的位姿/姿态角与 /pose_correction 的
    #    修正量按同一时间基准写入 <debug_log_dir>/diag_frames.csv, 便于对
    #    "摇晃后位姿跳变/无法重定位"做事后对齐分析; 终端同时打印跳变 WARN。
    diag_logger = Node(
        package='test_navigation',
        executable='diag_logger.py',
        output='screen',
        parameters=[{
            'odom_topic': odom_topic,
            'pose_correction_topic': '/pose_correction',
            # 与 C++ 调试日志同目录, 所有诊断数据集中一处
            'log_dir': debug_log_dir,
            'log_file': diag_log_file,
            'odom_jump_m': ParameterValue(diag_odom_jump_m, value_type=float),
            'odom_jump_yaw_deg': ParameterValue(
                diag_odom_jump_yaw_deg, value_type=float),
        }],
        condition=IfCondition(with_diag))

    # 7) Nav2 导航栈 (controller/planner/behavior/bt_navigator/...):
    #    自定义行为树以 5 Hz (200 ms) 周期重规划, 配合实时代价地图实现动态避障。
    #    定位由 FAST-LIO /Odometry 提供, 故不启动 AMCL/map_server。
    #
    # 【行为树路径必须在这里改写成绝对路径】
    # Nav2 Humble 打开 bt_navigator.default_nav_to_pose_bt_xml 时只会:
    #   (a) 直接按该字符串打开文件(绝对路径, 或相对"进程工作目录"的路径);
    #   (b) 找不到时, 退回到 nav2_bt_navigator 包的 behavior_trees/ 目录里找。
    # 也就是说 yaml 里写 "test_navigation/config/xxx.xml" 这种"包名/相对
    # 路径"形式 Nav2 并不认: 它既不是绝对路径, 也不在 nav2_bt_navigator 的
    # behavior_trees 下 → 报
    #     "Couldn't open input XML file: test_navigation/config/xxx.xml"
    # → bt_navigator 激活失败(lifecycle_manager 报 Failed to change state) →
    # NavigateToPose 动作服务器不可用 → 网页点击目标后 planner_server 根本不会
    # 被调用 → /plan 一条消息都没有 → "没有规划出路径"。
    # 用 RewrittenYaml 在 launch 阶段把本包 config/ 下的绝对路径写进参数即可,
    # 换机器/换用户名依然有效(因此无需在 yaml 里硬编码绝对路径)。
    bt_xml_path = PathJoinSubstitution([
        FindPackageShare('test_navigation'), 'config',
        'navigate_to_pose_w_replanning_5hz.xml'])

    nav2_params_abs_bt = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml_path,
            'default_nav_through_poses_bt_xml': bt_xml_path,
        },
        convert_types=True)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch',
            'navigation_launch.py'])),
        launch_arguments={
            'params_file': nav2_params_abs_bt,
            'use_sim_time': 'false',
            'autostart': 'true'}.items(),
        condition=IfCondition(with_nav2))


    return LaunchDescription([
        # 链路开关
        DeclareLaunchArgument('with_driver', default_value='true'),
        DeclareLaunchArgument('config_file', default_value='avia.yaml'),

        # PGM 节点参数
        DeclareLaunchArgument('cloud_topic', default_value='/cloud_registered'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('zmin', default_value='-2.25'),
        DeclareLaunchArgument('zmax', default_value='0.22'),
        DeclareLaunchArgument('dilate', default_value='2'),
        # 静态基准图膨胀量 (-1 = 沿用 dilate)
        DeclareLaunchArgument('baseline_dilate', default_value='-1'),
        # 车体自身自过滤正方形边长 (m): 1.0 = 车体中心 1.0 m 见方内清除; <=0 关闭
        DeclareLaunchArgument('self_filter_size', default_value='1.0'),
        DeclareLaunchArgument('xmin', default_value='-20.0'),
        DeclareLaunchArgument('ymin', default_value='-20.0'),
        DeclareLaunchArgument('xmax', default_value='20.0'),
        DeclareLaunchArgument('ymax', default_value='20.0'),
        DeclareLaunchArgument('output', default_value='/tmp/realtime_map.pgm'),
        DeclareLaunchArgument('history_output', default_value='/tmp/realtime_map_history.pgm'),
        DeclareLaunchArgument('history_duration', default_value='300.0'),
        DeclareLaunchArgument('history_max_points_per_frame', default_value='5000'),
        DeclareLaunchArgument('history_min_hits', default_value='2'),
        DeclareLaunchArgument('render_period_ms', default_value='150'),

        # ---- 调试日志 / 异常快照 ----
        # 逐帧点云统计 CSV + 异常帧 PCD 快照; 便于"雷达突然全是障碍"事后溯源。
        DeclareLaunchArgument('debug_log_enabled', default_value='true'),
        # 日志目录: 默认在包安装目录下的 log/; 也可传绝对路径, 例如
        #   debug_log_dir:=/path/to/Navigation/test/test_navigation/log
        DeclareLaunchArgument(
            'debug_log_dir',
            default_value=PathJoinSubstitution([
                FindPackageShare('test_navigation'), 'log'])),
        DeclareLaunchArgument('debug_pcd_enabled', default_value='true'),
        # PCD 快照上限 (超过后不再保存, 避免磁盘写满)
        DeclareLaunchArgument('debug_pcd_max_files', default_value='200'),
        # 点数突增判定: 本帧 > 上帧 * ratio 且 差值 > delta
        DeclareLaunchArgument(
            'debug_cloud_count_jump_ratio', default_value='3.0'),
        DeclareLaunchArgument(
            'debug_cloud_count_jump_delta', default_value='20000'),
        # 点云质心跳变判定 (m)
        DeclareLaunchArgument('debug_centroid_jump_m', default_value='1.5'),
        # 里程计位姿跳变判定: 单帧位移 (m) / 偏航 (deg)
        DeclareLaunchArgument('debug_odom_jump_m', default_value='0.5'),
        DeclareLaunchArgument('debug_odom_jump_yaw_deg', default_value='30.0'),
        # 位姿跳变"拒绝窗口"时长 (s): 检出跳变后该时长内的点云帧被丢弃
        #   (不入历史缓冲/不发 Nav2), 防止错位点云形成假障碍; <=0 关闭
        DeclareLaunchArgument('pose_jump_reject_sec', default_value='0.8'),
        DeclareLaunchArgument('pose_correction_global_period_sec', default_value='1.0'),
        DeclareLaunchArgument('pose_correction_global_range_m', default_value='15.0'),
        DeclareLaunchArgument('pose_correction_local_range_m', default_value='1.0'),
        DeclareLaunchArgument('pose_correction_score_min', default_value='0.12'),
        # 单帧修正变化上限 (m): 现为"速率限制"而非硬拒绝 (每帧最多朝目标逼近该值)
        DeclareLaunchArgument(
            'pose_correction_max_delta_m', default_value='0.3'),
        # 时序滤波: EMA 平滑系数 (1=不平滑) 与滞回增益阈值 (新解须比当前解
        #   评分高出该值才接受, 否则保持不动)
        DeclareLaunchArgument(
            'pose_correction_ema_alpha', default_value='0.5'),
        DeclareLaunchArgument(
            'pose_correction_score_gain', default_value='0.05'),
        # 全局兜底搜索触发: 局部搜索连续失败达到该帧数才做 ±15 m/±180° 全搜
        DeclareLaunchArgument(
            'pose_correction_global_trigger_fails', default_value='5'),
        # 历史图占据比例单帧涨幅超该值 -> WARN
        DeclareLaunchArgument('debug_occ_ratio_jump', default_value='0.15'),

        # ---- 位姿诊断记录 (diag_logger.py) ----
        # 记录 /Odometry 位姿/姿态角 + /pose_correction 修正量 -> diag_frames.csv
        DeclareLaunchArgument('with_diag', default_value='true'),
        DeclareLaunchArgument('diag_log_file', default_value='diag_frames.csv'),
        # 终端 WARN 阈值: 里程计帧间位移 (m) / 偏航 (deg); <=0 关闭该项告警
        DeclareLaunchArgument('diag_odom_jump_m', default_value='0.30'),
        DeclareLaunchArgument('diag_odom_jump_yaw_deg', default_value='10.0'),

        # 动态障碍 (可变障碍): 网页锁定历史累计图后, 新出现的障碍单独成层
        # (绿色显示 + 不写入历史地图, 但点云仍供 Nav2 obstacle_layer 避障)
        DeclareLaunchArgument(
            'dynamic_obstacle_topic', default_value='/cloud_dynamic_obstacles'),
        DeclareLaunchArgument(
            'dynamic_obstacle_image_topic',
            default_value='/realtime_pgm/image_dynamic'),

        # 自过滤点云 (frame=base_link): nav2_params.yaml 的 pointcloud 源订阅该话题
        # 并【只开 clearing, marking=false】, 提供覆盖全 360° 的清除射线(全场清除
        # 主力)。话题名必须与 nav2_params.yaml 里 pointcloud.topic 完全一致。
        DeclareLaunchArgument(
            'self_filtered_topic', default_value='/cloud_self_filtered'),
        # 发布频率 (Hz): <=0 = 每收到一帧点云就发 (不限频)
        DeclareLaunchArgument(
            'self_filtered_publish_rate', default_value='20.0'),
        # 每帧最大点数 (等间隔抽稀): <=0 = 不抽稀
        DeclareLaunchArgument(
            'self_filtered_max_points', default_value='6000'),

        # 清除射线点云: 让红色代价区域实时跟随绿色障碍消失
        # (nav2_params.yaml 的 clear_ring 源订阅该话题, marking=false/clearing=true)
        DeclareLaunchArgument(
            'clear_ring_topic', default_value='/cloud_clear_ring'),
        # 环上点数 (每 360/N 度一个): <=0 关闭该机制
        DeclareLaunchArgument('clear_ring_points', default_value='72'),
        # 环半径 (m): 决定清除射线覆盖车周多远; <=0 关闭
        DeclareLaunchArgument('clear_ring_radius', default_value='1.2'),

        # 刷新历史累计图时, 同时整张清空 Nav2 局部/全局代价地图 (红区)。
        # 服务名与 nav2_params.yaml / 行为树里一致。
        DeclareLaunchArgument(
            'clear_costmap_on_history_reset', default_value='true'),
        DeclareLaunchArgument(
            'clear_local_costmap_service',
            default_value='/local_costmap/clear_entirely_local_costmap'),
        DeclareLaunchArgument(
            'clear_global_costmap_service',
            default_value='/global_costmap/clear_entirely_global_costmap'),

        DeclareLaunchArgument('image_topic', default_value='/realtime_pgm/image'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('with_viewer', default_value='true'),
        DeclareLaunchArgument('robot_length', default_value='0.60'),
        DeclareLaunchArgument('robot_width', default_value='0.45'),
        # 雷达->车体中心外参 (与 pgm_viewer.py 默认值一致; 由 Livox_Calibration.py 标定)
        DeclareLaunchArgument('lidar_to_base_x', default_value='0.239'),
        DeclareLaunchArgument('lidar_to_base_y', default_value='-0.158'),

        # 导航运动控制: 8 字节运动帧 (control/shoot/tar_yaw/tar_pitch/vx/vy)
        DeclareLaunchArgument('motion_topic', default_value='/nav_motion_cmd'),
        # 避障: Nav2 局部路径话题与新鲜度阈值
        DeclareLaunchArgument('local_plan_topic', default_value='/local_plan'),
        DeclareLaunchArgument('local_plan_timeout', default_value='0.5'),
        DeclareLaunchArgument('motion_control', default_value='1'),
        DeclareLaunchArgument('motion_vx_scale', default_value='200.0'),
        DeclareLaunchArgument('motion_vy_scale', default_value='200.0'),
        DeclareLaunchArgument('motion_rate_hz', default_value='100.0'),
        DeclareLaunchArgument('motion_target_speed', default_value='1.00'),
        DeclareLaunchArgument('motion_lookahead', default_value='0.45'),
        # 前视距离速度自适应: L = motion_lookahead + gain * V (0 = 关闭)
        DeclareLaunchArgument(
            'motion_lookahead_speed_gain', default_value='0.52'),
        # 前视距离上限 (m): <= 0 表示不限
        DeclareLaunchArgument('motion_lookahead_max', default_value='1.50'),
        DeclareLaunchArgument('motion_yaw_kp', default_value='1.20'),
        DeclareLaunchArgument('motion_goal_tolerance', default_value='0.25'),
        # 到达后位置校验阈值 (m): 车体中心 -> 用户目标点 距离超该值视为
        # "只到了 Nav2 路径末端但未到目标点", 用自建 A* 从当前位置重新导航;
        # 0 表示关闭该校验 (仅看路径末端距离)
        DeclareLaunchArgument(
            'motion_goal_check_tolerance', default_value='0.35'),
        # 位置校验失败时用自建 A* 重新导航的最大重试次数 (超限则彻底停车告警)
        DeclareLaunchArgument(
            'motion_goal_check_max_retry', default_value='2'),
        DeclareLaunchArgument('motion_max_lateral', default_value='0.0'),
        # 全局路径新鲜度上限 (s): /plan 超过该时间未更新即判定 Nav2 已停止
        # 重规划(导航结束/中止)并停车; 0 表示关闭该监督
        DeclareLaunchArgument('motion_plan_timeout', default_value='1.0'),
        # 减速起始距离 (m): <= 0 = 自动, 由 motion_target_speed 决定
        DeclareLaunchArgument('motion_slow_radius', default_value='0.0'),
        DeclareLaunchArgument('motion_min_speed', default_value='0.08'),
        # 车体中心 -> 雷达原点 (m): 与 lidar_to_base_x/y 互为反向量
        DeclareLaunchArgument(
            'motion_base_to_lidar_x', default_value='-0.239'),
        DeclareLaunchArgument(
            'motion_base_to_lidar_y', default_value='0.158'),

        # ---- 自建 A* 兜底规划器 ----
        # /plan 不可用 (Nav2 因起点落在障碍膨胀区内而规划失败/停止刷新) 时,
        # 用节点内 A* 规划 "当前位置 -> Web 目标点" 的路径, 以 astar_speed 低速
        # 跟随, 逃出障碍后继续前往目标; A* 失败则停车并保持重试;
        # Nav2 的 /plan 一恢复即切回跟随 Nav2。
        DeclareLaunchArgument('astar_enabled', default_value='true'),
        # A* 兜底路径跟随速度 (m/s): <=0 表示跟随 motion_target_speed (与正常巡航一致)
        DeclareLaunchArgument('astar_speed', default_value='-1.0'),
        # 硬性最小间距 (m, 车体旋转轴中心 -> 障碍): 与 nav2_params.yaml 的
        # robot_radius 统一口径 (均为 0.50)
        DeclareLaunchArgument('astar_inflate_radius', default_value='0.50'),
        # 靠障碍代价惩罚权重: 越大越倾向远离障碍的宽阔通路 (0 = 纯最短路)
        DeclareLaunchArgument('astar_obstacle_penalty', default_value='0.8'),
        # 软性避让范围 (m, 障碍邻近代价影响半径): 与 nav2_params.yaml 的
        # inflation_radius 统一 (均为 0.60)
        DeclareLaunchArgument('astar_cost_reach', default_value='0.60'),
        # A* 路径抽稀间距 (m): 0 = 不抽稀 (逐格保留)
        DeclareLaunchArgument('astar_path_spacing', default_value='0.15'),
        # A* 重算限频 (s): 距上次规划达该间隔即重算 (0 = 每帧重算, 有卡顿风险)
        DeclareLaunchArgument('astar_replan_interval', default_value='0.20'),
        # 车体位移达该距离即立刻重算 A* (m)
        DeclareLaunchArgument('astar_replan_move_dist', default_value='0.05'),
        # Web 目标点转发话题 (PoseStamped, map 系): 供 C++ 节点 A* 作为终点
        DeclareLaunchArgument('goal_pose_topic', default_value='/nav_goal_pose'),
        # 自建 A* 兜底路径话题 (Path, map 系): 供网页黄色显示
        DeclareLaunchArgument('astar_plan_topic', default_value='/astar_plan'),

        # ---- 占点 (hold point) 行为 ----
        # 是否默认开启占点 (网页"选择占点位置"按钮会实时覆盖该开关)
        DeclareLaunchArgument('hold_point_enabled', default_value='false'),
        # 占点范围 (m): 半径内的障碍才算威胁, 网页滑条可动态调整
        DeclareLaunchArgument('hold_point_radius', default_value='1.50'),
        # 占点机动速度 (m/s): <=0 表示跟随 motion_target_speed (与正常巡航一致)
        DeclareLaunchArgument('hold_point_speed', default_value='-1.0'),
        # 占点减速起始距离 (m): <0 自动 (取常规减速距离的一半, 更短更快),
        #   0 关闭减速, >0 手动指定 (须 > motion_goal_tolerance)
        DeclareLaunchArgument('hold_point_slow_radius', default_value='-1.0'),
        # 阻挡站位间距系数 (>=1, 越接近 1 越贴近敌人): 站位点距障碍 = k * 硬性间距
        DeclareLaunchArgument('hold_point_standoff_scale', default_value='1.10'),
        # 回位死区 (m): 距占点中心小于该值且无威胁时不再微调
        DeclareLaunchArgument(
            'hold_point_recenter_deadband', default_value='0.08'),
        # 占点状态/可视化话题 (供网页展示威胁与画圈)
        DeclareLaunchArgument(
            'hold_point_status_topic', default_value='/hold_point_status'),
        DeclareLaunchArgument(
            'hold_point_viz_topic', default_value='/hold_point_viz'),

        # SocketCAN 直发: 默认关闭, 需要时用 can_enabled:=true 打开
        DeclareLaunchArgument('can_enabled', default_value='false'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_id', default_value='255'),
        DeclareLaunchArgument('can_send_period_ms', default_value='10'),

        # Nav2: 关闭时仍发布 /map 与 TF, 但不启动规划/控制节点
        # (重规划 200 ms/次由 config/nav2_params.yaml 的 default_nav_to_pose_bt_xml
        #  指向自定义 BT 实现, 见该文件 bt_navigator 段)
        DeclareLaunchArgument('with_nav2', default_value='true'),
        DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('test_navigation'),
            'config', 'nav2_params.yaml'])),
        # 3D 重定位节点 (GICP): 锁定历史累计图后接管位姿修正
        DeclareLaunchArgument(
            'with_relocalization_3d', default_value='true',
            description='是否启动 test_relocalization 的 3D 重定位节点'),
        # 位姿修正来源: false(默认)=外部 3D 节点; true=主节点自带 2D 配准
        DeclareLaunchArgument(
            'apply_pose_correction', default_value='false',
            description='false=修正由 3D 重定位节点承担; true=主节点 2D 配准'),

        livox_driver,
        fastlio,
        pgm_node,
        viewer,
        nav2_bridge,
        relocalization_3d,
        diag_logger,
        nav2,
    ])

######lidar_to_base_x > 0：车体中心在雷达前方
######lidar_to_base_y > 0：车体中心在雷达左方