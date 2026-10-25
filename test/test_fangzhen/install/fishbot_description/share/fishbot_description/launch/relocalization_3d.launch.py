# 3D 重定位节点 (仿真侧副本, 迁自 test_relocalization):
#   ros2 launch fishbot_description relocalization_3d.launch.py
#
# 默认拉起点云转换节点 cloud_to_odom.py:
#   /livox/lidar (livox_frame) --TF--> /cloud_registered (odom)
# 这样 GICP 解出的是 map->odom 修正量, 与实车 FAST-LIO 链路语义一致。
#
# 覆盖示例:
#   ros2 launch fishbot_description relocalization_3d.launch.py \
#     init_x:=-5.0 init_y:=3.0 init_yaw:=-1.5708 init_yaw_hypotheses:=1
#   已有外部 /cloud_registered (真实 FAST-LIO / 其它发布者) 时关掉转换:
#   ros2 launch fishbot_description relocalization_3d.launch.py with_cloud_converter:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_map_yaml():
    """工作空间根目录的 map.yaml (stl2pgm.py 产出, 与 map.pcd 同一坐标系)"""
    pkg_share = get_package_share_directory('fishbot_description')
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    candidate = os.path.join(ws_root, 'map.yaml')
    return candidate if os.path.isfile(candidate) else os.path.join(
        os.getcwd(), 'map.yaml')


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('fishbot_description'),
        'config', 'relocalization_3d.yaml')

    with_converter = LaunchConfiguration('with_cloud_converter')
    with_rviz = LaunchConfiguration('rviz')
    cloud_topic = LaunchConfiguration('cloud_topic')
    prior_map_path = LaunchConfiguration('prior_map_path')
    enabled = LaunchConfiguration('enabled')
    rate_hz = LaunchConfiguration('rate_hz')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')
    init_yaw_hypotheses = LaunchConfiguration('init_yaw_hypotheses')
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_pgm_map = LaunchConfiguration('with_pgm_map')
    map_yaml = LaunchConfiguration('map_yaml')

    declare_args = [
        DeclareLaunchArgument(
            'with_cloud_converter', default_value='true',
            description='是否拉起点云转换节点 (传感器系 -> odom 系)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否同时打开 RViz (config/relocalization.rviz)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='用 /clock 仿真时间。Gazebo 的 TF 是仿真时间戳, '
                        '混用墙钟会让 RViz 在 map 坐标系下查不到变换'),
        DeclareLaunchArgument(
            'with_pgm_map', default_value='true',
            description='是否用 map_server 发布 PGM 先验地图 (/map)'),
        DeclareLaunchArgument(
            'map_yaml', default_value=_default_map_yaml(),
            description='PGM 地图 yaml 路径 (同目录需有同名 pgm)'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/cloud_registered',
            description='送入配准的 odom 系点云话题'),
        DeclareLaunchArgument(
            'prior_map_path',
            default_value='/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/'
                          'test/test_fangzhen/map.pcd',
            description='先验点云地图 (PCD)'),
        DeclareLaunchArgument(
            'enabled', default_value='true',
            description='是否立即开始配准 (仿真里默认开)'),
        DeclareLaunchArgument(
            'rate_hz', default_value='1.0',
            description='配准频率 (Hz)'),
        # 默认值 = gazebo_sim_launch.py 的出生点 (-5.0, 3.0, -1.5708):
        #   odom 原点在 map 下的位姿 (= map->odom 初值)。若为 0, 会与真实出生点
        #   相差 5.8 m 导致配准失败, 机器人显示在先验点云中心 (map 原点)。
        DeclareLaunchArgument(
            'init_x', default_value='-5.0', description='初始位姿 x (m), 默认=出生点'),
        DeclareLaunchArgument(
            'init_y', default_value='3.0', description='初始位姿 y (m), 默认=出生点'),
        DeclareLaunchArgument(
            'init_yaw', default_value='-1.5708',
            description='初始 yaw (rad), 默认=出生朝向 -90°'),
        DeclareLaunchArgument(
            'init_yaw_hypotheses', default_value='8',
            description='首次配准尝试的 yaw 假设个数 (8=每 45°; 有可信 yaw 时设 1)'),
    ]

    # 仿真点云转换: /livox/lidar -> /cloud_registered (odom 系)
    cloud_converter = Node(
        package='fishbot_description',
        executable='cloud_to_odom.py',
        name='cloud_to_odom',
        output='screen',
        condition=IfCondition(with_converter),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'input_topic': '/livox/lidar',
            'output_topic': cloud_topic,
            'target_frame': 'odom',
        }])

    relocalization_3d = Node(
        package='fishbot_description',
        executable='relocalization_3d',
        name='relocalization_3d',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'cloud_topic': cloud_topic,
                'prior_map_path': prior_map_path,
                'enabled': ParameterValue(enabled, value_type=bool),
                'rate_hz': ParameterValue(rate_hz, value_type=float),
                'init_x': ParameterValue(init_x, value_type=float),
                'init_y': ParameterValue(init_y, value_type=float),
                'init_yaw': ParameterValue(init_yaw, value_type=float),
                'init_yaw_hypotheses': ParameterValue(
                    init_yaw_hypotheses, value_type=int),
            },
        ])

    # RViz: 固定坐标系 map, 已配好先验图 / 实时点云 / 位姿三个显示
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_relocalization',
        output='screen',
        condition=IfCondition(with_rviz),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        arguments=['-d', os.path.join(
            get_package_share_directory('fishbot_description'),
            'config', 'relocalization.rviz')])

    # PGM 先验地图 -> /map (nav_msgs/OccupancyGrid), 供 RViz 的 Map 显示叠加
    #   注意: 这里只发 /map, 不发 map->odom (那条由 relocalization_3d 提供);
    #   与 map_sim.launch.py 的 static map->odom 占位 TF 不能混用
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        condition=IfCondition(with_pgm_map),
        parameters=[{
            'yaml_filename': map_yaml,
            'topic_name': '/map',
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }])

    map_configure = TimerAction(period=3.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
            output='screen')])

    map_activate = TimerAction(period=8.0, actions=[
        ExecuteProcess(
            cmd=['bash', '-c',
                 "ros2 lifecycle get /map_server | grep -q '^active' "
                 "|| ros2 lifecycle set /map_server activate"],
            output='screen')])

    return LaunchDescription(
        declare_args +
        [cloud_converter, relocalization_3d, map_server, map_configure,
         map_activate, rviz])
