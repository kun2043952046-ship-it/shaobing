# 导航一键启动 (3D 重定位 + Nav2 + 实时栅格 + RViz)
#
#   ros2 launch fishbot_description navigation.launch.py \
#     init_x:=-5.0 init_y:=3.0 init_yaw:=-1.5708 init_yaw_hypotheses:=1
#
# TF 所有权:
#   map -> odom          : 只由 relocalization_3d 广播 (本 launch 内唯一)
#   odom -> base_footprint: gazebo diff_drive_controller
#   Nav2 侧只消费 TF, 不发布 map->odom
#
# 地图:
#   全局底图 /map          = nav2_map_server 加载 map.pgm (静态)
#   局部障碍 /cloud_registered = 实时点云 (odom 系), 进 local_costmap
#   参考栅格 /realtime_grid     = realtime_grid.py, 不参与规划
#
# 前提: Gazebo 已在跑 (gazebo_sim_launch.py), 或已有点云源。
#demo
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('fishbot_description')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    with_nav2 = LaunchConfiguration('with_nav2')
    rviz = LaunchConfiguration('rviz')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')
    init_yaw_hypotheses = LaunchConfiguration('init_yaw_hypotheses')
    prior_map_path = LaunchConfiguration('prior_map_path')
    map_yaml = LaunchConfiguration('map_yaml')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'with_nav2', default_value='true',
            description='是否启动 Nav2 (navigation_launch.py)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否打开 RViz (config/relocalization.rviz)'),
        # 默认 = Gazebo 出生点 (gazebo_sim_launch.py: -x -5.0 -y 3.0 -Y -1.5708)
        DeclareLaunchArgument(
            'init_x', default_value='-5.0',
            description='初始位姿 x (m), 默认=出生点'),
        DeclareLaunchArgument(
            'init_y', default_value='3.0',
            description='初始位姿 y (m), 默认=出生点'),
        DeclareLaunchArgument(
            'init_yaw', default_value='-1.5708',
            description='初始 yaw (rad), 默认=出生朝向 -90°'),
        DeclareLaunchArgument(
            'init_yaw_hypotheses', default_value='8',
            description='首次配准 yaw 假设数 (有可信 yaw 时设 1)'),
        DeclareLaunchArgument(
            'prior_map_path',
            default_value='/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/'
                          'test/test_fangzhen/map.pcd',
            description='3D 配准用先验点云 (PCD)'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/'
                          'test/test_fangzhen/map.yaml',
            description='Nav2 全局底图 (PGM 的 yaml)'),
    ]

    # 1) 3D 重定位 + 点云转换 + map_server (+ 可选 RViz)
    relocalization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'relocalization_3d.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'with_pgm_map': 'true',
            'map_yaml': map_yaml,
            'prior_map_path': prior_map_path,
            'init_x': init_x,
            'init_y': init_y,
            'init_yaw': init_yaw,
            'init_yaw_hypotheses': init_yaw_hypotheses,
        }.items())

    # 2) Nav2 (不含 map_server/amcl: 地图由上面提供, 定位由 3D 重定位提供)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'navigation_launch.py')),
        condition=IfCondition(with_nav2),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            # 注意: nav2 的 navigation_launch.py 用 PythonExpression('not ...')
            # 求值, 必须传 Python 字面量 True/False (小写 true/false 会 NameError)
            'autostart': 'True',
            'use_composition': 'False',
            'map_subscribe_transient_local': 'True',
        }.items())

    # 3) 实时 2D 栅格 (参考用)
    realtime_grid = Node(
        package='fishbot_description',
        executable='realtime_grid.py',
        name='realtime_grid',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'cloud_topic': '/cloud_registered',
            'output_topic': '/realtime_grid',
            'resolution': 0.05,
            'extent': 15.0,
        }])

    return LaunchDescription(declare_args + [relocalization, nav2, realtime_grid])
