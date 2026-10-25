import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_map_yaml():
    """默认取工作空间根目录的 map.yaml（stl2pgm.py 的输出位置）"""
    pkg_share = get_package_share_directory('fishbot_description')
    # <ws>/install/fishbot_description/share/fishbot_description -> <ws>
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    candidate = os.path.join(ws_root, 'map.yaml')
    return candidate if os.path.isfile(candidate) else os.path.join(os.getcwd(), 'map.yaml')


def generate_launch_description():
    """带先验地图的仿真版本：Gazebo + 先验 /map + RViz

    与 run_fangzhen.sh（SLAM 在线建图）的区别：
      * 不启动 slam_toolbox，/map 由 map_server 加载的先验地图（PGM+YAML）提供
      * 额外发布 map→odom 的单位静态 TF 作为占位——没有 AMCL/SLAM 时
        RViz 的 Fixed Frame=map 需要这条 TF 才有完整链路
      * RViz 用 display_robot_model.rviz，Map 显示项已开启（话题 /map）

    用法：
        ros2 launch fishbot_description map_sim.launch.py
        ros2 launch fishbot_description map_sim.launch.py map_yaml:=/abs/path/map.yaml
        ros2 launch fishbot_description map_sim.launch.py rviz:=false
    """
    pkg_share = get_package_share_directory('fishbot_description')
    rviz_config = os.path.join(pkg_share, 'config', 'display_robot_model.rviz')

    declared_arguments = [
        DeclareLaunchArgument(
            'map_yaml', default_value=_default_map_yaml(),
            description='先验地图 yaml 路径（同目录需有同名 pgm）'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='是否启动 RViz'),
    ]

    # 1) Gazebo 仿真（机器人 + Mid-360 + ros2_control）
    action_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_sim_launch.py')
        )
    )

    # 2) 先验地图：map_server 发布 /map（lifecycle 节点）
    action_map_server = launch_ros.actions.LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',                   # LifecycleNode 必须显式给 namespace
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_yaml'),
            'topic_name': '/map',
            'use_sim_time': True,
        }]
    )

    # map_server 的状态切换用命令行做：launch 的 ChangeState 事件在本节点上会重复
    # 投递 activate，日志里留下 "Transition is not registered" 报错
    action_configure = TimerAction(period=3.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
            output='screen')
    ])
    # 有些版本 configure 后会自动 activate，这里先判断再补一刀，避免多余报错
    action_activate = TimerAction(period=8.0, actions=[
        ExecuteProcess(
            cmd=['bash', '-c',
                 "ros2 lifecycle get /map_server | grep -q '^active' "
                 "|| ros2 lifecycle set /map_server activate"],
            output='screen')
    ])

    # 3) 没有定位模块时的占位 TF：map→odom（odom→base_footprint 由控制器发布）
    #    odom 从 0 起步，而 Gazebo 里车出生在世界坐标 (-5, 3)、朝向 -1.5708，
    #    所以 map→odom 必须等于出生点位姿，RViz 里车才能和 Gazebo 对上
    action_static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_stub',
        arguments=['-5', '3', '0', '-1.5708', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # 4) RViz
    action_rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    action_rviz_delayed = TimerAction(period=10.0, actions=[action_rviz])

    return launch.LaunchDescription(declared_arguments + [
        action_gazebo,
        action_map_server,
        action_static_tf,
        action_configure,
        action_activate,
        action_rviz_delayed,
    ])
