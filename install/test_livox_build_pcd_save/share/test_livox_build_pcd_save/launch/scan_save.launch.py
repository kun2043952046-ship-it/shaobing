# 一键启动: Livox 驱动 + FAST-LIO + 采集保存节点
# 用法:
#   ros2 launch test_livox_build_pcd_save scan_save.launch.py \
#     scan_duration:=10.0 output_dir:=/home/lele/source_code_from_git/ws_livox/src/Navigation
#
# 注意: 下方 fast_lio / livox_ros_driver2 的 launch 路径以 Ericsii/FAST_LIO_ROS2
# 和官方 livox_ros_driver2 为准, 如果你用的是别的 fork, 按其 README 改这两处。
#一键执行
#cd /home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation
#source /opt/ros/humble/setup.bash
#source /home/lele/source_code_from_git/ws_livox/install/setup.bash   # 驱动 + fast_lio
#source install/setup.bash                                            # 你的采集节点

#ros2 launch test_livox_build_pcd_save scan_save.launch.py \
#  scan_duration:=10.0 \
#  output_dir:=/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scan_duration = LaunchConfiguration('scan_duration')
    output_dir = LaunchConfiguration('output_dir')
    config_file = LaunchConfiguration('config_file')

    # 1) Livox 驱动 (CustomMsg 格式, FAST-LIO 必须用 msg_ 系列 launch)
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('livox_ros_driver2'),
            'launch_ROS2', 'msg_MID360_launch.py'])))

    # 2) FAST-LIO 建图 (MID360 直接用 avia.yaml: lidar_type=1 即 Livox CustomMsg)
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'])),
        launch_arguments={'config_file': config_file}.items())

    # 3) 采集 10s 并保存 PCD 的节点
    collector = Node(
        package='test_livox_build_pcd_save',
        executable='test_livox_build_pcd_save_node',
        output='screen',
        parameters=[
            {'cloud_topic': '/cloud_registered'},
            {'scan_duration': ParameterValue(scan_duration, value_type=float)},
            {'output_dir': output_dir},
        ])

    # 采集节点保存完自动退出 -> 顺便把另外两个节点也关掉
    return LaunchDescription([
        DeclareLaunchArgument('scan_duration', default_value='10.0'),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/lele/source_code_from_git/ws_livox/src/Navigation'),
        DeclareLaunchArgument('config_file', default_value='avia.yaml'),
        livox_driver,
        fastlio,
        collector,
        RegisterEventHandler(OnProcessExit(
            target_action=collector,
            on_exit=[EmitEvent(event=Shutdown(reason='PCD saved'))])),
    ])
