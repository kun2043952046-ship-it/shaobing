# 重定位独立运行 launch:
#   ros2 launch test_relocalization relocalization.launch.py
# 不拉起 Livox/FAST-LIO (假设已在外部运行):
#   ros2 launch test_relocalization relocalization.launch.py with_driver:=false
# 关闭网页查看器:
#   ros2 launch test_relocalization relocalization.launch.py with_viewer:=false
#
# 注意: 本包与 test_draw_pgm_realtime 不要同时运行
#   (都会消费 /cloud_registered, 都发 /realtime_pgm/* 话题)。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 开关: 是否同时拉起 Livox 驱动 + FAST-LIO
    with_driver = LaunchConfiguration('with_driver')
    # 开关: 是否启动网页查看器 (端口 1727)
    with_viewer = LaunchConfiguration('with_viewer')
    # FAST-LIO 配置文件 (MID360 用 avia.yaml)
    config_file = LaunchConfiguration('config_file')

    declare_args = [
        DeclareLaunchArgument(
            'with_driver', default_value='true',
            description='是否同时拉起 Livox 驱动 + FAST-LIO'),
        DeclareLaunchArgument(
            'with_viewer', default_value='true',
            description='是否启动网页查看器 (端口 1727)'),
        DeclareLaunchArgument(
            'config_file', default_value='avia.yaml',
            description='FAST-LIO 配置文件 (MID360 直接用 avia.yaml)'),
    ]

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

    # 3) 重定位节点 (继承自主节点: 建图/历史缓冲/位姿修正)
    #    2D 配准整体让位给 3D 节点:
    #      - apply_pose_correction=false: 本节点不再自算/施加 2D 修正
    #        (只出原始点云, 省 CPU, 也避免错误的 2D 解把点云搬偏);
    #      - 修正量改发 /pose_correction_2d (仅供对比调试),
    #        /pose_correction 由 3D 节点独占发布。
    relocalization_node = Node(
        package='test_relocalization',
        executable='test_relocalization_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('test_relocalization'),
                'config', 'relocalization.yaml'),
            {
                'apply_pose_correction': False,
                'pose_correction_topic': '/pose_correction_2d',
            },
        ])

    # 4) 3D 重定位节点 (map_clouds.bin 先验地图 + GICP, 锁定后接管
    #    /pose_correction 与 map->odom TF)
    relocalization_3d = Node(
        package='test_relocalization',
        executable='relocalization_3d',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('test_relocalization'),
            'config', 'relocalization.yaml')])

    # 5) 网页查看器 (精简版: 地图/点云/位姿显示 + 参数热改, 无导航控制)
    viewer = Node(
        package='test_relocalization',
        executable='relocalization_viewer.py',
        output='screen',
        condition=IfCondition(with_viewer),
        parameters=[{
            'target_node': '/test_relocalization',
            'target_node_3d': '/relocalization_3d',
            'port': 1727,
        }])

    return LaunchDescription(declare_args + [
        livox_driver,
        fastlio,
        relocalization_node,
        relocalization_3d,
        viewer,
    ])
