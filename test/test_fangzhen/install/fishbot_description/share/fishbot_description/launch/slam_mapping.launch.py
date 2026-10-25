import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """建图：点云转 2D 扫描（复用 lidar_2d.launch.py）+ slam_toolbox

    用法：
        ros2 launch fishbot_description slam_mapping.launch.py

    注意：建图（slam_toolbox）会发布 map→odom，若外部还有别的定位模块
    （AMCL 等）同时发布 map→odom 会相互冲突，两者不要同时启动。
    """
    pkg_share = get_package_share_directory('fishbot_description')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    # ============ 1) 3D 点云 → 2D 激光扫描 ============
    # 直接复用 lidar_2d.launch.py，保证建图与导航用的是同一份切片参数
    action_lidar_2d = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar_2d.launch.py')
        )
    )

    # ============ 2) slam_toolbox 在线异步建图 ============
    action_slam = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam_params_file': slam_params_file,
        }.items()
    )

    return launch.LaunchDescription([
        action_lidar_2d,
        action_slam,
    ])
