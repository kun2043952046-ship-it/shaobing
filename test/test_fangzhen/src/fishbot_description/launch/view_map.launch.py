import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """单独查看 RViz（不起仿真、不建图）时用：RViz + 占位静态 TF

    背景：display_robot_model.rviz 的 Fixed Frame 已改为 map，单独开 RViz 时
    没有 map 坐标系，Global Status 会报 "Fixed Frame [map] does not exist"，
    所有显示项都不渲染。这里发一个单位静态 TF map→base_footprint 占位，
    让 Grid / RobotModel / Map 都能正常显示。

    用法：
        ros2 launch fishbot_description view_map.launch.py

    注意：仿真 + 建图时不要用这个 launch —— slam_toolbox 会发布 map→odom，
    odom→base_footprint 由控制器发布，再叠加这里的 map→base_footprint 会让
    base_footprint 有两个父坐标系，TF 树冲突。仿真请用 run_fangzhen.sh 或
    gazebo_sim_launch.py + slam_mapping.launch.py。
    """
    pkg_share = get_package_share_directory('fishbot_description')
    rviz_config = os.path.join(pkg_share, 'config', 'display_robot_model.rviz')

    action_static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_footprint_stub',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
        output='screen'
    )

    action_rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return launch.LaunchDescription([
        action_static_tf,
        action_rviz,
    ])
