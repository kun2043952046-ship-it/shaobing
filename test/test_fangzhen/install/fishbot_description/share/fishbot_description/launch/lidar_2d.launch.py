import launch
import launch_ros


def generate_launch_description():
    """Mid-360 的 3D 点云 → 2D LaserScan（/scan）

    下游的建图（slam_toolbox）和外部导航/定位都依赖这个节点，
    所以单独做成一个 launch，保证各处用的是同一份切片参数。

    用法：
        ros2 launch fishbot_description lidar_2d.launch.py      # 只转扫描（交给外部导航用）
        ros2 launch fishbot_description slam_mapping.launch.py  # 转扫描 + 建图
    """
    action_pc2ls = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
        parameters=[{
            # 必须用水平坐标系：livox_frame 上仰了 22.5°，用它做切片会把
            # 车后方的地面切进来，导致地图里车后一直出现"假墙"
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.01,
            # 高度为「离地高度」，取值由实测直方图确定：
            #   地面集中在 0.00~0.10（1.5万点），墙体分布在 0.10~0.50，0.50 以上为空
            #   → 下限 0.05（排除地面）、上限 0.50（墙体顶端）
            'min_height': 0.05,
            'max_height': 0.50,
            'angle_min': -3.14159265,
            'angle_max': 3.14159265,
            'angle_increment': 0.0087,   # 0.5°
            'scan_time': 0.2,            # 点云 5Hz
            'range_min': 0.50,           # 排除车身（车身最大半径 0.35m）
            'range_max': 20.0,
            'use_inf': True,
        }],
        output='screen'
    )

    return launch.LaunchDescription([action_pc2ls])
