import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    nav_pkg_share = get_package_share_directory("r2n_bringup_pkg")
    map_trans_pkg_share = get_package_share_directory("map_trans_pkg")
    fast_lio_pkg_dir = get_package_share_directory("fast_lio")
    livox_driver_pkg_dir = get_package_share_directory("livox_ros_driver2")

    # Map file paths (centralized here)
    pcd_file_path = os.path.join(nav_pkg_share, "pcds", "test.pcd")      # PCD for relocalization
    map_yaml_file = os.path.join(nav_pkg_share, "maps", "test.yaml")     # YAML + PGM for Nav2
    nav2_params_file = os.path.join(nav_pkg_share, "params", "nav2_params_test.yaml")
    rviz_config_path = os.path.join(nav_pkg_share, "rviz", "nav2.rviz")

    # 车体中心相对雷达的偏移(m): x 向前为正, y 向左为正
    # 雷达在车体正中时均为 0; 由 lidar_to_base 标定得到
    lidar_to_base_x = LaunchConfiguration("lidar_to_base_x")
    lidar_to_base_y = LaunchConfiguration("lidar_to_base_y")

    return LaunchDescription(
        [
            # === 标定参数: 车体中心相对雷达的偏移(m) ===
            # x 向前为正, y 向左为正; 由 lidar_base_check.py 在本车实测标定
            # (两次独立反推 (0.1775,-0.2071) / (0.1752,-0.1957), 取均值)
            DeclareLaunchArgument("lidar_to_base_x", default_value="0.176"),
            DeclareLaunchArgument("lidar_to_base_y", default_value="-0.20"),

            # === Livox Mid-360 Driver ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(livox_driver_pkg_dir, "launch", "msg_MID360_launch.py")
                )
            ),

            # === Fast-LIO Mapping ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fast_lio_pkg_dir, "launch", "mapping.launch.py")
                )
            ),

            # === Relocalization Node (directly launched, with PCD path) ===
            Node(
                package="map_trans_pkg",
                executable="relocalization_node",
                name="relocalization_node",
                output="screen",
                parameters=[
                    {
                        "global_map_path": pcd_file_path,   # PCD map for relocalization
                        "map_voxel_size": 0.3,
                        "scan_voxel_size": 0.1,
                        # 多帧累积窗口: /cloud_registered ≈ 10Hz, 10 帧 ≈ 1s, 与 1Hz 匹配周期对齐
                        "accum_frames": 10,
                        "localization_freq": 1.0,
                        "localization_threshold": 0.4,
                        "fov": 6.28318530718,  # 2π
                        "fov_far": 30.0,
                        "tf_pub_freq": 50.0,
                        # ==== 首次锁定: 需连续 init_confirm_frames 帧结果互相一致才采纳 ====
                        # 与 localization_threshold 不同, 该判据不依赖 fitness 绝对值,
                        # 所以阈值降到 0.4 后也不会被单帧误匹配带偏初始位姿。
                        "init_confirm_frames": 3,        # 连续一致帧数(1Hz 下 ≈ 秒)
                        "init_consistency_pos": 0.15,    # 帧间平移一致阈值(m)
                        "init_consistency_yaw": 0.10,    # 帧间 yaw 一致阈值(rad)
                        # ==== 全局粗配准(FPFH + RANSAC): 未锁定时使用, 不需要初值 ====
                        # 用于实现无人干预的自动全局定位(车可停在场地任意位置)
                        "global_reg_voxel": 0.5,        # 降采样体素(m), 越大越快但越粗
                        "global_reg_max_iter": 100000,  # RANSAC 最大迭代次数
                        "global_reg_period": 5.0,       # 未锁定时最小调用间隔(s)
                        "global_reg_threshold": 0.30,   # 可接受的最小内点率
                    }
                ],
            ),

            # === Nav2 Bringup (localization only, no SLAM) ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        # 注意 启动的是navigationlaunch 不是bring_uplaunch 所以不包含acml
                        PathJoinSubstitution(
                            [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "autostart": "true",
                    "params_file": nav2_params_file,
                    # "map": map_yaml_file, 没有这个参数
                    # "slam": "false", # 没有这个参数
                    # "localization": "false", #没有 这个参数     
                    "use_lifecycle_manager": "false",
                }.items(),
            ),

            # === PointCloud to LaserScan (for Nav2 costmap) ===
            Node(
                package="map_trans_pkg",
                executable="pointcloud2_to_laserscan",
                name="pointcloud2_to_laserscan",
                output="screen",
                parameters=[
                    {
                        "pc_topic": "/cloud_registered_body",
                        "laserscan_topic": "/scan",
                        "output_frame": "body_2d",
                        "min_height": -0.2,
                        "max_height": 0.2,
                        "min_dist": 0.1,
                        "max_dist": 10.0,
                        "angle_resolution_deg": 1.0,
                        "angle_min_deg": -180.0,
                        "angle_max_deg": 180.0,
                        "filter_mean_k": 5,
                        "filter_stddev": 1.0,
                        "grid_resolution": 0.05,
                    }
                ],
            ),

            # === Static Map Publisher for Nav2 ===
            Node(
                package="map_trans_pkg",
                executable="map_publisher_node",
                name="map_for_nav2",
                output="screen",
                parameters=[
                    {
                        "map_path": map_yaml_file,      # Same YAML used by Nav2
                        "publish_topic": "/map",
                        "frame_id": "map",
                        "publish_interval": 10.0,
                    }
                ],
            ),
            # # 1. Serial CAN bridge node (connects ROS ↔ serial port)
            # Node(
            #     package="can_serial_pkg",          # Replace with your actual package name
            #     executable="can_serial_node",
            #     name="serial_can_node",
            #     output="screen",
            #     parameters=[
            #         {
            #             "serial_port": "/dev/ttyACM0",   # Adjust if needed
            #             "baud_rate": 115200,
            #             "can_downward_topic": "/msg_to_can",
            #             "can_upward_topic": "/msg_from_can",
            #         }
            #     ],
            # ),

            # # 2. Middle coding node: /cmd_vel → encoded CAN frame → /msg_to_can
            # Node(
            #     package="can_serial_pkg",          # Same package as above
            #     executable="middle_coding_node",
            #     name="middle_coding_node",
            #     output="screen",
            #     # No parameters needed — topics are hardcoded in the node
            # ),
            # === 车体中心 TF: body_2d(雷达) -> base_link(车体中心) ===
            # 把雷达坐标系平移到车体几何中心; Nav2 的 robot_base_frame 用 base_link,
            # 使 footprint / 定位 / 控制都以车体中心为参考。
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body2d_to_base_link",
                output="screen",
                arguments=[
                    "--x", lidar_to_base_x,
                    "--y", lidar_to_base_y,
                    "--z", "0.0",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "body_2d",
                    "--child-frame-id", "base_link",
                ],
            ),

            # === RViz ===
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )

# nav使用shan ge地图进行 规划
# 通过map_yaml_file 即maps/test.yaml 生成2D全局地图

# pcd 需要加载先验PCD test.pcd用于重定位
# 重定位 使用先验pcd和现在的pcd进行对比 确定机器人当前在地图的位置 map → odom

# 随时扫描的/scan 转为pcd后用于代价地图 costmap给nav2用

# 用途	文件	谁在用	作用
# 3D点云地图	test.pcd	relocalization_node	用来定位
# 2D栅格地图	test.yaml + test.pgm	Nav2 + map_publisher	用来规划
# 实时雷达点云	Livox MID360	Fast-LIO + costmap	用来避障