import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取 fishbot_description 包的 share 目录，拼出 urdf 文件路径
    urdf_package_path = get_package_share_directory('fishbot_description')

    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'shaobing', 'shaobing.urdf.xacro')
    # default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'worlds', 'rm_field.world')
    # 世界文件里用 model://rm_field 引用场地模型，模型已随包携带到 models/ 下，
    # 通过 GAZEBO_MODEL_PATH 指向它，换机器/换工作空间后不需要手工拷到 ~/.gazebo/models
    models_package_path = os.path.join(urdf_package_path, 'models')
    # 本地搜索路径必须包含系统自带模型目录，否则 sun/ground_plane 之类解析失败
    # 后会转而去在线模型库下载，网络不通时 gzserver 就一直卡住
    system_models_path = '/usr/share/gazebo-11/models'
    user_models_path = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    action_set_gazebo_model_path = launch.actions.SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [models_package_path, ':', system_models_path, ':', user_models_path]
    )
    # 关闭在线模型库（models.gazebosim.org）：联网拉取会阻塞世界加载，
    # /spawn_entity 服务迟迟不出现，导致 spawn_entity 超时、控制器起不来
    action_set_gazebo_model_db = launch.actions.SetEnvironmentVariable(
        'GAZEBO_MODEL_DATABASE_URI', ''
    )
    # 声明一个 urdf 目录参数，方便修改要加载的模型文件
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description='加载的模型文件路径'
    )

    # 通过文件路径获取内容，并转化成参数值对象，将内容传递给 robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(
        ['xacro ', launch.substitutions.LaunchConfiguration('model')]
    )
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result, value_type=str
    )

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('world', default_gazebo_world_path), ('verbose', 'true')]
    )
    #action_rviz_node = launch_ros.actions.Node(
     #   executable='rviz2',
     #   arguments=['-d', default_rviz_config_path]
    #)

    # 若在 z=0 生成会落到世界地面(ground_plane)上、掉到地板下方，故抬高到 0.6m 自然下落到地板上
    # 出生点：地图/世界坐标 x=-5.0, y=3.0（rm_field 地图覆盖 x -6.21~5.99 / y -4.06~4.04，
    # 该点距最近障碍 0.95m，0.35m 半径车身有余量）
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'shaobing',
                   '-x', '-5.0', '-y', '3.0', '-z', '0.6',
                   '-Y', '-1.5708']  # 车头朝 -Y：绕 Z 轴转 -90°
    )
    # 加载并激活 ros2_control 控制器（spawner 会自动等待 controller_manager 就绪）
    action_spawner_jsb = launch_ros.actions.Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    action_spawner_diff = launch_ros.actions.Node(
        package='controller_manager', executable='spawner',
        arguments=['fishbot_diff_drive_controller'],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_spawn_entity,
        action_spawner_jsb,
        action_spawner_diff,
        #action_rviz_node
        action_set_gazebo_model_path,
        action_set_gazebo_model_db,
        action_launch_gazebo
    ])
