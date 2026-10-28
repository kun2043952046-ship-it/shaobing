# 06 其他文件与构建归属（非 C++ 核心源码）

约定：`config/` **只放 yaml**（调参），`tools/` 纯计算，`io/` 硬件与外部交互，`tasks/` 业务流程，`src/` 装配。

## 1. 文件清单与归属

| 现有文件 | 类型 | 作用 | 新归属 | 改动要点 |
|---|---|---|---|---|
| `package.xml` | 构建清单 | 依赖声明（rclcpp / sensor_msgs / nav_msgs / std_msgs / nav2_msgs / rclpy / launch / nav2_bringup / tf2_ros / test_relocalization） | 包根 | `nav2_msgs`、`rclcpp` 跟 `io/`、`tasks/` 走；`rclpy`、`launch` 留包级 |
| `CMakeLists.txt` | 构建脚本 | 现在只有 1 个可执行 + install(launch/config/log/scripts) | 包根 | 改为"三库一可执行"，见第 3 节 |
| `LICENSE` | 许可 | Apache-2.0 | 包根 | 原样 |
| `cmd.txt` | 运维备忘（非代码） | 编译/启动命令、CAN 拉起、Nav2 说明 + 一条遗留缺陷记录（`base_link` 系点云戳早于 TF → Message Filter 整帧丢弃） | `docs/ops_notes.md` | 缺陷记录转 issue，别丢；它正是 `io/ros_stamp.cpp` 要解决的问题 |
| `launch/realtime_pgm.launch.py` | 启动编排 | 拉起 Livox + FAST-LIO + 本节点 + `nav2_bridge.py` + Nav2，`RewrittenYaml` 注入行为树绝对路径 | `launch/` | 节点改名/拆包后改 `executable=` 与参数注入 |
| `config/nav2_params.yaml` | Nav2 参数（**yaml，留在 config**） | 全局/局部代价图、controller、planner、bt_navigator、`robot_radius`（与 `astar_inflate_radius` 同口径） | `config/nav2_params.yaml` | bt 路径改为新位置 |
| `config/navigate_to_pose_w_replanning_5hz.xml` | Nav2 行为树（**不是 yaml**） | 与官方同构，仅 `ComputePathToPose` 提到 5 Hz | `launch/behavior_trees/` | Humble 不支持 `error_code_id` 端口（文件头已注明） |
| `scripts/nav2_bridge.py` | 桥接节点 | `image_history`(mono8) → `/map`(transient_local)；`/Odometry` → TF `odom→base_link`；`/pose_correction` → TF `map→odom`（完整 SE(3)） | `scripts/`（或独立包 `nav_bridge`） | 依赖 `GetParameters` 同步 xmin/ymin/resolution，节点改名要同步 |
| `scripts/pgm_viewer.py` | 网页可视化 | 订阅 image / history / 点云预览 / odom / 代价图 / 路径，网页点选目标 → `NavigateToPose`、zmin/zmax 滑条、开始/停止运动 | `scripts/` 或 `web/` | 话题名与 C++ 侧强耦合 |
| `scripts/diag_logger.py` | 诊断记录 | 每帧 odom + pose_correction 落 CSV（roll/pitch/yaw、dxy、dyaw、corr_age、异常标签） | `scripts/` | 纯旁路，可独立跑 |
| `log/` | 运行期产物 | 逐帧统计 CSV、异常帧 PCD 占位（供 launch 的 `debug_log_dir` 默认值） | 保留（运行期目录） | 由 `io/debug_log_file` 创建，注意先 `create_directory` 再开文件 |
| `include/` | 空目录 | 无头文件 | 可删 | 头文件放各自层（`tools/`、`io/`、`tasks/`、`src/`）内 |
| `src/test_navigation.cpp` | 主实现 | 7937 行单类节点 | 拆到五层 | 见 `01`~`05` 文档 |

## 2. 建议的最终目录树

```
Navigation/
├── config/                              # 只有 yaml
│   ├── params.yaml                      122 个可调参数（键见 01_config.md）
│   └── nav2_params.yaml
├── tools/                               # 纯计算（无 ROS）
│   ├── types.hpp  constants.hpp
│   ├── tf_math.*  ros_utils.*  registration.*  motion_codec.*
│   ├── grid.*  grid_geom.*  grid_cluster.*  grid_clearance.*
│   └── astar.*  pure_pursuit.*          （二次抽取后新增）
├── io/                                  # 硬件与外部交互
│   ├── timed_cloud.hpp
│   ├── can_bus.*  ros_cloud_pub.*  ros_path_pub.*  ros_motion_pub.*
│   ├── ros_pose_pub.*  ros_image_pub.*  ros_stamp.*  ros_nav2_client.*
│   ├── file_pcd.*  file_bin.*  file_yaml.*  file_pgm.*
│   └── debug_log_file.*
├── tasks/                               # 业务流程
│   ├── cloud_pipeline.*  dynamic_split.*  pose_correction.*
│   ├── map_render.*  map_io.*  astar_planner.*
│   ├── hold_point.*  chase.*  motion_control.*  debug_log.*
├── src/                                 # 装配与回调
│   ├── main.cpp
│   ├── realtime_pgm_node.hpp / .cpp
│   ├── callbacks.cpp
│   └── parameter_callback.cpp
├── launch/
│   ├── realtime_pgm.launch.py
│   └── behavior_trees/navigate_to_pose_w_replanning_5hz.xml
├── scripts/                             # nav2_bridge.py / pgm_viewer.py / diag_logger.py
├── docs/
│   ├── split/                           本套拆分文档
│   └── ops_notes.md                     cmd.txt 迁移目标
├── maps/  log/                          地图产物 / 运行期调试产物
├── package.xml
├── CMakeLists.txt
└── LICENSE
```

## 3. CMakeLists 推荐（三库一可执行）

```cmake
# ---------- tools：纯计算，不链接 ROS ----------
add_library(nav_tools STATIC
  tools/tf_math.cpp
  tools/registration.cpp
  tools/grid.cpp
  tools/grid_geom.cpp
  tools/grid_cluster.cpp
  tools/grid_clearance.cpp
  tools/astar.cpp
  tools/pure_pursuit.cpp
)
target_include_directories(nav_tools PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/tools
)
# 只允许 Eigen/标准库, 不允许 rclcpp

# ---------- io：硬件与外部交互 ----------
add_library(nav_io STATIC
  io/can_bus.cpp
  io/ros_cloud_pub.cpp
  io/ros_path_pub.cpp
  io/ros_motion_pub.cpp
  io/ros_pose_pub.cpp
  io/ros_image_pub.cpp
  io/ros_stamp.cpp
  io/ros_nav2_client.cpp
  io/file_pcd.cpp
  io/file_bin.cpp
  io/file_yaml.cpp
  io/file_pgm.cpp
  io/debug_log_file.cpp
)
target_link_libraries(nav_io PUBLIC nav_tools)
ament_target_dependencies(nav_io
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs nav2_msgs)

# ---------- tasks：业务流程 ----------
add_library(nav_tasks STATIC
  tasks/cloud_pipeline.cpp
  tasks/dynamic_split.cpp
  tasks/pose_correction.cpp
  tasks/map_render.cpp
  tasks/map_io.cpp
  tasks/astar_planner.cpp
  tasks/hold_point.cpp
  tasks/chase.cpp
  tasks/motion_control.cpp
  tasks/debug_log.cpp
)
target_link_libraries(nav_tasks PUBLIC nav_io)

# ---------- src：装配 ----------
add_executable(test_navigation_node
  src/main.cpp
  src/realtime_pgm_node.cpp
  src/callbacks.cpp
  src/parameter_callback.cpp
)
target_link_libraries(test_navigation_node PRIVATE nav_tasks)
ament_target_dependencies(test_navigation_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs nav2_msgs)
```

原有 `install(DIRECTORY launch config log ...)`、`install(PROGRAMS scripts/...)`、chmod 加固段保持不变（`config` 现在只含两个 yaml）。

## 4. 参数外置后 launch 的写法

`config/params.yaml` 落地后，launch 里用参数文件代替一堆 `declare_parameter` 内联默认值：

```python
Node(
    package='test_navigation',
    executable='test_navigation_node',
    parameters=[PathJoinSubstitution([FindPackageShare('test_navigation'), 'config', 'params.yaml'])],
    ...
)
```

行为树 XML 继续用 `RewrittenYaml` 注入绝对路径（与 `nav2_bringup` 处理 map 的方式一致）。

## 5. 验证命令（来自 `cmd.txt`）

```bash
cd /home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation
source /opt/ros/humble/setup.bash
source /home/lele/source_code_from_git/ws_livox/install/setup.bash
colcon build --packages-select test_navigation
source install/setup.bash
ros2 launch test_navigation realtime_pgm.launch.py

# CAN 直发
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ros2 launch test_navigation realtime_pgm.launch.py can_enabled:=true
candump -t d can0,0FF:7FF
```

## 6. 工作区顶层目录现状（搬运落点）

| 目录 | 现状 | 放什么 |
|---|---|---|
| `Navigation/config/` | 只有 `map.yaml`（地图产物） | 只放 `params.yaml` / `nav2_params.yaml`；`map.yaml` 挪到 `maps/` |
| `Navigation/tools/` | 空 | `02_tools.md` 的 35 个条目 + 二次抽取内核 |
| `Navigation/io/` | **待新建** | `03_io.md` 的 24 个条目 |
| `Navigation/tasks/` | 空 | `04_tasks.md` 的 22 个条目 |
| `Navigation/src/` | 只有 `Calibration/` | `05_node.md` 的 20 个条目 |
| `Navigation/maps/`、`backups/`、`log/`、`build/`、`install/`、`test/` | 构建产物/备份/旧测试 | 拆分期间只读 |
