# 05 其他文件的分类（非 C++ 源码）

分析对象：`test/test_navigation/` 包内除 `src/test_navigation.cpp` 之外的全部文件。

## 1. 文件清单与归属

| 现有文件 | 类型 | 作用 | 建议归属 | 搬运/改造要点 |
|---|---|---|---|---|
| `package.xml` | 构建清单 | 包名、依赖声明（rclcpp/sensor_msgs/nav_msgs/std_msgs/nav2_msgs + rclpy/launch/opencv/numpy/nav2_bringup/tf2_ros/test_relocalization） | 保留在包根 | 若拆成多包，`nav2_msgs`/`tf2_ros` 跟着 `tasks/` 走；`rclpy`/`launch` 留在包级 |
| `CMakeLists.txt` | 构建脚本 | 当前只有 1 个可执行文件 + install(launch/config/log/scripts) + chmod 加固 | 保留在包根，按新结构改写 | 见本文第 3 节的推荐写法 |
| `LICENSE` | 许可 | Apache-2.0 文本 | 包根 | 原样保留 |
| `cmd.txt` | 运维备忘（**非代码**） | 编译/启动命令、CAN 拉起步骤、Nav2 说明，以及一条遗留问题记录（`base_link` 系点云时间戳早于 TF → 被 Message Filter 整帧丢弃） | `docs/ops_notes.md` | 里面的时间戳问题属于真实缺陷记录，建议转成 issue 条目不丢 |
| `launch/realtime_pgm.launch.py` | 启动编排 | 拉起 Livox 驱动 + FAST-LIO + 本节点 + `nav2_bridge.py` + Nav2（`with_driver/with_viewer/with_nav2` 开关，`RewrittenYaml` 注入行为树绝对路径） | `launch/`（配置/编排层） | 节点改名/拆包后，只改 `executable=` 与参数注入处 |
| `config/nav2_params.yaml` | Nav2 参数 | 全局/局部代价图、controller、planner、bt_navigator（`default_nav_to_pose_bt_xml` 指向本包 XML）、robot_radius 与 `astar_inflate_radius` 同口径 | `config/`（配置层） | 若包名/路径变化，`progress_checker`、`bt` 路径同步改 |
| `config/navigate_to_pose_w_replanning_5hz.xml` | Nav2 行为树 | 与官方 `navigate_to_pose_w_replanning_and_recovery.xml` 同构，仅把 `ComputePathToPose` 的 RateController 提到 5 Hz | `config/behavior_trees/` | 注意 Humble 不支持 `error_code_id` 端口（文件头已注明） |
| `scripts/nav2_bridge.py` | 桥接节点 | `/realtime_pgm/image_history`(mono8) → `/map`(OccupancyGrid, transient_local)；`/Odometry` → TF `odom→base_link`（叠加雷达外参）；`/pose_correction` → TF `map→odom`（按完整 SE(3)） | `scripts/`（工具/桥接层），或独立包 `nav_bridge` | 依赖 `GetParameters` 服务同步 xmin/ymin/resolution，节点改名后要同步服务名 |
| `scripts/pgm_viewer.py` | 网页可视化 | 订阅 `/realtime_pgm/image` + 历史图 + 点云预览 + 里程计 + 代价图 + 路径，网页交互（点选目标 → `NavigateToPose`、zmin/zmax 滑条、开始/停止运动） | `scripts/` 或 `web/` | 与 C++ 侧话题名强耦合，改话题要同步 |
| `scripts/diag_logger.py` | 诊断记录 | 每帧 `/Odometry` + `/pose_correction` 落 CSV（roll/pitch/yaw、dxy、dyaw、corr_age、异常标签） | `scripts/`（诊断工具） | 纯旁路，可独立运行 |
| `log/` | 运行时产物 | 逐帧点云统计 CSV、异常帧 PCD 快照（CMake 会安装一份占位，供 launch 的 `debug_log_dir` 默认值开箱即用） | 保留（运行期目录） | 建议改为安装到 `share/<pkg>/log` 的空目录，或用运行时 `$HOME/.ros/log` |
| `include/` | 空目录 | 目前没有任何头文件 | 拆分时作为头文件落点 | 见第 2 节建议 |
| `src/test_navigation.cpp` | 主实现 | 7937 行单类节点 | 拆到 `src/` `tasks/` `tools/` `config/` | 见 `01`~`04` 文档 |

## 2. 建议的新目录结构（含非 C++ 文件）

```
Navigation/
├── config/                          # 参数与数据类型（纯头文件/纯数据）
│   ├── constants.hpp  types.hpp  config.hpp  config_loader.*
│   ├── params.yaml                  # 从构造函数 104 个 declare_parameter 提炼的默认值
│   └── nav2/                        # 第三方栈参数
│       ├── nav2_params.yaml
│       └── behavior_trees/navigate_to_pose_w_replanning_5hz.xml
├── tools/                           # 纯算法库（无 ROS 依赖）
├── tasks/                           # 业务/行为类
├── src/                             # main + 节点装配 + 回调
├── launch/realtime_pgm.launch.py
├── scripts/                         # nav2_bridge.py / pgm_viewer.py / diag_logger.py
├── docs/split/                      # 本套拆分文档
├── docs/ops_notes.md                # cmd.txt 内容迁移目标
├── maps/                            # 地图产物（PGM/YAML/BIN/PCD）
├── log/                             # 运行期调试产物
├── package.xml
├── CMakeLists.txt
└── LICENSE
```

## 3. CMakeLists 推荐改写

当前 `CMakeLists.txt` 只有单一源文件；拆分后建议"两库一可执行"，让 `tools/` 能单独做单元测试：

```cmake
# ---- 纯算法库：不链接任何 ROS ----
add_library(nav_tools STATIC
  tools/tf_math.cpp
  tools/motion_codec.cpp
  tools/grid.cpp
  tools/grid_geom.cpp
  tools/grid_cluster.cpp
  tools/grid_clearance.cpp
  tools/astar.cpp
  tools/pure_pursuit.cpp
  tools/registration.cpp
  tools/io_pcd.cpp
  tools/io_bin.cpp
  tools/io_yaml.cpp
  tools/io_pgm.cpp
)
target_include_directories(nav_tools PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/tools
  ${CMAKE_CURRENT_SOURCE_DIR}/config
)

# ---- 业务库：需要 ROS 消息类型 ----
add_library(nav_tasks STATIC
  tasks/cloud_pipeline.cpp
  tasks/dynamic_split.cpp
  tasks/pose_correction.cpp
  tasks/cloud_publish.cpp
  tasks/map_render.cpp
  tasks/map_io.cpp
  tasks/astar_planner.cpp
  tasks/hold_point.cpp
  tasks/chase.cpp
  tasks/motion_control.cpp
  tasks/motion_frame.cpp
  tasks/nav2_service.cpp
  tasks/debug_log.cpp
)
target_link_libraries(nav_tasks PUBLIC nav_tools)
ament_target_dependencies(nav_tasks
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs nav2_msgs)

# ---- 可执行文件：只做装配 ----
add_executable(test_navigation_node
  src/main.cpp
  src/realtime_pgm_node.cpp
  src/callbacks.cpp
  src/parameter_callback.cpp
  src/can_socket.cpp
)
target_link_libraries(test_navigation_node PRIVATE nav_tasks)
ament_target_dependencies(test_navigation_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs nav2_msgs)
```

其余 `install(DIRECTORY launch config ...)`、`install(PROGRAMS scripts/...)`、chmod 加固段原样保留即可（把 `config` 换成新的 `config/`，`log` 保留）。

## 4. 可直接复用的验证命令（来自 `cmd.txt`）

```bash
cd /home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation
source /opt/ros/humble/setup.bash
source /home/lele/source_code_from_git/ws_livox/install/setup.bash
colcon build --packages-select test_navigation
source install/setup.bash
ros2 launch test_navigation realtime_pgm.launch.py

# 需要 CAN 直发时
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ros2 launch test_navigation realtime_pgm.launch.py can_enabled:=true
candump -t d can0,0FF:7FF     # 另一终端验证
```

## 5. 工作区顶层目录（搬运落点现状）

| 目录 | 现状 | 用途建议 |
|---|---|---|
| `Navigation/src/` | 只有 `Calibration/` | 搬 `main.cpp` + 节点装配 + 回调 |
| `Navigation/tasks/` | 空 | 搬 `03_tasks.md` 的 43 个条目 |
| `Navigation/tools/` | 空 | 搬 `02_tools.md` 的 31 个条目 + 二次抽取内核 |
| `Navigation/config/` | 只有 `map.yaml` | 搬 `01_config.md` 的常量/结构体/参数，`map.yaml` 归地图产物 |
| `Navigation/maps/`、`backups/`、`log/`、`build/`、`install/`、`test/` | 构建产物 / 备份 / 旧测试 | 拆分期间只读，不要动 |
