#!/usr/bin/env bash
# ============================================================
#  带先验地图的仿真一键启动：Gazebo + 先验 /map + RViz
#  （不跑 slam_toolbox，地图来自 map.pgm/map.yaml）
#
#  用法：
#    ./run_fangzhen_map.sh                              # 用工作空间根目录 map.yaml
#    ./run_fangzhen_map.sh /abs/path/other.yaml         # 指定其它先验地图
#    MAP_YAML=/abs/path/other.yaml ./run_fangzhen_map.sh
#
#  Ctrl+C 一次性关闭所有节点（由 ros2 launch 负责）
#
#  说明：
#    - 脚本自动 cd 到本文件所在工作空间并 source install/setup.bash
#    - ROS 2 Humble 已写进 ~/.bashrc 时无需额外操作；否则自动 source /opt/ros/humble
#    - 首次使用或改了 urdf/launch/config 后，先执行 colcon build
#    - 在线建图请用 run_fangzhen.sh（那个跑 slam_toolbox）
# ============================================================
# 注意：ROS 的 setup.bash 里会引用未定义变量（如 COLCON_TRACE），故不能开 set -u
set +u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR" || exit 1

# ---------- 1) 环境准备 ----------
if ! command -v ros2 >/dev/null 2>&1; then
  if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  else
    echo "[ERROR] 未找到 ros2，也没有 /opt/ros/humble/setup.bash" >&2
    exit 1
  fi
fi

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "[ERROR] 工作空间尚未编译：$WS_DIR/install/setup.bash 不存在" >&2
  echo "        先执行：cd $WS_DIR && colcon build" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

# ---------- 1.5) 依赖检查（缺包只提示，不中断）----------
for pkg in gazebo_ros gazebo_ros2_control controller_manager \
           nav2_map_server rviz2; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[WARN] 缺少依赖包 $pkg，安装：sudo apt install ros-humble-${pkg//_/-}" >&2
  fi
done

# ---------- 2) 先验地图路径 ----------
# 优先级：命令行参数 > 环境变量 MAP_YAML > 工作空间根目录 map.yaml
MAP_YAML="${1:-${MAP_YAML:-$WS_DIR/map.yaml}}"
if [ ! -f "$MAP_YAML" ]; then
  echo "[ERROR] 先验地图不存在：$MAP_YAML" >&2
  echo "        先用地形 STL 生成一张：" >&2
  echo "        python3 src/fishbot_description/scripts/stl2pgm.py \\" >&2
  echo "          src/fishbot_description/models/rm_field/meshes/sk_rmmap.stl -o map" >&2
  exit 1
fi

echo "[INFO] 使用先验地图：$MAP_YAML"
echo "[INFO] 启动 Gazebo + map_server(先验 /map) + RViz ..."
echo "[INFO] 键盘遥控（另开终端）：ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "[INFO] Ctrl+C 关闭所有节点"

# ---------- 3) 启动 ----------
# 交给 ros2 launch 管理子进程，Ctrl+C 时它会按顺序关掉全部节点
exec ros2 launch fishbot_description map_sim.launch.py map_yaml:="$MAP_YAML" "${@:2}"
