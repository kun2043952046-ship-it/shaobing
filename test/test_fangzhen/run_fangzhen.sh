#!/usr/bin/env bash
# ============================================================
#  一键启动：Gazebo 仿真 + 点云转 /scan + slam_toolbox 建图 + RViz
#
#  用法：
#    ./run_fangzhen.sh     # 全部启动（Ctrl+C 一次性关闭所有节点）
#
#  说明：
#    - 脚本自动 cd 到本文件所在工作空间并 source install/setup.bash
#    - ROS 2 Humble 已写进 ~/.bashrc 时无需额外操作；否则自动 source /opt/ros/humble
#    - 首次使用或改了 urdf/launch/config 后，先执行 colcon build
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

RVIZ_CONFIG="$WS_DIR/install/fishbot_description/share/fishbot_description/config/display_robot_model.rviz"

# ---------- 1.5) 依赖检查（缺包只提示，不中断）----------
for pkg in gazebo_ros gazebo_ros2_control controller_manager \
           pointcloud_to_laserscan slam_toolbox nav2_map_server rviz2; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[WARN] 缺少依赖包 $pkg，安装：sudo apt install ros-humble-${pkg//_/-}" >&2
  fi
done

# ---------- 2) Ctrl+C 时统一清理子进程 ----------
PIDS=()
# 用 setsid 让每个命令成为独立进程组（pgid = pid），清理时整组发信号，
# 否则 gzserver 这类孙进程会收到信号后残留下来
start_bg() {
  setsid "$@" &
  PIDS+=($!)
}

cleanup() {
  trap - EXIT INT TERM
  echo
  echo "[INFO] 正在关闭所有节点..."
  # 1) 先整组发 SIGINT，给 launch 和 gzserver 正常退出的机会
  for pid in "${PIDS[@]}"; do
    kill -INT -"$pid" 2>/dev/null
  done
  # 2) 最多等 10s，进程组仍存活则升级为 SIGTERM
  for _ in $(seq 1 10); do
    alive=0
    for pid in "${PIDS[@]}"; do
      kill -0 -"$pid" 2>/dev/null && alive=1
    done
    [ "$alive" -eq 0 ] && break
    sleep 1
  done
  for pid in "${PIDS[@]}"; do
    kill -TERM -"$pid" 2>/dev/null
  done
  sleep 2
  # 3) 兜底 SIGKILL
  for pid in "${PIDS[@]}"; do
    kill -KILL -"$pid" 2>/dev/null
  done
  wait 2>/dev/null
  echo "[INFO] 已全部退出"
}
trap cleanup EXIT INT TERM

# ---------- 3) 分步启动 ----------
echo "[1/3] 启动 Gazebo 仿真（机器人 + Mid-360 + ros2_control）..."
start_bg ros2 launch fishbot_description gazebo_sim_launch.py
sleep 3

echo "[2/3] 启动建图（点云转 /scan + slam_toolbox）..."
start_bg ros2 launch fishbot_description slam_mapping.launch.py
sleep 3

echo "[3/3] 启动 RViz（复用 display_robot_model.rviz）..."
# 注意参数顺序：-d 必须在 --ros-args 之前，否则 rviz2 会抛
# rclcpp::exceptions::UnknownROSArgsError（found unknown ROS arguments: '-d'）并立刻退出
start_bg ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true

echo
echo "[INFO] 全部启动完成：Gazebo + slam_toolbox + RViz"
echo "[INFO] 键盘遥控（另开终端）：ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "[INFO] 存图（另开终端）：ros2 run nav2_map_server map_saver_cli -f $WS_DIR/my_map"
echo "[INFO] Ctrl+C 关闭所有节点"

wait
