#!/bin/bash
# 锁定后位姿修正稳定性验证 (C 组): 漂移停止后修正量应迅速稳定。
# 用法: bash test_stable.sh
set +u
TOOLS="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$TOOLS/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

cleanup() {
  pkill -f test_relocalization_node 2>/dev/null
  pkill -f fake_drift 2>/dev/null
}
trap cleanup EXIT TERM INT
cleanup; sleep 2

python3 "$TOOLS/fake_drift.py" --ros-args -p drift:=0.2 > /tmp/c_fake.log 2>&1 &
ros2 run test_relocalization test_relocalization_node --ros-args \
  -p resolution:=0.03 -p dilate:=1 > /tmp/c_node.log 2>&1 &
sleep 5

ros2 param set /test_relocalization dynamic_obstacle_enabled true > /dev/null 2>&1
sleep 6
echo "=== 停止漂移前一刻的修正量 ==="
python3 "$TOOLS/sample_corr.py" 1 2>/dev/null | grep -E "^CSV" | tail -1

ros2 param set /fake_drift drift 0.0 > /dev/null 2>&1
sleep 2
echo "=== 停止漂移后 6 s (期望 span 很小, 无持续单向漂移) ==="
python3 "$TOOLS/sample_corr.py" 6 2>/dev/null | grep -E "^(CSV|N=|x:|y:|相邻)"

echo "=== 节点日志 (后 3 条) ==="
grep -aE "配准|速率逼近" /tmp/c_node.log | tail -3 | sed 's/.*\[test_relocalization\]: //'
