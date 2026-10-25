
cd /home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation
source /opt/ros/humble/setup.bash
source /home/lele/source_code_from_git/ws_livox/install/setup.bash
colcon build --packages-select test_draw_pgm_realtime
source install/setup.bash
#ros2 launch test_draw_pgm_realtime realtime_pgm.launch.py

#需要通讯时执行
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ros2 launch test_draw_pgm_realtime realtime_pgm.launch.py can_enabled:=true