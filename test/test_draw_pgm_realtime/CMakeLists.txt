cmake_minimum_required(VERSION 3.8)
project(test_draw_pgm_realtime)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(test_draw_pgm_realtime_node
  src/test_draw_pgm_realtime.cpp
)

ament_target_dependencies(
  test_draw_pgm_realtime_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  nav2_msgs
)

# SocketCAN 直发: linux/can.h 属于内核头文件, 无需额外 find_package
# (运行环境为 Linux, 发送路径直接使用 socket/bind/write 系统调用)

install(TARGETS
  test_draw_pgm_realtime_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/pgm_viewer.py scripts/nav2_bridge.py
  scripts/diag_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 调试日志目录 (逐帧点云统计 CSV + 异常帧 PCD 快照):
# 随包安装一份占位, 使 launch 的 debug_log_dir 默认值
# (share/${PROJECT_NAME}/log) 开箱即用; 运行时节点会向其中写入日志文件。
install(DIRECTORY log
  DESTINATION share/${PROJECT_NAME}
)

# 加固: 编辑器重写脚本后可能会丢掉执行位,
# 而 symlink-install 下 libexec 里是软链接, 权限跟随源文件,
# 导致 launch 报 "executable 'xxx.py' not found".
# 这里每次安装都强制补上可执行位 (chmod 会跟随软链接作用于源文件).
install(CODE "
  execute_process(COMMAND chmod +x
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/pgm_viewer.py\"
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/nav2_bridge.py\"
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/diag_logger.py\")
")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
