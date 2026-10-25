cmake_minimum_required(VERSION 3.8)
project(fishbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 保留：URDF 模型 / Gazebo 仿真 / RViz 查看 / SLAM 建图
# （导航相关：Nav2 参数、地图、定位包已移除）
install(DIRECTORY launch urdf config worlds models
  DESTINATION share/${PROJECT_NAME})

# ---- 3D 重定位节点 (仿真侧副本, 迁自 test_relocalization) ----
# 仿真点云 + 先验 PCD -> GICP scan-to-map, 输出 /pose_correction (+ map->odom TF)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

add_executable(relocalization_3d src/relocalization_3d_node.cpp)
ament_target_dependencies(relocalization_3d
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions tf2_ros)
target_include_directories(relocalization_3d PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(relocalization_3d ${PCL_LIBRARIES})
target_compile_definitions(relocalization_3d PRIVATE ${PCL_DEFINITIONS})

install(TARGETS relocalization_3d
  DESTINATION lib/${PROJECT_NAME})

# 工具脚本（点云高度直方图 / PGM 转 PNG 预览地图 / STL 转 PGM 地图）
install(PROGRAMS
  scripts/height_hist.py
  scripts/pgm2png.py
  scripts/stl2pgm.py
  # 仿真点云 -> odom 系 (替代 FAST-LIO 的 /cloud_registered, 对齐实车语义)
  scripts/cloud_to_odom.py
  # 实时点云 -> 2D 占据栅格 (参考用, 不参与 Nav2 规划)
  scripts/realtime_grid.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
