cmake_minimum_required(VERSION 3.8)
project(test_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# 节点: 继承自主节点 test_draw_pgm_realtime 的建图/历史缓冲/位姿修正,
#   裁剪掉运动控制/A*/占点/chase/CAN/Nav2 交互, 作为重定位独立开发基底。
add_executable(test_relocalization_node src/test_relocalization_node.cpp)
ament_target_dependencies(test_relocalization_node
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)

# 3D 重定位节点: map_clouds.bin 先验地图 + GICP scan-to-map,
#   输出 /pose_correction + map->odom TF + fitness
add_executable(relocalization_3d src/relocalization_3d_node.cpp)
ament_target_dependencies(relocalization_3d
  rclcpp std_msgs sensor_msgs geometry_msgs pcl_conversions tf2_ros)
target_include_directories(relocalization_3d PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(relocalization_3d ${PCL_LIBRARIES})
target_compile_definitions(relocalization_3d PRIVATE ${PCL_DEFINITIONS})

install(TARGETS test_relocalization_node relocalization_3d
  DESTINATION lib/${PROJECT_NAME})

# launch / config / scripts (网页查看器)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/relocalization_viewer.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
