cmake_minimum_required(VERSION 3.8)
project(test_livox_build_pcd_save)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(test_livox_build_pcd_save_node src/test_livox_build_pcd_save.cpp)
target_compile_features(test_livox_build_pcd_save_node PUBLIC cxx_std_17)  # std::filesystem 需要
ament_target_dependencies(test_livox_build_pcd_save_node
  rclcpp
  sensor_msgs
)

install(TARGETS test_livox_build_pcd_save_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
