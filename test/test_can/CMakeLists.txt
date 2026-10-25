cmake_minimum_required(VERSION 3.8)
project(test_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(test_can_node src/test_can.cpp)
target_compile_features(test_can_node PUBLIC cxx_std_17)
ament_target_dependencies(test_can_node
  rclcpp
)

# 内置 HTTP 网页面板需要 pthread
find_package(Threads REQUIRED)
target_link_libraries(test_can_node Threads::Threads)

install(TARGETS test_can_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
