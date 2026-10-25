cmake_minimum_required(VERSION 3.8)
project(test_pcd_To_pgm)

# PCL 1.12（ROS 2 Humble 自带）要求 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(pcd_to_pgm src/test_pcd_To_pgm.cpp)
target_include_directories(pcd_to_pgm PRIVATE
  ${PCL_INCLUDE_DIRS}
  include
)
target_compile_definitions(pcd_to_pgm PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(pcd_to_pgm ${PCL_LIBRARIES})

install(TARGETS pcd_to_pgm
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
