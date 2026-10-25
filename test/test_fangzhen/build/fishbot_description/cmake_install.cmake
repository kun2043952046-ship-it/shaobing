# Install script for directory: /home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/install/fishbot_description")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description" TYPE DIRECTORY FILES
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/launch"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/urdf"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/config"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/worlds"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/models"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fishbot_description/relocalization_3d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fishbot_description/relocalization_3d")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fishbot_description/relocalization_3d"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/fishbot_description" TYPE EXECUTABLE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/relocalization_3d")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fishbot_description/relocalization_3d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fishbot_description/relocalization_3d")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fishbot_description/relocalization_3d"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/fishbot_description/relocalization_3d")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/fishbot_description" TYPE PROGRAM FILES
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/scripts/height_hist.py"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/scripts/pgm2png.py"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/scripts/stl2pgm.py"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/scripts/cloud_to_odom.py"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/scripts/realtime_grid.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/fishbot_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/fishbot_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description/environment" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description/environment" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_index/share/ament_index/resource_index/packages/fishbot_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description/cmake" TYPE FILE FILES
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_core/fishbot_descriptionConfig.cmake"
    "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/ament_cmake_core/fishbot_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fishbot_description" TYPE FILE FILES "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/src/fishbot_description/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/lele/aaaaa_mine_NJIT_Vision_Gen2/Navigation/test/test_fangzhen/build/fishbot_description/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
