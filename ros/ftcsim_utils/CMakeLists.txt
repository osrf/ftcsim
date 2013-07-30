cmake_minimum_required(VERSION 2.4.6)

# Push the ROS_PACKAGE_PATH and ROS_ROOT we were passed up into the environment, to allow us to find things at configure time
set(ENV{ROS_ROOT} ${ROS_ROOT})
set(ENV{ROS_PACKAGE_PATH} ${ROS_PACKAGE_PATH})
# Push the PKG_CONFIG_PATH we were passed up into the environment, to allow us to find gazebo
set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})

project(ftcsim_utils)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

add_subdirectory(launch)
add_subdirectory(scripts)

#Get Gazebo Package Configuration
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# Include Gazebo.
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_init()

# VERY IMPORTANT: Tell CMake not to strip off the RPATH from libs and executables during installation.  This is needed to resolve
# # dependencies on ROS libraries.
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH 1)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(FTC src/FTC.cc src/FTC.hh)
target_link_libraries(FTC ${GAZEBO_libraries})

rosbuild_add_library(FTC2 src/FTC2.cc src/FTC2.hh)
target_link_libraries(FTC2 ${GAZEBO_libraries})


rosbuild_add_library(FTCCameraPlugin src/CameraPlugin.cc src/CameraPlugin.hh)
target_link_libraries(FTCCameraPlugin ${GAZEBO_libraries})

install(FILES manifest.xml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${FTC_SIM_PROJECT_NAME}-${FTC_SIM_VERSION}/ros/${PROJECT_NAME})

install(TARGETS FTC FTC2 FTCCameraPlugin 
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${FTC_SIM_PROJECT_NAME}-${FTC_SIM_VERSION}/plugins)
