cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

set (CMAKE_INSTALL_PREFIX /usr CACHE STRING "Install path prefix")

project (ftcsim)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set (FTC_SIM_MAJOR_VERSION 0)
set (FTC_SIM_MINOR_VERSION 1)
set (FTC_SIM_PATCH_VERSION 0)
set (FTC_SIM_VERSION ${FTC_SIM_MAJOR_VERSION}.${FTC_SIM_MINOR_VERSION})
set (FTC_SIM_VERSION_FULL ${FTC_SIM_MAJOR_VERSION}.${FTC_SIM_MINOR_VERSION}.${FTC_SIM_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${FTC_SIM_VERSION_FULL}")

include (ExternalProject)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO REQUIRED gazebo)
endif()

# rosversion can appear in /usr/bin and will return a valid result even if the
# user hasn't sourced a ROS setup.sh.  So we'll further check that ROS_ROOT is
# set and bail if not.
if("$ENV{ROS_ROOT}" STREQUAL "")
  message(FATAL_ERROR "ROS_ROOT is not set. Did you source a ROS setup file (e.g., /opt/ros/fuerte/setup.sh)?")
endif()

# add_subdirectory(media)
add_subdirectory(worlds)
add_subdirectory(models)
add_subdirectory(ros)

########### Add uninstall target ###############
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")


#####################################
# Configure the setup script
configure_file(${CMAKE_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.sh DESTINATION
  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${FTC_SIM_VERSION}/)
