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
set (FTC_SIM_PATCH_VERSION 1)
set (FTC_SIM_VERSION ${FTC_SIM_MAJOR_VERSION}.${FTC_SIM_MINOR_VERSION})
set (FTC_SIM_VERSION_FULL ${FTC_SIM_MAJOR_VERSION}.${FTC_SIM_MINOR_VERSION}.${FTC_SIM_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${FTC_SIM_VERSION_FULL}")

include (ExternalProject)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO REQUIRED gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#################################################
# Packaging configuration
set (CPACK_PACKAGE_VERSION "${FTC_SIM_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${FTC_SIM_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${FTC_SIM_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${FTC_SIM_PATCH_VERSION}")
list (APPEND CPACK_SOURCE_GENERATOR "TBZ2")
list (APPEND CPACK_SOURCE_IGNORE_FILES ";Ogre.log;TODO;/.hg/;.hgignore;.swp$;/build/")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "@PROJECT_NAME@-@FTC_SIM_VERSION_FULL@")
include (CPack)

#################################################
# Infer ROS distro from the user's environment, allowing the user to override
# manually.
if (NOT ROS_DISTRO)
  find_program(ROSVERSION_PATH rosversion)
  if(NOT ROSVERSION_PATH)
    message(FATAL_ERROR "Can't find path to rosversion executable, which is needed to determine which ROS distro you're using.  Did you source a ROS setup file (e.g., /opt/ros/fuerte/setup.sh)?  You can manually override this check by passing `-DROS_DISTRO=<distro>` to cmake (but that really should not be necessary).")
  endif()
  execute_process(COMMAND ${ROSVERSION_PATH} -d
                  OUTPUT_VARIABLE ROS_DISTRO
                  RESULT_VARIABLE rosversion_error
                  ERROR_VARIABLE rosversion_stderr
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(rosversion_error)
    message(FATAL_ERROR "`rosversion -d` failed. stdout: ${ROS_DISTRO}.  stderr: ${rosversion_stderr}.")
  endif()
endif()
message(STATUS "Building against ROS distro ${ROS_DISTRO}, expected to be installed in /opt/ros/${ROS_DISTRO}")

# rosversion can appear in /usr/bin and will return a valid result even if the
# user hasn't sourced a ROS setup.sh.  So we'll further check that ROS_ROOT is
# set and bail if not.
if("$ENV{ROS_ROOT}" STREQUAL "")
  message(FATAL_ERROR "ROS_ROOT is not set. Did you source a ROS setup file (e.g., /opt/ros/fuerte/setup.sh)?")
endif()

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "Release" CACHE STRING
    "Choose the type of build, options are: Debug Release RelWithDebInfo" FORCE)
  message(STATUS "Build type not selected: Release selected by default")
endif()

#################################################
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

configure_file(${CMAKE_SOURCE_DIR}/cmake/setup-unversioned.sh.in ${PROJECT_BINARY_DIR}/unversioned/setup.sh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/unversioned/setup.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ftcsim/)

