set(ROS_PACKAGE_NAME first_utils)

add_subdirectory(launch)
add_subdirectory(scripts)

set (files
  manifest.xml
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${ROS_PACKAGE_NAME})
