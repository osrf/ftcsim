set (files 
  run_gazebo
)

install(PROGRAMS ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${ROS_PACKAGE_NAME}/scripts/)
