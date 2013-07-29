set (files
  ftc_demo.launch
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/${ROS_PACKAGE_NAME}/launch/)
