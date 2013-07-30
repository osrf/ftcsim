set (files 
  run_gazebo
)

install(PROGRAMS ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${FTC_SIM_PROJECT_NAME}-${FTC_SIM_VERSION}/ros/${PROJECT_NAME}/scripts/)
