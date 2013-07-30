set (files
  ftc_demo.launch
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${FTC_SIM_PROJECT_NAME}-${FTC_SIM_VERSION}/ros/${PROJECT_NAME}/launch/)
