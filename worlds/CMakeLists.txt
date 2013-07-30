set (files 
ftc_2013.world
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${FTC_SIM_VERSION}/worlds/)
