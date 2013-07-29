set (files 
  chassis.stl
  claw.stl
  lift.stl
  wheel.stl
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/${MODEL_NAME}/meshes)

