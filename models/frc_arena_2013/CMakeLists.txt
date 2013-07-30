set(MODEL_NAME frc_arena_2013)
set (files 
  model.config
  model.sdf
)

add_subdirectory(meshes)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${FTC_SIM_VERSION}/models/${MODEL_NAME}/)
