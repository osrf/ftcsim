set(MODEL_NAME frc_red_ring)
set (files 
  model.config
  model.sdf
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/${MODEL_NAME}/)
