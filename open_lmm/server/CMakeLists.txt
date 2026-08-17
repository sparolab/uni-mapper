add_library(open_lmm_map_server STATIC)
target_sources(open_lmm_map_server PRIVATE
  map_server.cpp
  stage_runner.cpp
  artifact_repository.cpp
  pipeline_controller.cpp
)
target_link_libraries(open_lmm_map_server PUBLIC
  Eigen3::Eigen
  open_lmm_backend_optimizer
  open_lmm_loop_detector
  open_lmm_data_loader
  open_lmm_dynamic_remover
)
openlmm_set_global_target_properties(open_lmm_map_server)
