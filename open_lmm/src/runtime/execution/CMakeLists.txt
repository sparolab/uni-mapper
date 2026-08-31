add_library(open_lmm_runtime_execution_objects OBJECT
  pipeline.cpp
  stage_executor.cpp
  stages/alignment_artifact_store.cpp
  stages/alignment_executor.cpp
  stages/data_load_executor.cpp
  stages/map_update_executor.cpp
  stages/optimize_executor.cpp
  stages/save_executor.cpp
  stages/stage_coordinator.cpp
  nodes/map_update_node.cpp
)
target_link_libraries(open_lmm_runtime_execution_objects PRIVATE
  Eigen3::Eigen
  PCL::PCL
  gtsam
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_profiling
  nlohmann_json::nlohmann_json
)
openlmm_set_global_target_properties(open_lmm_runtime_execution_objects)
