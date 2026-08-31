function(openlmm_add_visualization_suite target selector layer invariants)
  add_executable(${target}
    visualization/fault_concurrency/visualization_projector_tests.cpp)
  target_compile_definitions(${target} PRIVATE
    OPEN_LMM_VISUALIZATION_SUITE=${selector})
  target_link_libraries(${target} PRIVATE
    open_lmm_visualization_projection_objects open_lmm_common
    open_lmm_algorithm_config open_lmm_utils open_lmm_profiling
    Eigen3::Eigen PCL::PCL gtsam nlohmann_json::nlohmann_json)
  openlmm_set_global_target_properties(${target})
  openlmm_add_test(
    NAME ${target} TARGET ${target}
    LAYER ${layer} MODULE visualization.projection OWNER VisualizationProjector
    INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan)
endfunction()

openlmm_add_visualization_suite(
  open_lmm_visualization_projector_tests 1 L2 "INV-09;INV-16")
openlmm_add_visualization_suite(
  open_lmm_visualization_projector_fault_tests 2 L5 "INV-04;INV-09;INV-10")
