add_executable(open_lmm_visualization_projector_suite
  visualization/fault_concurrency/visualization_projector_tests.cpp)
target_link_libraries(open_lmm_visualization_projector_suite PRIVATE
  open_lmm_visualization_projection_objects open_lmm_common
  open_lmm_algorithm_config open_lmm_utils open_lmm_profiling
  Eigen3::Eigen PCL::PCL gtsam nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_visualization_projector_suite)

function(openlmm_add_visualization_suite target selector layer invariants)
  openlmm_add_test(
    NAME ${target} TARGET open_lmm_visualization_projector_suite
    COMMAND_ARGS --suite ${selector}
    LAYER ${layer} MODULE visualization.projection OWNER VisualizationProjector
    INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan)
endfunction()

openlmm_add_visualization_suite(
  open_lmm_visualization_projector_tests 1 L2 "INV-09;INV-16")
openlmm_add_visualization_suite(
  open_lmm_visualization_projector_fault_tests 2 L5 "INV-04;INV-09;INV-10")
