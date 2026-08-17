add_executable(open_lmm_safety_regression_tests
  safety_regression_tests.cpp
)
target_link_libraries(open_lmm_safety_regression_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_safety_regression_tests)
target_link_options(open_lmm_safety_regression_tests PRIVATE
  LINKER:--no-as-needed
)
add_test(NAME open_lmm_safety_regression_tests
         COMMAND open_lmm_safety_regression_tests)

add_executable(open_lmm_artifact_compare artifact_compare.cpp)
target_link_libraries(open_lmm_artifact_compare PRIVATE PCL::PCL)
openlmm_set_global_target_properties(open_lmm_artifact_compare)

add_executable(open_lmm_pipeline_controller_tests
  pipeline_controller_tests.cpp
)
target_link_libraries(open_lmm_pipeline_controller_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_pipeline_controller_tests)
add_test(NAME open_lmm_pipeline_controller_tests
         COMMAND open_lmm_pipeline_controller_tests)

add_executable(open_lmm_map_alignment_coordinator_tests
  map_alignment_coordinator_tests.cpp
)
target_link_libraries(open_lmm_map_alignment_coordinator_tests PRIVATE
  open_lmm_loop_detector
)
openlmm_set_global_target_properties(open_lmm_map_alignment_coordinator_tests)
add_test(NAME open_lmm_map_alignment_coordinator_tests
         COMMAND open_lmm_map_alignment_coordinator_tests)

add_executable(open_lmm_gui_plugin_tests gui_plugin_tests.cpp)
target_link_libraries(open_lmm_gui_plugin_tests PRIVATE open_lmm_gui_core)
openlmm_set_global_target_properties(open_lmm_gui_plugin_tests)
add_test(NAME open_lmm_gui_plugin_tests COMMAND open_lmm_gui_plugin_tests)

add_executable(open_lmm_profiling_macro_tests
  profiling_macro_tests.cpp
)
target_link_libraries(open_lmm_profiling_macro_tests PRIVATE
  open_lmm_profiling
)
openlmm_set_global_target_properties(open_lmm_profiling_macro_tests)
add_test(NAME open_lmm_profiling_macro_tests
         COMMAND open_lmm_profiling_macro_tests)

add_executable(open_lmm_logging_tests logging_tests.cpp)
target_link_libraries(open_lmm_logging_tests PRIVATE open_lmm_utils)
openlmm_set_global_target_properties(open_lmm_logging_tests)
add_test(NAME open_lmm_logging_tests COMMAND open_lmm_logging_tests)

add_test(
  NAME open_lmm_architecture_boundary_tests
  COMMAND ${CMAKE_COMMAND}
          -DOPEN_LMM_SOURCE_DIR=${PROJECT_SOURCE_DIR}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/architecture_boundary_tests.cmake
)
