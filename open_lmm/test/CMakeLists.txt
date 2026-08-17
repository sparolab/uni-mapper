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
