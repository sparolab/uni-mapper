set(_open_lmm_quality_support
  ${CMAKE_CURRENT_SOURCE_DIR}/quality/support)

add_executable(open_lmm_rigid_transform_property_tests
  quality/property/rigid_transform_property_tests.cpp)
target_include_directories(open_lmm_rigid_transform_property_tests PRIVATE
  ${_open_lmm_quality_support})
target_link_libraries(open_lmm_rigid_transform_property_tests PRIVATE
  open_lmm_contracts Eigen3::Eigen)
openlmm_set_global_target_properties(open_lmm_rigid_transform_property_tests)
openlmm_add_test(
  NAME open_lmm_rigid_transform_property_tests
  TARGET open_lmm_rigid_transform_property_tests
  LAYER L1 MODULE foundation.contracts OWNER RigidTransform
  INVARIANTS INV-12 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_runtime_revision_property_tests
  quality/property/runtime_revision_property_tests.cpp)
target_include_directories(open_lmm_runtime_revision_property_tests PRIVATE
  ${_open_lmm_quality_support})
target_link_libraries(open_lmm_runtime_revision_property_tests PRIVATE
  open_lmm_runtime_model_objects open_lmm_runtime_state_objects
  open_lmm_common open_lmm_algorithm_config open_lmm_utils open_lmm_profiling
  Eigen3::Eigen PCL::PCL gtsam nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_runtime_revision_property_tests)
openlmm_add_test(
  NAME open_lmm_runtime_revision_property_tests
  TARGET open_lmm_runtime_revision_property_tests
  LAYER L2 MODULE runtime.state OWNER RuntimeStateStore
  INVARIANTS INV-01 INV-03 INV-04 INV-07 INV-11
  LANES pr SANITIZERS asan-ubsan tsan)

add_executable(open_lmm_file_set_transaction_property_tests
  quality/property/file_set_transaction_property_tests.cpp)
target_include_directories(open_lmm_file_set_transaction_property_tests PRIVATE
  ${_open_lmm_quality_support})
target_link_libraries(open_lmm_file_set_transaction_property_tests PRIVATE
  open_lmm_storage_objects open_lmm_common open_lmm_algorithm_config
  open_lmm_utils open_lmm_profiling Eigen3::Eigen PCL::PCL gtsam
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(
  open_lmm_file_set_transaction_property_tests)
openlmm_add_test(
  NAME open_lmm_file_set_transaction_property_tests
  TARGET open_lmm_file_set_transaction_property_tests
  LAYER L3 MODULE storage.transactions OWNER FileSetTransaction
  INVARIANTS INV-03 INV-04 INV-07 INV-16
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_config_canonical_property_tests
  quality/property/config_canonical_property_tests.cpp)
target_include_directories(open_lmm_config_canonical_property_tests PRIVATE
  ${_open_lmm_quality_support})
target_link_libraries(open_lmm_config_canonical_property_tests PRIVATE
  open_lmm_utils nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_config_canonical_property_tests)
openlmm_add_test(
  NAME open_lmm_config_canonical_property_tests
  TARGET open_lmm_config_canonical_property_tests
  LAYER L2 MODULE config.schema OWNER ConfigSchema
  INVARIANTS INV-03 INV-12 INV-16
  LANES pr SANITIZERS asan-ubsan)

openlmm_add_test(
  NAME open_lmm_critical_coverage_contract_tests
  COMMAND /usr/bin/python3
  COMMAND_ARGS
    ${CMAKE_CURRENT_SOURCE_DIR}/quality/coverage/critical_coverage_contract_tests.py
    ${PROJECT_SOURCE_DIR}/../scripts/ci/critical_coverage.py
  LAYER L2 MODULE quality.coverage OWNER CriticalCoverage
  INVARIANTS INV-01 INV-03 INV-04 INV-07 INV-11 INV-18
  LANES pr)

if(OPEN_LMM_ENABLE_FUZZING)
  add_executable(open_lmm_config_schema_fuzz
    quality/fuzz/config_schema_fuzz.cpp)
  target_link_libraries(open_lmm_config_schema_fuzz PRIVATE
    open_lmm_utils nlohmann_json::nlohmann_json)
  openlmm_set_global_target_properties(open_lmm_config_schema_fuzz)
  openlmm_enable_fuzzing_target(open_lmm_config_schema_fuzz)

  add_executable(open_lmm_agent_id_fuzz
    quality/fuzz/agent_id_fuzz.cpp)
  target_link_libraries(open_lmm_agent_id_fuzz PRIVATE open_lmm_contracts)
  openlmm_set_global_target_properties(open_lmm_agent_id_fuzz)
  openlmm_enable_fuzzing_target(open_lmm_agent_id_fuzz)

  add_custom_target(open_lmm_fuzz_targets DEPENDS
    open_lmm_config_schema_fuzz open_lmm_agent_id_fuzz)
endif()
