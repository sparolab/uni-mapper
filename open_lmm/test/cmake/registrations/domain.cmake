add_executable(open_lmm_safety_regression_suite
  workflows/integration/safety_regression_tests.cpp)
target_link_libraries(open_lmm_safety_regression_suite PRIVATE
  open_lmm_map_server open_lmm_data_loader open_lmm_backend_optimizer
  open_lmm_dynamic_remover)
target_link_options(open_lmm_safety_regression_suite PRIVATE
  LINKER:--no-as-needed)
openlmm_set_global_target_properties(open_lmm_safety_regression_suite)

function(openlmm_add_safety_owner_suite target suite module owner invariants)
  openlmm_add_test(
    NAME ${target} TARGET open_lmm_safety_regression_suite
    COMMAND_ARGS --suite ${suite}
    LAYER L3 MODULE ${module} OWNER ${owner}
    INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan)
endfunction()

openlmm_add_safety_owner_suite(
  open_lmm_storage_file_set_tests 1 storage.transactions FileSetTransaction
  "INV-03;INV-04;INV-07")
openlmm_add_safety_owner_suite(
  open_lmm_domain_support_tests 2 domain.support DomainSupport
  "INV-03;INV-12;INV-16")
openlmm_add_safety_owner_suite(
  open_lmm_data_loader_behavior_tests 3 domain.data_loader DataLoaderFile
  "INV-04;INV-12;INV-16")
openlmm_add_safety_owner_suite(
  open_lmm_optimizer_behavior_tests 4 domain.optimization BackendOptimizer
  "INV-03;INV-04;INV-12")
openlmm_add_safety_owner_suite(
  open_lmm_remover_behavior_tests 5 domain.dynamic_removal DynamicRemover
  "INV-03;INV-04;INV-12;INV-16")
openlmm_add_safety_owner_suite(
  open_lmm_config_input_tests 6 config.document ConfigInput
  "INV-03;INV-04;INV-12;INV-16")
openlmm_add_safety_owner_suite(
  open_lmm_pipeline_integration_tests 7 workflows.integration PipelineIntegration
  "INV-03;INV-04;INV-07")

add_executable(open_lmm_algorithm_contract_tests
  domain/contract/algorithm_contract_tests.cpp
)
target_link_libraries(open_lmm_algorithm_contract_tests PRIVATE
  open_lmm_data_loader
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
)
openlmm_set_global_target_properties(open_lmm_algorithm_contract_tests)
openlmm_add_test(
  NAME open_lmm_algorithm_contract_tests
  TARGET open_lmm_algorithm_contract_tests
  LAYER L2 MODULE domain.algorithms OWNER AlgorithmContracts
  INVARIANTS INV-12 LANES pr SANITIZERS asan-ubsan)
add_executable(open_lmm_descriptor_engine_tests
  domain/descriptor/contract/descriptor_engine_tests.cpp
)
target_link_libraries(open_lmm_descriptor_engine_tests PRIVATE
  open_lmm_descriptor
)
openlmm_set_global_target_properties(open_lmm_descriptor_engine_tests)
openlmm_add_test(
  NAME open_lmm_descriptor_engine_tests
  TARGET open_lmm_descriptor_engine_tests
  LAYER L2 MODULE domain.descriptor OWNER DescriptorEngine
  INVARIANTS INV-12 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_descriptor_database_tests
  domain/loop_detection/contract/descriptor_database_tests.cpp
)
target_link_libraries(open_lmm_descriptor_database_tests PRIVATE
  descriptor_kdtree
  open_lmm_descriptor
)
openlmm_set_global_target_properties(open_lmm_descriptor_database_tests)
openlmm_add_test(
  NAME open_lmm_descriptor_database_tests
  TARGET open_lmm_descriptor_database_tests
  LAYER L2 MODULE domain.loop_detection OWNER DescriptorDatabase
  INVARIANTS INV-12 INV-16 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_alignment_decision_policy_tests
  domain/alignment/unit/alignment_decision_policy_tests.cpp
)
target_link_libraries(open_lmm_alignment_decision_policy_tests PRIVATE
  open_lmm_alignment
)
openlmm_set_global_target_properties(open_lmm_alignment_decision_policy_tests)
openlmm_add_test(
  NAME open_lmm_alignment_decision_policy_tests
  TARGET open_lmm_alignment_decision_policy_tests
  LAYER L1 MODULE domain.alignment OWNER AlignmentDecisionPolicy
  INVARIANTS INV-12 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_alignment_contract_fixture_tests
  domain/alignment/contract/alignment_contract_tests.cpp
)
target_link_libraries(open_lmm_alignment_contract_fixture_tests PRIVATE
  open_lmm_alignment
)
openlmm_set_global_target_properties(open_lmm_alignment_contract_fixture_tests)
openlmm_add_test(
  NAME open_lmm_alignment_contract_fixture_tests
  TARGET open_lmm_alignment_contract_fixture_tests
  LAYER L2 MODULE domain.alignment OWNER AlignmentContract
  INVARIANTS INV-12 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_alignment_map_resolution_tests
  domain/loop_detection/contract/alignment_map_resolution_tests.cpp
)
target_link_libraries(open_lmm_alignment_map_resolution_tests PRIVATE
  open_lmm_loop_detector
)
openlmm_set_global_target_properties(open_lmm_alignment_map_resolution_tests)
openlmm_add_test(
  NAME open_lmm_alignment_map_resolution_tests
  TARGET open_lmm_alignment_map_resolution_tests
  LAYER L2 MODULE domain.loop_detection OWNER AlignmentMapResolution
  INVARIANTS INV-12 LANES pr SANITIZERS asan-ubsan)
