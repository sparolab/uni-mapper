set(open_lmm_cli_contract_script
  "${CMAKE_CURRENT_SOURCE_DIR}/cli_contract_tests.cmake")
set(open_lmm_cli_fixture_source
  "${CMAKE_CURRENT_SOURCE_DIR}/fixture")
set(open_lmm_cli_contract_root
  "${CMAKE_CURRENT_BINARY_DIR}/contract")

function(openlmm_add_cli_contract_test name case_name)
  add_test(NAME ${name}
    COMMAND ${CMAKE_COMMAND}
      -DOPEN_LMM_BATCH_EXECUTABLE=$<TARGET_FILE:open_lmm_batch>
      -DOPEN_LMM_CLI_CASE=${case_name}
      -DOPEN_LMM_CLI_TEST_ROOT=${open_lmm_cli_contract_root}/${case_name}
      -DOPEN_LMM_CLI_FIXTURE_SOURCE=${open_lmm_cli_fixture_source}
      -P ${open_lmm_cli_contract_script})
  set_tests_properties(${name} PROPERTIES
    LABELS
      "layer:L5;module:applications.cli;owner:OpenLmmBatch;invariant:INV-13;invariant:INV-14;invariant:INV-15;lane:pr"
    TIMEOUT 90)
endfunction()

openlmm_add_cli_contract_test(open_lmm_cli_help_contract help)
openlmm_add_cli_contract_test(open_lmm_cli_invalid_noarg_contract invalid_noarg)
openlmm_add_cli_contract_test(open_lmm_cli_invalid_many_contract invalid_many)
openlmm_add_cli_contract_test(
  open_lmm_cli_bootstrap_failure_contract bootstrap_failure)
openlmm_add_cli_contract_test(open_lmm_cli_run_failure_contract run_failure)
openlmm_add_cli_contract_test(open_lmm_cli_success_contract success)
