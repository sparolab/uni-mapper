foreach(required IN ITEMS OPEN_LMM_REPOSITORY_ROOT OPEN_LMM_BUILD_DIR
                          OPEN_LMM_BENCHMARK_TEST_ROOT)
  if(NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
    message(FATAL_ERROR "missing required variable ${required}")
  endif()
endforeach()

file(REMOVE_RECURSE "${OPEN_LMM_BENCHMARK_TEST_ROOT}")
execute_process(
  COMMAND bash
    "${OPEN_LMM_REPOSITORY_ROOT}/scripts/benchmark/run_benchmarks.sh"
    --build "${OPEN_LMM_BUILD_DIR}"
    --profile contract
    --fixture small-v1
    --scenario data-load
    --repetitions 1
    --warmup 0
    --output "${OPEN_LMM_BENCHMARK_TEST_ROOT}"
    --container-digest
      sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  WORKING_DIRECTORY "${OPEN_LMM_REPOSITORY_ROOT}"
  RESULT_VARIABLE run_result
  OUTPUT_VARIABLE run_output
  ERROR_VARIABLE run_error)
if(NOT run_result EQUAL 0)
  message(FATAL_ERROR
    "fresh-process benchmark failed (${run_result})\n${run_output}\n${run_error}")
endif()
set(bundle "${OPEN_LMM_BENCHMARK_TEST_ROOT}/data-load/bundle.json")
set(owner_bundle
  "${OPEN_LMM_BENCHMARK_TEST_ROOT}/data-load/owner-bundle.json")
if(NOT EXISTS "${bundle}" OR NOT EXISTS "${owner_bundle}")
  message(FATAL_ERROR "benchmark bundle was not generated")
endif()
file(READ "${bundle}" bundle_json)
string(JSON comparison GET "${bundle_json}" comparison)
string(JSON report_count LENGTH "${bundle_json}" reports)
if(NOT comparison STREQUAL "uncalibrated" OR NOT report_count EQUAL 1)
  message(FATAL_ERROR
    "unexpected bundle contract: comparison=${comparison} reports=${report_count}")
endif()

file(READ "${owner_bundle}" owner_bundle_json)
string(JSON public_key GET "${bundle_json}" key)
string(JSON owner_key GET "${owner_bundle_json}" key)
set(contract_catalog "${OPEN_LMM_BENCHMARK_TEST_ROOT}-catalog.json")
file(WRITE "${contract_catalog}"
  "{\n"
  "  \"schema_version\": 1,\n"
  "  \"catalog_id\": \"contract-public-owner-v1\",\n"
  "  \"baselines\": [\n"
  "    {\"schema_version\":1,\"baseline_id\":\"contract-public-v1\","
  "\"key\":${public_key},"
  "\"source_bundle_sha256\":\"sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\","
  "\"metrics\":[{\"name\":\"timing.wall_time_ns\",\"statistic\":\"median\",\"expected\":0,\"relative_allowance\":0,\"absolute_allowance\":1000000000000000000}]},\n"
  "    {\"schema_version\":1,\"baseline_id\":\"contract-owner-v1\","
  "\"key\":${owner_key},"
  "\"source_bundle_sha256\":\"sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\","
  "\"metrics\":[{\"name\":\"timing.wall_time_ns\",\"statistic\":\"median\",\"expected\":0,\"relative_allowance\":0,\"absolute_allowance\":1000000000000000000}]}\n"
  "  ]\n"
  "}\n")
set(calibrated_root "${OPEN_LMM_BENCHMARK_TEST_ROOT}-calibrated")
file(REMOVE_RECURSE "${calibrated_root}")
execute_process(
  COMMAND bash
    "${OPEN_LMM_REPOSITORY_ROOT}/scripts/benchmark/run_benchmarks.sh"
    --build "${OPEN_LMM_BUILD_DIR}"
    --profile contract
    --fixture small-v1
    --scenario data-load
    --repetitions 1
    --warmup 0
    --output "${calibrated_root}"
    --container-digest
      sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
    --baseline "${contract_catalog}"
  WORKING_DIRECTORY "${OPEN_LMM_REPOSITORY_ROOT}"
  RESULT_VARIABLE calibrated_result
  OUTPUT_VARIABLE calibrated_output
  ERROR_VARIABLE calibrated_error)
if(NOT calibrated_result EQUAL 0)
  message(FATAL_ERROR
    "public/owner calibrated benchmark failed (${calibrated_result})\n"
    "${calibrated_output}\n${calibrated_error}")
endif()
foreach(calibrated_bundle IN ITEMS bundle.json owner-bundle.json)
  file(READ "${calibrated_root}/data-load/${calibrated_bundle}"
    calibrated_json)
  string(JSON calibrated_comparison GET "${calibrated_json}" comparison)
  if(NOT calibrated_comparison STREQUAL "pass")
    message(FATAL_ERROR
      "baseline catalog did not gate ${calibrated_bundle}: "
      "${calibrated_comparison}")
  endif()
endforeach()

set(pair_root "${OPEN_LMM_BENCHMARK_TEST_ROOT}-pair")
file(REMOVE_RECURSE "${pair_root}")
execute_process(
  COMMAND bash
    "${OPEN_LMM_REPOSITORY_ROOT}/scripts/benchmark/run_benchmarks.sh"
    --build "${OPEN_LMM_BUILD_DIR}"
    --profile contract
    --fixture small-v1
    --scenario map-update-pair
    --repetitions 1
    --warmup 0
    --output "${pair_root}"
    --container-digest
      sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  WORKING_DIRECTORY "${OPEN_LMM_REPOSITORY_ROOT}"
  RESULT_VARIABLE pair_result
  OUTPUT_VARIABLE pair_output
  ERROR_VARIABLE pair_error)
if(NOT pair_result EQUAL 0)
  message(FATAL_ERROR
    "paired map-update benchmark failed (${pair_result})\n${pair_output}\n${pair_error}")
endif()
set(pair_report "${pair_root}/map-update-pair/report.json")
if(NOT EXISTS "${pair_report}" OR
   NOT EXISTS "${pair_root}/map-update-pair/parity-1.log" OR
   EXISTS "${pair_root}/map-update-pair/artifacts")
  message(FATAL_ERROR "paired evidence was not finalized correctly")
endif()
file(READ "${pair_report}" pair_json)
string(JSON pair_status GET "${pair_json}" result)
string(JSON pair_comparison GET "${pair_json}" comparison)
if(NOT pair_status STREQUAL "pass" OR
   NOT pair_comparison STREQUAL "uncalibrated")
  message(FATAL_ERROR
    "unexpected pair contract: result=${pair_status} comparison=${pair_comparison}")
endif()
file(REMOVE_RECURSE "${OPEN_LMM_BENCHMARK_TEST_ROOT}")
file(REMOVE_RECURSE "${pair_root}")
file(REMOVE_RECURSE "${calibrated_root}")
file(REMOVE "${contract_catalog}")
