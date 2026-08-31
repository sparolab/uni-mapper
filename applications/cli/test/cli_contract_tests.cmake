cmake_minimum_required(VERSION 3.25)

foreach(required_variable IN ITEMS
    OPEN_LMM_BATCH_EXECUTABLE
    OPEN_LMM_CLI_CASE
    OPEN_LMM_CLI_TEST_ROOT
    OPEN_LMM_CLI_FIXTURE_SOURCE)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

if(NOT EXISTS "${OPEN_LMM_BATCH_EXECUTABLE}")
  message(FATAL_ERROR
    "batch executable is missing: ${OPEN_LMM_BATCH_EXECUTABLE}")
endif()

function(fail_contract reason result stdout stderr)
  message(FATAL_ERROR
    "${OPEN_LMM_CLI_CASE} contract failed: ${reason} (${result})\n"
    "stdout:\n${stdout}\n"
    "stderr:\n${stderr}")
endfunction()

function(assert_terminal_newline stream_name payload)
  string(LENGTH "${payload}" payload_length)
  if(payload_length EQUAL 0)
    message(FATAL_ERROR
      "${OPEN_LMM_CLI_CASE} ${stream_name} unexpectedly empty")
  endif()
  math(EXPR last_index "${payload_length} - 1")
  string(SUBSTRING "${payload}" ${last_index} 1 last_character)
  if(NOT last_character STREQUAL "\n")
    message(FATAL_ERROR
      "${OPEN_LMM_CLI_CASE} ${stream_name} lacks a terminal newline")
  endif()
endfunction()

function(write_valid_fixture fixture_root)
  set(config_root "${fixture_root}/config")
  set(data_root "${fixture_root}/data")
  set(output_root "${fixture_root}/output")
  file(MAKE_DIRECTORY
    "${config_root}/server"
    "${config_root}/core"
    "${data_root}/agent1/Scans")
  file(COPY "${OPEN_LMM_CLI_FIXTURE_SOURCE}/poses.txt"
    DESTINATION "${data_root}/agent1")
  file(COPY "${OPEN_LMM_CLI_FIXTURE_SOURCE}/000000.pcd"
    DESTINATION "${data_root}/agent1/Scans")
  file(WRITE "${config_root}/config.json"
    "{\"global\":{\"config_map_server\":\"server/map.json\","
    "\"config_data_loader\":\"core/data.json\","
    "\"config_loop_detector\":\"core/loop.json\","
    "\"config_backend_optimizer\":\"core/optimizer.json\","
    "\"config_dynamic_remover\":\"core/remover.json\"},"
    "\"directory\":{\"root_dir_path\":\"${data_root}\","
    "\"sub_dir_list\":[\"agent1\"],"
    "\"root_save_dir\":\"${output_root}\"}}")
  file(WRITE "${config_root}/server/map.json"
    "{\"map_server\":{\"enable_map_updater\":false,"
    "\"anchor_agent_index\":0,\"save_voxel_size\":0.2,"
    "\"parallel_data_load\":false,\"parallel_map_update\":false,"
    "\"max_parallel_agents\":1}}")
  file(WRITE "${config_root}/core/data.json"
    "{\"data_loader\":{\"data_loader_type\":\"file_based\","
    "\"pose_format\":\"kitti\",\"pose_file_name\":\"poses.txt\","
    "\"extrinsic\":[0,0,0,0,0,0,1],\"scan_type\":\"pcd\","
    "\"scan_dir_name\":\"Scans\",\"voxel_size\":0.5,"
    "\"min_range\":1.0,\"max_range\":60.0,\"delimiter\":\" \"}}")
  file(WRITE "${config_root}/core/loop.json"
    "{\"loop_detector\":{\"loop_detector_type\":\"kdtree\","
    "\"model\":\"scan_context\"},\"database\":{"
    "\"descriptor_vector_dim\":20,\"distance_threshold\":0.15,"
    "\"num_candidates\":3,\"rebuild_threshold\":50},\"alignment\":{"
    "\"pcm_translation_threshold\":10.0,"
    "\"pcm_rotation_threshold_deg\":20.0,"
    "\"pcm_solver\":\"heuristic\",\"pcm_threads\":1,"
    "\"pcm_max_candidates\":0}}")
  file(WRITE "${config_root}/core/optimizer.json"
    "{\"backend_optimizer\":{"
    "\"backend_optimizer_type\":\"incremental\","
    "\"relinearizeThreshold\":0.1,\"relinearizeSkip\":1,"
    "\"isam_extra_updates\":1,\"min_loop_frame_gap\":30,"
    "\"icp_search_num\":1}}")
  file(WRITE "${config_root}/core/remover.json"
    "{\"dynamic_remover\":{\"dynamic_remover_type\":\"offline\","
    "\"model\":\"free_dom\"}}")
  set(OPEN_LMM_WRITTEN_CONFIG_ROOT "${config_root}" PARENT_SCOPE)
  set(OPEN_LMM_WRITTEN_DATA_ROOT "${data_root}" PARENT_SCOPE)
endfunction()

function(run_batch)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env
            --unset=LD_LIBRARY_PATH --unset=PYTHONPATH
            "${OPEN_LMM_BATCH_EXECUTABLE}" ${ARGN}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE stdout
    ERROR_VARIABLE stderr
    TIMEOUT 90)
  set(OPEN_LMM_RUN_RESULT "${result}" PARENT_SCOPE)
  set(OPEN_LMM_RUN_STDOUT "${stdout}" PARENT_SCOPE)
  set(OPEN_LMM_RUN_STDERR "${stderr}" PARENT_SCOPE)
endfunction()

file(REMOVE_RECURSE "${OPEN_LMM_CLI_TEST_ROOT}")
file(MAKE_DIRECTORY "${OPEN_LMM_CLI_TEST_ROOT}")
set(expected_usage
  "Usage: ${OPEN_LMM_BATCH_EXECUTABLE} <config_dir_path>\n")

if(OPEN_LMM_CLI_CASE STREQUAL "help")
  run_batch(--help)
  if(NOT OPEN_LMM_RUN_RESULT EQUAL 0 OR
     NOT OPEN_LMM_RUN_STDOUT STREQUAL expected_usage OR
     NOT OPEN_LMM_RUN_STDERR STREQUAL "")
    fail_contract("help output changed" "${OPEN_LMM_RUN_RESULT}"
      "${OPEN_LMM_RUN_STDOUT}" "${OPEN_LMM_RUN_STDERR}")
  endif()
elseif(OPEN_LMM_CLI_CASE STREQUAL "invalid_noarg")
  run_batch()
  if(NOT OPEN_LMM_RUN_RESULT EQUAL 1 OR
     NOT OPEN_LMM_RUN_STDOUT STREQUAL "" OR
     NOT OPEN_LMM_RUN_STDERR STREQUAL expected_usage)
    fail_contract("no-argument output changed" "${OPEN_LMM_RUN_RESULT}"
      "${OPEN_LMM_RUN_STDOUT}" "${OPEN_LMM_RUN_STDERR}")
  endif()
elseif(OPEN_LMM_CLI_CASE STREQUAL "invalid_many")
  run_batch(first second)
  if(NOT OPEN_LMM_RUN_RESULT EQUAL 1 OR
     NOT OPEN_LMM_RUN_STDOUT STREQUAL "" OR
     NOT OPEN_LMM_RUN_STDERR STREQUAL expected_usage)
    fail_contract("too-many-arguments output changed" "${OPEN_LMM_RUN_RESULT}"
      "${OPEN_LMM_RUN_STDOUT}" "${OPEN_LMM_RUN_STDERR}")
  endif()
elseif(OPEN_LMM_CLI_CASE STREQUAL "bootstrap_failure")
  set(missing_config "${OPEN_LMM_CLI_TEST_ROOT}/missing-config")
  file(MAKE_DIRECTORY "${missing_config}")
  run_batch("${missing_config}")
  set(expected_error "File not found: ${missing_config}/config.json\n")
  string(REGEX REPLACE
    "\\[[0-9-]+ [0-9:.]+\\] \\[error\\] " ""
    normalized_stderr "${OPEN_LMM_RUN_STDERR}")
  if(NOT OPEN_LMM_RUN_RESULT EQUAL 1 OR
     NOT OPEN_LMM_RUN_STDOUT STREQUAL "" OR
     NOT normalized_stderr STREQUAL expected_error)
    fail_contract("bootstrap failure output changed" "${OPEN_LMM_RUN_RESULT}"
      "${OPEN_LMM_RUN_STDOUT}" "${OPEN_LMM_RUN_STDERR}")
  endif()
elseif(OPEN_LMM_CLI_CASE STREQUAL "run_failure")
  write_valid_fixture("${OPEN_LMM_CLI_TEST_ROOT}/fixture")
  file(WRITE
    "${OPEN_LMM_WRITTEN_DATA_ROOT}/agent1/Scans/000000.pcd"
    "deterministic malformed PCD fixture\n")
  run_batch("${OPEN_LMM_WRITTEN_CONFIG_ROOT}")
  if(NOT OPEN_LMM_RUN_RESULT EQUAL 1 OR
     OPEN_LMM_RUN_STDOUT MATCHES "(^|\\n).*\\[error\\]" OR
     OPEN_LMM_RUN_STDOUT MATCHES "I/O error:|failed to load PCD" OR
     OPEN_LMM_RUN_STDERR STREQUAL "")
    fail_contract("runtime failure output changed" "${OPEN_LMM_RUN_RESULT}"
      "${OPEN_LMM_RUN_STDOUT}" "${OPEN_LMM_RUN_STDERR}")
  endif()
  string(CONCAT expected_failure
    "I/O error: agent agent1, frame 0, scan "
    "${OPEN_LMM_WRITTEN_DATA_ROOT}/agent1/Scans/000000.pcd: "
    "failed to load PCD file: "
    "${OPEN_LMM_WRITTEN_DATA_ROOT}/agent1/Scans/000000.pcd")
  foreach(required_stderr IN ITEMS
      "[agent1] enumerate input scans"
      "[agent1] read and filter scans"
      "[pcl::PCDReader::readHeader] No points to read"
      "${expected_failure}")
    string(FIND "${OPEN_LMM_RUN_STDERR}" "${required_stderr}" found)
    if(found EQUAL -1)
      fail_contract("runtime failure stderr lost '${required_stderr}'"
        "${OPEN_LMM_RUN_RESULT}" "${OPEN_LMM_RUN_STDOUT}"
        "${OPEN_LMM_RUN_STDERR}")
    endif()
  endforeach()
  string(REGEX MATCHALL "I/O error:" failure_payloads
    "${OPEN_LMM_RUN_STDERR}")
  list(LENGTH failure_payloads failure_payload_count)
  if(NOT failure_payload_count EQUAL 1)
    fail_contract("runtime failure payload must occur exactly once"
      "${OPEN_LMM_RUN_RESULT}" "${OPEN_LMM_RUN_STDOUT}"
      "${OPEN_LMM_RUN_STDERR}")
  endif()
  assert_terminal_newline("stderr" "${OPEN_LMM_RUN_STDERR}")
elseif(OPEN_LMM_CLI_CASE STREQUAL "success")
  write_valid_fixture("${OPEN_LMM_CLI_TEST_ROOT}/fixture")
  run_batch("${OPEN_LMM_WRITTEN_CONFIG_ROOT}")
  if(NOT OPEN_LMM_RUN_RESULT EQUAL 0)
    fail_contract("fixture execution failed" "${OPEN_LMM_RUN_RESULT}"
      "${OPEN_LMM_RUN_STDOUT}" "${OPEN_LMM_RUN_STDERR}")
  endif()
  foreach(required_stdout IN ITEMS
      "[plugin ABI v1] same-toolchain compatibility mode: libcreate_scan_context.so"
      "[plugin ABI v1] same-toolchain compatibility mode: libcreate_free_dom.so"
      "[PROFILE] agent=agent1 module=LoopDetect"
      "[PROFILE] agent=agent1 module=Optimize")
    string(FIND "${OPEN_LMM_RUN_STDOUT}" "${required_stdout}" found)
    if(found EQUAL -1)
      fail_contract("success stdout lost '${required_stdout}'"
        "${OPEN_LMM_RUN_RESULT}" "${OPEN_LMM_RUN_STDOUT}"
        "${OPEN_LMM_RUN_STDERR}")
    endif()
  endforeach()
  foreach(required_stderr IN ITEMS
      "[agent1] enumerate input scans"
      "[agent1] detect loops"
      "[agent1] optimize graph")
    string(FIND "${OPEN_LMM_RUN_STDERR}" "${required_stderr}" found)
    if(found EQUAL -1)
      fail_contract("success stderr lost '${required_stderr}'"
        "${OPEN_LMM_RUN_RESULT}" "${OPEN_LMM_RUN_STDOUT}"
        "${OPEN_LMM_RUN_STDERR}")
    endif()
  endforeach()
  if(OPEN_LMM_RUN_STDOUT MATCHES "(^|\\n).*\\[error\\]" OR
     OPEN_LMM_RUN_STDERR MATCHES "(^|\\n).*\\[error\\]")
    fail_contract("success output contains an error payload"
      "${OPEN_LMM_RUN_RESULT}" "${OPEN_LMM_RUN_STDOUT}"
      "${OPEN_LMM_RUN_STDERR}")
  endif()
  assert_terminal_newline("stderr" "${OPEN_LMM_RUN_STDERR}")
else()
  message(FATAL_ERROR "unsupported OPEN_LMM_CLI_CASE: ${OPEN_LMM_CLI_CASE}")
endif()
