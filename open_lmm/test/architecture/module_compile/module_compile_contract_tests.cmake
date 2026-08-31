cmake_minimum_required(VERSION 3.20)

foreach(required_variable
    OPEN_LMM_BUILD_DIR OPEN_LMM_PROJECT_SOURCE_DIR
    OPEN_LMM_MODULE_COMPILE_SOURCE_DIR)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

set(compile_database "${OPEN_LMM_BUILD_DIR}/compile_commands.json")
if(NOT EXISTS "${compile_database}")
  message(FATAL_ERROR "compile database is missing: ${compile_database}")
endif()
file(READ "${compile_database}" compile_commands)
string(JSON compile_entry_count LENGTH "${compile_commands}")

set(contract_names
  runtime_model_contract_compile
  runtime_state_contract_compile
  runtime_resources_contract_compile
  storage_contract_compile
  runtime_execution_contract_compile
  runtime_control_contract_compile
  runtime_service_contract_compile
  runtime_composition_contract_compile
  runtime_client_contract_compile
  config_document_contract_compile
  config_schema_contract_compile
  config_domain_contract_compile
  config_application_contract_compile
  domain_no_runtime_contract_compile
  plugin_host_direction_contract_compile
  visualization_contract_compile
  adapter_public_leaf_contract_compile)

foreach(contract_name IN LISTS contract_names)
  set(source_path
    "${OPEN_LMM_MODULE_COMPILE_SOURCE_DIR}/${contract_name}.cpp")
  set(source_count 0)
  math(EXPR last_compile_entry "${compile_entry_count} - 1")
  foreach(entry_index RANGE 0 ${last_compile_entry})
    string(JSON entry_source GET "${compile_commands}" ${entry_index} file)
    if(entry_source STREQUAL source_path)
      math(EXPR source_count "${source_count} + 1")
      string(JSON entry_command GET
        "${compile_commands}" ${entry_index} command)
      string(FIND "${entry_command}" "-I${OPEN_LMM_PROJECT_SOURCE_DIR}/src "
        broad_source_include)
      if(NOT broad_source_include EQUAL -1)
        message(FATAL_ERROR
          "${contract_name} received the broad production src include")
      endif()
    endif()
  endforeach()
  if(NOT source_count EQUAL 1)
    message(FATAL_ERROR
      "compile contract ${contract_name} occurs ${source_count} times in "
      "compile_commands.json")
  endif()
endforeach()

message(STATUS "validated 17 isolated logical-module compile contracts")
