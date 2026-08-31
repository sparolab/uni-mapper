function(openlmm_add_module_compile_contract)
  set(one_value_args NAME OWNER_TARGET)
  set(multi_value_args ALLOWED_DIRS LINK_TARGETS)
  cmake_parse_arguments(CONTRACT "" "${one_value_args}" "${multi_value_args}"
    ${ARGN})
  if(NOT CONTRACT_NAME OR NOT CONTRACT_OWNER_TARGET OR
     NOT CONTRACT_ALLOWED_DIRS)
    message(FATAL_ERROR
      "module compile contract requires NAME, OWNER_TARGET, and ALLOWED_DIRS")
  endif()
  if(NOT TARGET "${CONTRACT_OWNER_TARGET}")
    message(FATAL_ERROR
      "module compile owner target does not exist: ${CONTRACT_OWNER_TARGET}")
  endif()

  set(isolated_include_root
    "${CMAKE_CURRENT_BINARY_DIR}/include/${CONTRACT_NAME}")
  file(REMOVE_RECURSE "${isolated_include_root}")
  foreach(allowed_dir IN LISTS CONTRACT_ALLOWED_DIRS)
    get_filename_component(allowed_parent "${allowed_dir}" DIRECTORY)
    file(MAKE_DIRECTORY "${isolated_include_root}/${allowed_parent}")
    file(COPY "${PROJECT_SOURCE_DIR}/src/${allowed_dir}"
      DESTINATION "${isolated_include_root}/${allowed_parent}"
      FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
  endforeach()

  add_library(${CONTRACT_NAME} OBJECT ${CONTRACT_NAME}.cpp)
  target_compile_features(${CONTRACT_NAME} PRIVATE cxx_std_20)
  target_include_directories(${CONTRACT_NAME} PRIVATE
    "${isolated_include_root}"
    "${PROJECT_SOURCE_DIR}/include")
  target_link_libraries(${CONTRACT_NAME} PRIVATE ${CONTRACT_LINK_TARGETS})
  add_dependencies(${CONTRACT_NAME} ${CONTRACT_OWNER_TARGET})
  set_property(GLOBAL APPEND PROPERTY OPEN_LMM_MODULE_COMPILE_TARGETS
    ${CONTRACT_NAME})
endfunction()

openlmm_add_module_compile_contract(
  NAME runtime_model_contract_compile
  OWNER_TARGET open_lmm_runtime_model_objects
  ALLOWED_DIRS runtime/model
  LINK_TARGETS open_lmm_contracts)

openlmm_add_module_compile_contract(
  NAME runtime_state_contract_compile
  OWNER_TARGET open_lmm_runtime_state_objects
  ALLOWED_DIRS runtime/model runtime/state runtime/resources
    foundation/concurrency config/domain domain/optimization
  LINK_TARGETS open_lmm_common open_lmm_algorithm_config open_lmm_utils
    Eigen3::Eigen PCL::PCL gtsam)

openlmm_add_module_compile_contract(
  NAME runtime_resources_contract_compile
  OWNER_TARGET open_lmm_runtime_resources_objects
  ALLOWED_DIRS runtime/resources foundation/concurrency
  LINK_TARGETS open_lmm_contracts)

openlmm_add_module_compile_contract(
  NAME storage_contract_compile
  OWNER_TARGET open_lmm_storage_objects
  ALLOWED_DIRS storage
  LINK_TARGETS open_lmm_contracts)

set(execution_allowed_dirs
  runtime/model runtime/state runtime/resources runtime/execution
  storage config domain plugins/host foundation/concurrency foundation/logging
  visualization)
set(runtime_compile_links
  open_lmm_common open_lmm_algorithm_config open_lmm_utils
  Eigen3::Eigen PCL::PCL gtsam nlohmann_json::nlohmann_json)

openlmm_add_module_compile_contract(
  NAME runtime_execution_contract_compile
  OWNER_TARGET open_lmm_runtime_execution_objects
  ALLOWED_DIRS ${execution_allowed_dirs}
  LINK_TARGETS ${runtime_compile_links})

openlmm_add_module_compile_contract(
  NAME runtime_control_contract_compile
  OWNER_TARGET open_lmm_runtime_control_objects
  ALLOWED_DIRS ${execution_allowed_dirs} runtime/control
  LINK_TARGETS ${runtime_compile_links})

openlmm_add_module_compile_contract(
  NAME runtime_service_contract_compile
  OWNER_TARGET open_lmm_runtime_service_objects
  ALLOWED_DIRS ${execution_allowed_dirs} runtime/control runtime/service
  LINK_TARGETS ${runtime_compile_links})

openlmm_add_module_compile_contract(
  NAME runtime_composition_contract_compile
  OWNER_TARGET open_lmm_runtime_composition_objects
  ALLOWED_DIRS ${execution_allowed_dirs} runtime/control runtime/service
    runtime/composition
  LINK_TARGETS ${runtime_compile_links})

openlmm_add_module_compile_contract(
  NAME runtime_client_contract_compile
  OWNER_TARGET open_lmm_runtime_client_objects
  ALLOWED_DIRS runtime/client
  LINK_TARGETS open_lmm_contracts Eigen3::Eigen)

openlmm_add_module_compile_contract(
  NAME config_document_contract_compile
  OWNER_TARGET open_lmm_config_document_objects
  ALLOWED_DIRS config/document foundation/logging
  LINK_TARGETS open_lmm_contracts Eigen3::Eigen nlohmann_json::nlohmann_json)

openlmm_add_module_compile_contract(
  NAME config_schema_contract_compile
  OWNER_TARGET open_lmm_config_schema_objects
  ALLOWED_DIRS config/schema
  LINK_TARGETS open_lmm_utils)

openlmm_add_module_compile_contract(
  NAME config_domain_contract_compile
  OWNER_TARGET open_lmm_config_domain_objects
  ALLOWED_DIRS config/domain
  LINK_TARGETS open_lmm_common open_lmm_utils Eigen3::Eigen)

openlmm_add_module_compile_contract(
  NAME config_application_contract_compile
  OWNER_TARGET open_lmm_config_application_objects
  ALLOWED_DIRS config plugins/host runtime/model runtime/state
    runtime/resources foundation/concurrency foundation/logging
    domain/optimization
  LINK_TARGETS ${runtime_compile_links})

openlmm_add_module_compile_contract(
  NAME domain_no_runtime_contract_compile
  OWNER_TARGET open_lmm_data_loader
  ALLOWED_DIRS domain config/document config/domain foundation/logging
  LINK_TARGETS open_lmm_common open_lmm_utils Eigen3::Eigen PCL::PCL gtsam)

openlmm_add_module_compile_contract(
  NAME plugin_host_direction_contract_compile
  OWNER_TARGET open_lmm_plugin_host_objects
  ALLOWED_DIRS plugins/host config/domain config/document foundation/logging
  LINK_TARGETS open_lmm_common open_lmm_algorithm_config open_lmm_utils
    Eigen3::Eigen)

openlmm_add_module_compile_contract(
  NAME visualization_contract_compile
  OWNER_TARGET open_lmm_visualization_projection_objects
  ALLOWED_DIRS visualization
  LINK_TARGETS open_lmm_common Eigen3::Eigen PCL::PCL gtsam)

openlmm_add_module_compile_contract(
  NAME adapter_public_leaf_contract_compile
  OWNER_TARGET open_lmm_client
  ALLOWED_DIRS runtime/client
  LINK_TARGETS open_lmm_client)

get_property(module_compile_targets GLOBAL PROPERTY
  OPEN_LMM_MODULE_COMPILE_TARGETS)
add_custom_target(open_lmm_module_compile_contracts ALL
  DEPENDS ${module_compile_targets})
