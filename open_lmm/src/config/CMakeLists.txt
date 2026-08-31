find_package(spdlog REQUIRED)

add_library(open_lmm_config_document_objects OBJECT
  document/config.cpp)
target_link_libraries(open_lmm_config_document_objects PRIVATE
  Eigen3::Eigen
  nlohmann_json::nlohmann_json)
target_include_directories(open_lmm_config_document_objects PRIVATE
  $<BUILD_INTERFACE:${nlohmann_json_SOURCE_DIR}/include>)
openlmm_set_global_target_properties(open_lmm_config_document_objects)

add_library(open_lmm_config_schema_objects OBJECT
  schema/config_schema.cpp
  schema/config_schema_catalog.cpp)
target_link_libraries(open_lmm_config_schema_objects PRIVATE
  Eigen3::Eigen
  nlohmann_json::nlohmann_json)
target_include_directories(open_lmm_config_schema_objects PRIVATE
  $<BUILD_INTERFACE:${nlohmann_json_SOURCE_DIR}/include>)
openlmm_set_global_target_properties(open_lmm_config_schema_objects)

# Stable compatibility DSO for foundation/config utilities.  Its public target
# name and exported symbols remain unchanged while implementation ownership is
# now explicit below src/.
add_library(open_lmm_utils SHARED
  $<TARGET_OBJECTS:open_lmm_foundation_concurrency_objects>
  $<TARGET_OBJECTS:open_lmm_foundation_logging_objects>
  $<TARGET_OBJECTS:open_lmm_config_document_objects>
  $<TARGET_OBJECTS:open_lmm_config_schema_objects>)
target_link_libraries(open_lmm_utils
 PUBLIC
  Eigen3::Eigen
 PRIVATE
  nlohmann_json::nlohmann_json
  spdlog::spdlog)
target_include_directories(open_lmm_utils PUBLIC
  $<BUILD_INTERFACE:${nlohmann_json_SOURCE_DIR}/include>)
openlmm_set_global_target_properties(open_lmm_utils)

add_library(open_lmm_config_domain_objects OBJECT
  domain/algorithm_config.cpp)
target_link_libraries(open_lmm_config_domain_objects PRIVATE
  open_lmm_common
  open_lmm_utils
  Eigen3::Eigen)
target_compile_definitions(open_lmm_config_domain_objects PRIVATE
  OPEN_LMM_HAVE_DESCRIPTOR_SCAN_CONTEXT=$<BOOL:${OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT}>
  OPEN_LMM_HAVE_DESCRIPTOR_SOLID=$<BOOL:${OPEN_LMM_BUILD_DESCRIPTOR_SOLID}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_HMM_MOS=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_DUFOMAP=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_OTD=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_FREE_DOM=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_ERASOR=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR}>)
openlmm_set_global_target_properties(open_lmm_config_domain_objects)

add_library(open_lmm_algorithm_config SHARED
  $<TARGET_OBJECTS:open_lmm_config_domain_objects>)
target_link_libraries(open_lmm_algorithm_config PUBLIC
  open_lmm_common
  open_lmm_utils
  Eigen3::Eigen)
openlmm_set_global_target_properties(open_lmm_algorithm_config)
set_target_properties(open_lmm_algorithm_config PROPERTIES EXPORT_NAME config)

add_library(open_lmm_config_application_objects OBJECT
  bootstrap/bootstrap_config.cpp
  application/runtime_reconfigurer.cpp)
target_link_libraries(open_lmm_config_application_objects PRIVATE
  Eigen3::Eigen
  PCL::PCL
  gtsam
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_profiling
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_config_application_objects)
