# A sourced, older install prefix may appear in LD_LIBRARY_PATH ahead of the
# build-tree RUNPATH. Unit tests must exercise the libraries just compiled.
set(_open_lmm_build_library_path
  "$<TARGET_FILE_DIR:open_lmm_batch>:"
  "$<TARGET_FILE_DIR:open_lmm_map_server>:"
  "$<TARGET_FILE_DIR:open_lmm_data_loader>:"
  "$<TARGET_FILE_DIR:open_lmm_loop_detector>:"
  "$<TARGET_FILE_DIR:open_lmm_backend_optimizer>:"
  "$<TARGET_FILE_DIR:open_lmm_dynamic_remover>:"
  "$<TARGET_FILE_DIR:open_lmm_algorithm_config>:"
  "$<TARGET_FILE_DIR:open_lmm_gui_core>:"
  "$<TARGET_FILE_DIR:open_lmm_contracts>:"
  "$<TARGET_FILE_DIR:open_lmm_client>:"
  "$<TARGET_FILE_DIR:open_lmm_common>:"
  "$<TARGET_FILE_DIR:open_lmm_descriptor>:"
  "$<TARGET_FILE_DIR:open_lmm_utils>:")
foreach(plugin_support_target IN ITEMS
    scan_context solid hmm_mos dufomap otd free_dom erasor)
  if(TARGET ${plugin_support_target})
    list(APPEND _open_lmm_build_library_path
      "$<TARGET_FILE_DIR:${plugin_support_target}>:")
  endif()
endforeach()
if(NOT OPEN_LMM_ENABLE_ASAN_UBSAN AND NOT OPEN_LMM_ENABLE_TSAN)
  list(APPEND _open_lmm_build_library_path "$ENV{LD_LIBRARY_PATH}")
endif()
string(JOIN "" _open_lmm_build_library_path ${_open_lmm_build_library_path})
openlmm_apply_test_environment(
  "LD_LIBRARY_PATH=${_open_lmm_build_library_path}")

openlmm_add_test(
  NAME open_lmm_architecture_boundary_tests
  COMMAND ${CMAKE_COMMAND}
  LAYER L2 MODULE architecture.policy OWNER ArchitecturePolicy
  INVARIANTS INV-01 INV-08 INV-09 INV-10 INV-11 INV-12 INV-13 INV-14 INV-15
  LANES pr
  COMMAND_ARGS
    -DOPEN_LMM_SOURCE_DIR=${PROJECT_SOURCE_DIR}
    -DOPEN_LMM_BUILD_DIR=${PROJECT_BINARY_DIR}
    -DOPEN_LMM_REQUIRE_COMPLETE_COMPILE_OWNERS=$<BOOL:${OPEN_LMM_BUILD_IRIDESCENCE_GUI}>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/architecture/policy/architecture_boundary_tests.cmake)

openlmm_add_test(
  NAME open_lmm_release_policy_tests
  COMMAND ${CMAKE_COMMAND}
  LAYER L2 MODULE release.policy OWNER ReleasePolicy
  INVARIANTS INV-14 INV-15 LANES pr
  COMMAND_ARGS
    -DOPEN_LMM_REPOSITORY_ROOT=${PROJECT_SOURCE_DIR}/..
    -P ${CMAKE_CURRENT_SOURCE_DIR}/release/policy/release_policy_tests.cmake)

openlmm_add_test(
  NAME open_lmm_test_manifest_tests
  COMMAND ${CMAKE_COMMAND}
  LAYER L2 MODULE architecture.test_manifest OWNER TestManifest
  INVARIANTS INV-11 INV-12 INV-13 INV-14 LANES pr
  COMMAND_ARGS
    -DOPEN_LMM_TEST_MANIFEST=${OPEN_LMM_TEST_MANIFEST}
    -DOPEN_LMM_SOURCE_DIR=${PROJECT_SOURCE_DIR}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/TestManifest.cmake)

openlmm_add_test(
  NAME open_lmm_module_compile_contract_tests
  COMMAND ${CMAKE_COMMAND}
  LAYER L2 MODULE architecture.module_compile OWNER ModuleCompileContracts
  INVARIANTS INV-11 INV-12 INV-13 INV-14 LANES pr
  COMMAND_ARGS
    -DOPEN_LMM_BUILD_DIR=${PROJECT_BINARY_DIR}
    -DOPEN_LMM_PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}
    -DOPEN_LMM_MODULE_COMPILE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/architecture/module_compile
    -P ${CMAKE_CURRENT_SOURCE_DIR}/architecture/module_compile/module_compile_contract_tests.cmake)

# Package installation mutates the selected prefix and is intentionally kept
# out of an ament build. Standalone CI runs it serially after all targets build.
if(NOT (DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" EQUAL "2"))
  openlmm_add_test(
    NAME open_lmm_package_consumer_tests
    COMMAND ${CMAKE_COMMAND}
    LAYER L3 MODULE package.api OWNER InstalledPackage
    INVARIANTS INV-13 INV-14 INV-15 LANES pr
    RUN_SERIAL TIMEOUT 300
    COMMAND_ARGS
      -DOPEN_LMM_BUILD_DIR=${PROJECT_BINARY_DIR}
      -DOPEN_LMM_SOURCE_DIR=${PROJECT_SOURCE_DIR}
      -DOPEN_LMM_PACKAGE_TEST_ROOT=${PROJECT_BINARY_DIR}/package-test
      -P ${CMAKE_CURRENT_SOURCE_DIR}/package/orchestrator/package_consumer_tests.cmake)
endif()

openlmm_finalize_test_manifest()
