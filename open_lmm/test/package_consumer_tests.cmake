foreach(required_variable
    OPEN_LMM_BUILD_DIR OPEN_LMM_SOURCE_DIR OPEN_LMM_PACKAGE_TEST_ROOT)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

set(install_prefix "${OPEN_LMM_PACKAGE_TEST_ROOT}/install")
set(consumer_source "${OPEN_LMM_PACKAGE_TEST_ROOT}/consumer-source")
set(consumer_build "${OPEN_LMM_PACKAGE_TEST_ROOT}/consumer-build")
set(consumer_compiler_args)
if(DEFINED OPEN_LMM_CONSUMER_C_COMPILER)
  list(APPEND consumer_compiler_args
    "-DCMAKE_C_COMPILER=${OPEN_LMM_CONSUMER_C_COMPILER}")
endif()
if(DEFINED OPEN_LMM_CONSUMER_CXX_COMPILER)
  list(APPEND consumer_compiler_args
    "-DCMAKE_CXX_COMPILER=${OPEN_LMM_CONSUMER_CXX_COMPILER}")
endif()
if(DEFINED OPEN_LMM_CONSUMER_CXX_FLAGS)
  list(APPEND consumer_compiler_args
    "-DCMAKE_CXX_FLAGS=${OPEN_LMM_CONSUMER_CXX_FLAGS}")
endif()
file(REMOVE_RECURSE "${OPEN_LMM_PACKAGE_TEST_ROOT}")
file(MAKE_DIRECTORY "${consumer_source}")
file(COPY "${OPEN_LMM_SOURCE_DIR}/test/package_consumer/"
     DESTINATION "${consumer_source}")

# Model an upgrade from the former recursive header installation. The known
# package-owned header must disappear, while an unknown user file must survive.
file(MAKE_DIRECTORY
  "${install_prefix}/include/open_lmm/server"
  "${install_prefix}/include/open_lmm/core/loop_detector")
file(WRITE "${install_prefix}/include/open_lmm/server/stage_executor.hpp"
  "// stale package-owned header\n")
file(WRITE "${install_prefix}/include/open_lmm/user-owned.hpp"
  "// must survive package cleanup\n")

execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${OPEN_LMM_BUILD_DIR}"
          --prefix "${install_prefix}"
  RESULT_VARIABLE install_result)
if(NOT install_result EQUAL 0)
  message(FATAL_ERROR "open_lmm package install failed: ${install_result}")
endif()

set(targets_file
  "${install_prefix}/share/open_lmm/cmake/open_lmmTargets.cmake")
if(NOT EXISTS "${targets_file}")
  message(FATAL_ERROR "installed target export is missing: ${targets_file}")
endif()
if(EXISTS "${install_prefix}/include/open_lmm/server/stage_executor.hpp")
  message(FATAL_ERROR "upgrade left a stale package-owned public header")
endif()
if(NOT EXISTS "${install_prefix}/include/open_lmm/user-owned.hpp")
  message(FATAL_ERROR "upgrade cleanup removed an unknown user file")
endif()
file(REMOVE "${install_prefix}/include/open_lmm/user-owned.hpp")

# The installed SDK surface is a reviewed golden list, not a recursive copy of
# source headers.  Any addition requires an intentional package API decision.
file(GLOB_RECURSE installed_headers
  RELATIVE "${install_prefix}/include/open_lmm"
  "${install_prefix}/include/open_lmm/*.h"
  "${install_prefix}/include/open_lmm/*.hpp")
list(SORT installed_headers)
file(STRINGS
  "${consumer_source}/public_header_allowlist.txt" expected_headers)
list(SORT expected_headers)
if(NOT installed_headers STREQUAL expected_headers)
  message(FATAL_ERROR
    "installed public headers differ from golden allowlist\n"
    "expected: ${expected_headers}\nactual: ${installed_headers}")
endif()
set(batch_launcher "${install_prefix}/bin/open_lmm_batch")
if(NOT EXISTS "${batch_launcher}")
  message(FATAL_ERROR "installed batch launcher is missing: ${batch_launcher}")
endif()

foreach(release_file IN ITEMS
    LICENCE RELEASE_POLICY.md THIRD_PARTY_NOTICES.md
    open_lmm-install-components.txt)
  if(NOT EXISTS "${install_prefix}/share/open_lmm/${release_file}")
    message(FATAL_ERROR
      "installed release metadata is missing: ${release_file}")
  endif()
endforeach()
file(READ
  "${install_prefix}/share/open_lmm/open_lmm-install-components.txt"
  install_components)
foreach(component IN ITEMS Runtime Development PluginSDK Plugins Tools)
  string(FIND "${install_components}" "${component}" component_found)
  if(component_found EQUAL -1)
    message(FATAL_ERROR
      "installed component manifest is missing: ${component}")
  endif()
endforeach()
foreach(third_party_license IN ITEMS
    thirdparty/eigen/LICENSE
    thirdparty/gtsam/LICENSE
    thirdparty/kiss_matcher/LICENSE
    thirdparty/nlohmann_json/LICENSE
    erasor/LICENCE)
  if(NOT EXISTS
      "${install_prefix}/share/open_lmm/licenses/${third_party_license}")
    message(FATAL_ERROR
      "installed third-party license is missing: ${third_party_license}")
  endif()
endforeach()

set(versioned_runtime_libraries
  open_lmm_contracts
  open_lmm_client
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_data_loader
  open_lmm_descriptor
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
  open_lmm_map_server
  open_lmm_gui_core)
find_program(OPEN_LMM_READELF readelf REQUIRED)
foreach(library IN LISTS versioned_runtime_libraries)
  set(library_base "lib${library}.so")
  set(library_major "${install_prefix}/lib/${library_base}.2")
  set(library_full "${install_prefix}/lib/${library_base}.2.0.0")
  if(NOT EXISTS "${library_major}" OR NOT EXISTS "${library_full}")
    message(FATAL_ERROR
      "versioned runtime library is missing: ${library_base}")
  endif()
  execute_process(
    COMMAND "${OPEN_LMM_READELF}" -d "${library_full}"
    RESULT_VARIABLE readelf_result
    OUTPUT_VARIABLE dynamic_section)
  if(NOT readelf_result EQUAL 0 OR
     NOT dynamic_section MATCHES "SONAME.*\\[${library_base}\\.2\\]")
    message(FATAL_ERROR
      "runtime library has an unexpected SONAME: ${library_full}")
  endif()
endforeach()

if(NOT EXISTS "${install_prefix}/lib/libopen_lmm_alignment.a")
  message(FATAL_ERROR "installed alignment development archive is missing")
endif()

foreach(plugin IN ITEMS create_scan_context create_free_dom)
  if(NOT EXISTS "${install_prefix}/lib/lib${plugin}.so.2.0.0")
    message(FATAL_ERROR "versioned plugin entry is missing: ${plugin}")
  endif()
endforeach()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH
          "${batch_launcher}" --help
  RESULT_VARIABLE batch_help_result)
if(NOT batch_help_result EQUAL 0)
  message(FATAL_ERROR
    "installed batch launcher could not run: ${batch_help_result}")
endif()
file(READ "${targets_file}" targets_contents)
string(FIND "${targets_contents}" "${OPEN_LMM_SOURCE_DIR}" source_reference)
if(NOT source_reference EQUAL -1)
  message(FATAL_ERROR "installed targets retain a source-tree reference")
endif()
string(REGEX MATCHALL
  "add_library\\(open_lmm::[A-Za-z0-9_]+" exported_target_declarations
  "${targets_contents}")
set(exported_targets)
foreach(declaration IN LISTS exported_target_declarations)
  string(REPLACE "add_library(open_lmm::" "" target_name "${declaration}")
  list(APPEND exported_targets "${target_name}")
endforeach()
list(SORT exported_targets)
file(STRINGS "${consumer_source}/exported_target_allowlist.txt"
  expected_exported_targets)
list(SORT expected_exported_targets)
if(NOT exported_targets STREQUAL expected_exported_targets)
  message(FATAL_ERROR
    "exported target surface differs from golden allowlist\n"
    "expected: ${expected_exported_targets}\nactual: ${exported_targets}")
endif()

# Every installed OpenLMM/plugin DSO must resolve from the install prefix and
# use the relocatable local-library runpath without ambient workspace paths.
find_program(OPEN_LMM_LDD ldd REQUIRED)
# Inspect every real shared object installed by the package, including plugin
# support libraries (Map, remover implementations, and small_gicp), rather
# than only the stable facade and create_* entry points.
file(GLOB installed_dso_candidates "${install_prefix}/lib/*.so*")
set(installed_dsos)
foreach(installed_dso_candidate IN LISTS installed_dso_candidates)
  if(NOT IS_SYMLINK "${installed_dso_candidate}")
    list(APPEND installed_dsos "${installed_dso_candidate}")
  endif()
endforeach()
if(NOT installed_dsos)
  message(FATAL_ERROR "installed package contains no shared objects")
endif()
foreach(installed_dso IN LISTS installed_dsos)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH
            "${OPEN_LMM_LDD}" "${installed_dso}"
    RESULT_VARIABLE ldd_result
    OUTPUT_VARIABLE ldd_output
    ERROR_VARIABLE ldd_error)
  if(NOT ldd_result EQUAL 0 OR ldd_output MATCHES "not found")
    message(FATAL_ERROR
      "installed DSO has unresolved dependencies: ${installed_dso}\n"
      "${ldd_output}${ldd_error}")
  endif()
  execute_process(COMMAND "${OPEN_LMM_READELF}" -d "${installed_dso}"
    RESULT_VARIABLE dso_readelf_result OUTPUT_VARIABLE dso_dynamic_section)
  if(NOT dso_readelf_result EQUAL 0 OR
     NOT dso_dynamic_section MATCHES "(RPATH|RUNPATH).*\\[\\$ORIGIN\\]")
    message(FATAL_ERROR
      "installed DSO lacks exact relocatable $ORIGIN runpath: ${installed_dso}")
  endif()
endforeach()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -S "${consumer_source}" -B "${consumer_build}"
          "-DCMAKE_PREFIX_PATH=${install_prefix}"
          ${consumer_compiler_args}
  RESULT_VARIABLE configure_result)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR "package consumer configure failed: ${configure_result}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${consumer_build}" --parallel 1
  RESULT_VARIABLE build_result)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR "package consumer build failed: ${build_result}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH
          "${consumer_build}/open_lmm_package_consumer"
  RESULT_VARIABLE run_result)
if(NOT run_result EQUAL 0)
  message(FATAL_ERROR "package consumer run failed: ${run_result}")
endif()
foreach(light_consumer IN ITEMS
    open_lmm_contracts_consumer open_lmm_client_consumer
    open_lmm_plugin_sdk_consumer)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH
            "${consumer_build}/${light_consumer}"
    RESULT_VARIABLE light_run_result)
  if(NOT light_run_result EQUAL 0)
    message(FATAL_ERROR
      "installed ${light_consumer} could not run: ${light_run_result}")
  endif()
endforeach()

set(header_build "${OPEN_LMM_PACKAGE_TEST_ROOT}/header-self-containment-build")
execute_process(
  COMMAND "${CMAKE_COMMAND}"
          -S "${consumer_source}/header_self_containment"
          -B "${header_build}"
          "-DCMAKE_PREFIX_PATH=${install_prefix}"
          ${consumer_compiler_args}
  RESULT_VARIABLE header_configure_result)
if(NOT header_configure_result EQUAL 0)
  message(FATAL_ERROR
    "public-header self-containment configure failed: ${header_configure_result}")
endif()
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${header_build}" --parallel 1
  RESULT_VARIABLE header_build_result)
if(NOT header_build_result EQUAL 0)
  message(FATAL_ERROR
    "public-header self-containment build failed: ${header_build_result}")
endif()

# Configure each lightweight surface independently. This catches accidental
# package-config discovery of Eigen/PCL/GTSAM for contracts/client/plugin SDK.
foreach(component IN ITEMS contracts client plugin_sdk)
  set(component_build "${OPEN_LMM_PACKAGE_TEST_ROOT}/${component}-only-build")
  set(lightweight_dependency_guards)
  if(component STREQUAL "contracts" OR component STREQUAL "plugin_sdk")
    list(APPEND lightweight_dependency_guards
      -DCMAKE_DISABLE_FIND_PACKAGE_Eigen3=TRUE
      -DCMAKE_DISABLE_FIND_PACKAGE_PCL=TRUE
      -DCMAKE_DISABLE_FIND_PACKAGE_GTSAM=TRUE)
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}"
            -S "${consumer_source}/${component}_only"
            -B "${component_build}"
            "-DCMAKE_PREFIX_PATH=${install_prefix}"
            ${lightweight_dependency_guards}
            ${consumer_compiler_args}
    RESULT_VARIABLE component_configure_result)
  if(NOT component_configure_result EQUAL 0)
    message(FATAL_ERROR
      "${component}-only consumer configure failed: ${component_configure_result}")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${component_build}" --parallel 1
    RESULT_VARIABLE component_build_result)
  if(NOT component_build_result EQUAL 0)
    message(FATAL_ERROR
      "${component}-only consumer build failed: ${component_build_result}")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH
            "${component_build}/${component}_only"
    RESULT_VARIABLE component_run_result)
  if(NOT component_run_result EQUAL 0)
    message(FATAL_ERROR
      "${component}-only consumer run failed: ${component_run_result}")
  endif()
endforeach()
