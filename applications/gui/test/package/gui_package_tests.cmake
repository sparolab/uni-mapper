foreach(required_variable IN ITEMS
    OPEN_LMM_GUI_BUILD_DIR OPEN_LMM_GUI_SOURCE_DIR
    OPEN_LMM_GUI_CORE_PREFIX OPEN_LMM_GUI_PACKAGE_TEST_ROOT)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()
if(DEFINED OPEN_LMM_GUI_REPOSITORY_ROOT AND
   NOT OPEN_LMM_GUI_REPOSITORY_ROOT STREQUAL "")
  get_filename_component(open_lmm_repository_root
    "${OPEN_LMM_GUI_REPOSITORY_ROOT}" ABSOLUTE)
else()
  get_filename_component(open_lmm_repository_root
    "${OPEN_LMM_GUI_SOURCE_DIR}/../.." ABSOLUTE)
endif()
set(gui_manifest_dir
  "${OPEN_LMM_GUI_SOURCE_DIR}/test/package/manifests")
file(GLOB_RECURSE gui_physical_public_headers
  RELATIVE "${OPEN_LMM_GUI_SOURCE_DIR}/include"
  "${OPEN_LMM_GUI_SOURCE_DIR}/include/*.h"
  "${OPEN_LMM_GUI_SOURCE_DIR}/include/*.hpp")
file(STRINGS "${gui_manifest_dir}/public_header_allowlist.txt"
  gui_expected_public_headers)
list(SORT gui_physical_public_headers)
list(SORT gui_expected_public_headers)
if(NOT gui_physical_public_headers STREQUAL gui_expected_public_headers)
  message(FATAL_ERROR
    "GUI public headers differ from their canonical allowlist")
endif()
file(STRINGS "${gui_manifest_dir}/exported_target_allowlist.tsv"
  gui_exported_target_allowlist)
if(NOT gui_exported_target_allowlist STREQUAL
   "open_lmm_gui::gui\tSHARED_LIBRARY\tGuiDevelopment")
  message(FATAL_ERROR "GUI exported-target allowlist is not canonical")
endif()
file(STRINGS "${gui_manifest_dir}/compatibility_target_allowlist.tsv"
  gui_compatibility_target_allowlist)
if(NOT gui_compatibility_target_allowlist STREQUAL
   "open_lmm::gui\topen_lmm_gui::gui\t3.0.0\tdeprecated_config_alias")
  message(FATAL_ERROR "Compat-A target allowlist is not canonical")
endif()
file(STRINGS "${gui_manifest_dir}/install_component_manifest.tsv"
  gui_component_manifest)
list(LENGTH gui_component_manifest gui_component_manifest_count)
if(NOT gui_component_manifest_count EQUAL 12)
  message(FATAL_ERROR "GUI install-component manifest is incomplete")
endif()

set(gui_consumer_cmake_args)
if(DEFINED OPEN_LMM_GUI_C_COMPILER AND
   NOT OPEN_LMM_GUI_C_COMPILER STREQUAL "")
  list(APPEND gui_consumer_cmake_args
    "-DCMAKE_C_COMPILER=${OPEN_LMM_GUI_C_COMPILER}")
endif()
if(DEFINED OPEN_LMM_GUI_CXX_COMPILER AND
   NOT OPEN_LMM_GUI_CXX_COMPILER STREQUAL "")
  list(APPEND gui_consumer_cmake_args
    "-DCMAKE_CXX_COMPILER=${OPEN_LMM_GUI_CXX_COMPILER}")
endif()
if(DEFINED OPEN_LMM_GUI_CXX_FLAGS AND
   NOT OPEN_LMM_GUI_CXX_FLAGS STREQUAL "")
  list(APPEND gui_consumer_cmake_args
    "-DCMAKE_CXX_FLAGS=${OPEN_LMM_GUI_CXX_FLAGS}")
endif()

set(install_prefix "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/install")
set(gui_consumer_source
  "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/consumer-source")
file(REMOVE_RECURSE "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}")
file(MAKE_DIRECTORY "${install_prefix}" "${gui_consumer_source}")
foreach(gui_consumer IN ITEMS
    core_client_consumer core_gui_required direct_consumer compat_consumer
    unqualified_consumer version_skew)
  file(MAKE_DIRECTORY "${gui_consumer_source}/${gui_consumer}")
  file(COPY
    "${OPEN_LMM_GUI_SOURCE_DIR}/test/package/${gui_consumer}/"
    DESTINATION "${gui_consumer_source}/${gui_consumer}")
endforeach()
file(COPY "${OPEN_LMM_GUI_CORE_PREFIX}/" DESTINATION "${install_prefix}")
file(GLOB_RECURSE core_owned_paths
  LIST_DIRECTORIES FALSE RELATIVE "${install_prefix}"
  "${install_prefix}/*")

foreach(gui_owned_header IN ITEMS gui_plugin.hpp gui_runtime_host.hpp)
  if(EXISTS "${install_prefix}/include/open_lmm/gui/${gui_owned_header}")
    message(FATAL_ERROR
      "core-only artifact owns extracted GUI header: ${gui_owned_header}")
  endif()
endforeach()
if(EXISTS "${install_prefix}/lib/libopen_lmm_gui_core.so" OR
   EXISTS "${install_prefix}/share/OpenLmmGui")
  message(FATAL_ERROR "core-only artifact owns extracted GUI files")
endif()

set(core_client_build "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/core-client-build")
execute_process(
  COMMAND "${CMAKE_COMMAND}"
          -S "${gui_consumer_source}/core_client_consumer"
          -B "${core_client_build}"
          "-DCMAKE_PREFIX_PATH=${install_prefix}"
          -DCMAKE_DISABLE_FIND_PACKAGE_OpenLmmGui=TRUE
          --trace-expand --trace-format=json-v1
          ${gui_consumer_cmake_args}
  RESULT_VARIABLE core_client_result
  ERROR_VARIABLE core_client_trace)
if(NOT core_client_result EQUAL 0)
  message(FATAL_ERROR "core client-only configure failed: ${core_client_result}")
endif()
foreach(forbidden_dependency IN ITEMS Iridescence OpenGL glfw spdlog)
  string(REGEX MATCH
    "\"cmd\":\"find_(package|dependency)\"[^\n]*${forbidden_dependency}"
    forbidden_discovery "${core_client_trace}")
  if(forbidden_discovery)
    message(FATAL_ERROR
      "core client-only discovery requested GUI-local dependency: ${forbidden_dependency}")
  endif()
endforeach()

find_program(OPEN_LMM_GUI_READELF readelf REQUIRED)
file(GLOB core_shared_libraries "${install_prefix}/lib/*.so*")
foreach(core_shared_library IN LISTS core_shared_libraries)
  if(IS_DIRECTORY "${core_shared_library}")
    continue()
  endif()
  execute_process(
    COMMAND "${OPEN_LMM_GUI_READELF}" -d "${core_shared_library}"
    RESULT_VARIABLE core_readelf_result
    OUTPUT_VARIABLE core_dynamic_section ERROR_QUIET)
  if(core_readelf_result EQUAL 0 AND
     core_dynamic_section MATCHES
       "NEEDED.*(Iridescence|libOpenGL|libGLX|libGL\\.so|libglfw)")
    message(FATAL_ERROR
      "core-only artifact retains a GUI/OpenGL DT_NEEDED edge: ${core_shared_library}")
  endif()
endforeach()

set(core_gui_build "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/core-gui-build")
execute_process(
  COMMAND "${CMAKE_COMMAND}"
          -S "${gui_consumer_source}/core_gui_required"
          -B "${core_gui_build}"
          "-DCMAKE_PREFIX_PATH=${install_prefix}"
          -DCMAKE_DISABLE_FIND_PACKAGE_OpenLmmGui=TRUE
          ${gui_consumer_cmake_args}
  RESULT_VARIABLE core_gui_result
  OUTPUT_QUIET ERROR_QUIET)
if(core_gui_result EQUAL 0)
  message(FATAL_ERROR "core-only COMPONENTS gui must fail closed")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${OPEN_LMM_GUI_BUILD_DIR}"
          --prefix "${install_prefix}"
  RESULT_VARIABLE install_result)
if(NOT install_result EQUAL 0)
  message(FATAL_ERROR "GUI package install failed: ${install_result}")
endif()

file(READ "${OPEN_LMM_GUI_BUILD_DIR}/install_manifest.txt"
  gui_install_manifest_contents)
string(REPLACE "\n" ";" gui_installed_paths
  "${gui_install_manifest_contents}")
set(gui_normalized_manifest)
foreach(gui_installed_path IN LISTS gui_installed_paths)
  if(gui_installed_path STREQUAL "")
    continue()
  endif()
  file(RELATIVE_PATH gui_relative_path
    "${install_prefix}" "${gui_installed_path}")
  if(gui_relative_path MATCHES "^\\.\\./")
    message(FATAL_ERROR
      "GUI install manifest escaped the package prefix: ${gui_installed_path}")
  endif()
  list(APPEND gui_normalized_manifest "${gui_relative_path}")
  list(FIND core_owned_paths "${gui_relative_path}" collision_index)
  if(NOT collision_index EQUAL -1)
    message(FATAL_ERROR
      "core/GUI normalized install manifest collision: ${gui_relative_path}")
  endif()
endforeach()
list(LENGTH gui_normalized_manifest gui_manifest_count)
if(gui_manifest_count LESS 10)
  message(FATAL_ERROR "GUI install manifest is unexpectedly incomplete")
endif()

foreach(required_file IN ITEMS
    include/open_lmm/gui/gui_plugin.hpp
    include/open_lmm/gui/gui_runtime_host.hpp
    lib/libopen_lmm_gui_core.so.3.0.0
    bin/open_lmm_gui
    share/OpenLmmGui/cmake/OpenLmmGuiConfig.cmake
    share/OpenLmmGui/cmake/OpenLmmGuiTargets.cmake)
  if(NOT EXISTS "${install_prefix}/${required_file}")
    message(FATAL_ERROR "installed GUI artifact is missing: ${required_file}")
  endif()
endforeach()

execute_process(
  COMMAND "${OPEN_LMM_GUI_READELF}" -d
          "${install_prefix}/lib/libopen_lmm_gui_core.so.3.0.0"
  RESULT_VARIABLE readelf_result OUTPUT_VARIABLE dynamic_section)
if(NOT readelf_result EQUAL 0 OR
   NOT dynamic_section MATCHES "SONAME.*\\[libopen_lmm_gui_core\\.so\\.3\\]" OR
   NOT dynamic_section MATCHES "(RPATH|RUNPATH).*\\[\\$ORIGIN\\]")
  message(FATAL_ERROR "GUI core SONAME/RPATH is not v3 compatible")
endif()
string(FIND "${dynamic_section}" "${OPEN_LMM_GUI_SOURCE_DIR}" source_rpath)
string(FIND "${dynamic_section}" "${OPEN_LMM_GUI_BUILD_DIR}" build_rpath)
if(NOT source_rpath EQUAL -1 OR NOT build_rpath EQUAL -1)
  message(FATAL_ERROR "installed GUI core retains a source/build RPATH")
endif()

foreach(consumer IN ITEMS direct_consumer compat_consumer)
  set(consumer_build "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/${consumer}-build")
  execute_process(
    COMMAND "${CMAKE_COMMAND}"
            -S "${gui_consumer_source}/${consumer}"
            -B "${consumer_build}"
            "-DCMAKE_PREFIX_PATH=${install_prefix}"
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
            ${gui_consumer_cmake_args}
    RESULT_VARIABLE configure_result)
  if(NOT configure_result EQUAL 0)
    message(FATAL_ERROR "${consumer} configure failed: ${configure_result}")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${consumer_build}" --parallel 1
    RESULT_VARIABLE build_result)
  if(NOT build_result EQUAL 0)
    message(FATAL_ERROR "${consumer} build failed: ${build_result}")
  endif()
  file(READ "${consumer_build}/compile_commands.json" compile_commands)
  string(FIND "${compile_commands}"
    "${gui_consumer_source}/${consumer}/main.cpp" staged_source_found)
  if(staged_source_found EQUAL -1)
    message(FATAL_ERROR
      "${consumer} did not compile its staged source-free fixture")
  endif()
  foreach(forbidden IN ITEMS
      "${OPEN_LMM_GUI_SOURCE_DIR}/src"
      "${OPEN_LMM_GUI_SOURCE_DIR}/include"
      "${open_lmm_repository_root}/applications/gui/src"
      "${open_lmm_repository_root}/applications/gui/include"
      "${open_lmm_repository_root}/open_lmm/include"
      "${open_lmm_repository_root}/open_lmm/src"
      "open_lmm/src"
      "plugins/host")
    string(FIND "${compile_commands}" "${forbidden}" found)
    if(NOT found EQUAL -1)
      message(FATAL_ERROR
        "${consumer} retained a forbidden source/build edge: ${forbidden}")
    endif()
  endforeach()
  if(consumer STREQUAL "direct_consumer")
    set(executable gui_direct)
    set(consumer_arguments)
  else()
    set(executable gui_boundary)
    file(READ "${consumer_build}/plugin-target-file.txt"
      compatibility_plugin_file)
    string(STRIP "${compatibility_plugin_file}"
      compatibility_plugin_file)
    if(NOT EXISTS "${compatibility_plugin_file}")
      message(FATAL_ERROR
        "Compat-A runtime plugin is missing: ${compatibility_plugin_file}")
    endif()
    set(consumer_arguments "${compatibility_plugin_file}")
  endif()
  file(READ "${consumer_build}/CMakeFiles/${executable}.dir/link.txt"
    consumer_link_command)
  foreach(forbidden_link IN ITEMS
      "${OPEN_LMM_GUI_SOURCE_DIR}/src"
      "${OPEN_LMM_GUI_SOURCE_DIR}/include"
      "${open_lmm_repository_root}/applications/gui/src"
      "${open_lmm_repository_root}/applications/gui/include"
      "${open_lmm_repository_root}/open_lmm/include"
      "${open_lmm_repository_root}/open_lmm/src"
      "${OPEN_LMM_GUI_BUILD_DIR}/libopen_lmm_gui_core"
      "plugins/host")
    string(FIND "${consumer_link_command}" "${forbidden_link}"
      forbidden_link_found)
    if(NOT forbidden_link_found EQUAL -1)
      message(FATAL_ERROR
        "${consumer} retained a forbidden link edge: ${forbidden_link}")
    endif()
  endforeach()
  execute_process(
    COMMAND "${OPEN_LMM_GUI_READELF}" -d
            "${consumer_build}/${executable}"
    RESULT_VARIABLE consumer_readelf_result
    OUTPUT_VARIABLE consumer_dynamic_section)
  if(NOT consumer_readelf_result EQUAL 0 OR
     NOT consumer_dynamic_section MATCHES
       "NEEDED.*\\[libopen_lmm_gui_core\\.so\\.3\\]")
    message(FATAL_ERROR
      "${consumer} does not retain the canonical GUI runtime dependency")
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH
            "${consumer_build}/${executable}" ${consumer_arguments}
    RESULT_VARIABLE run_result)
  if(NOT run_result EQUAL 0)
    message(FATAL_ERROR "${consumer} executable failed: ${run_result}")
  endif()
endforeach()

set(unqualified_build
  "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/unqualified-consumer-build")
execute_process(
  COMMAND "${CMAKE_COMMAND}"
          -S "${gui_consumer_source}/unqualified_consumer"
          -B "${unqualified_build}"
          "-DCMAKE_PREFIX_PATH=${install_prefix}"
          ${gui_consumer_cmake_args}
  RESULT_VARIABLE unqualified_result)
if(NOT unqualified_result EQUAL 0)
  message(FATAL_ERROR
    "combined unqualified discovery failed: ${unqualified_result}")
endif()

set(version_skew_build
  "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/version-skew-build")
execute_process(
  COMMAND "${CMAKE_COMMAND}"
          -S "${gui_consumer_source}/version_skew"
          -B "${version_skew_build}"
          "-DCMAKE_PREFIX_PATH=${install_prefix}"
          ${gui_consumer_cmake_args}
  RESULT_VARIABLE version_skew_result
  OUTPUT_QUIET ERROR_QUIET)
if(version_skew_result EQUAL 0)
  message(FATAL_ERROR "core/GUI exact-version skew must fail closed")
endif()

file(READ
  "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/direct_consumer-build/gui-target-file.txt"
  canonical_target_file)
string(STRIP "${canonical_target_file}" canonical_target_file)
if(NOT canonical_target_file STREQUAL
   "${install_prefix}/lib/libopen_lmm_gui_core.so.3.0.0")
  message(FATAL_ERROR
    "canonical target resolves to an unexpected file: ${canonical_target_file}")
endif()

file(READ
  "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/compat_consumer-build/gui-target-file.txt"
  compatibility_target_file)
string(STRIP "${compatibility_target_file}" compatibility_target_file)
if(NOT compatibility_target_file STREQUAL canonical_target_file)
  message(FATAL_ERROR
    "Compat-A target does not resolve to the canonical GUI DSO")
endif()

function(assert_component_file component relative_path expected)
  set(component_path
    "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/component-${component}/${relative_path}")
  if(expected AND NOT EXISTS "${component_path}")
    message(FATAL_ERROR
      "${component} is missing owned file: ${relative_path}")
  elseif(NOT expected AND EXISTS "${component_path}")
    message(FATAL_ERROR
      "${component} acquired a foreign file: ${relative_path}")
  endif()
endfunction()

foreach(component IN ITEMS
    GuiRuntime GuiDevelopment GuiApplication GuiPlugins)
  set(component_prefix
    "${OPEN_LMM_GUI_PACKAGE_TEST_ROOT}/component-${component}")
  execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${OPEN_LMM_GUI_BUILD_DIR}"
            --prefix "${component_prefix}" --component "${component}"
    RESULT_VARIABLE component_install_result)
  if(NOT component_install_result EQUAL 0)
    message(FATAL_ERROR "${component} install failed")
  endif()
endforeach()

assert_component_file(
  GuiRuntime lib/libopen_lmm_gui_core.so.3.0.0 TRUE)
assert_component_file(
  GuiRuntime share/open_lmm_gui/RELEASE_POLICY.md TRUE)
assert_component_file(
  GuiRuntime include/open_lmm/gui/gui_plugin.hpp FALSE)
assert_component_file(GuiRuntime bin/open_lmm_gui FALSE)

assert_component_file(
  GuiDevelopment include/open_lmm/gui/gui_plugin.hpp TRUE)
assert_component_file(
  GuiDevelopment share/OpenLmmGui/cmake/OpenLmmGuiConfig.cmake TRUE)
assert_component_file(
  GuiDevelopment lib/libopen_lmm_gui_core.so.3.0.0 FALSE)
assert_component_file(GuiDevelopment bin/open_lmm_gui FALSE)

assert_component_file(GuiApplication bin/open_lmm_gui TRUE)
assert_component_file(
  GuiApplication share/open_lmm_gui/README.md TRUE)
assert_component_file(
  GuiApplication lib/libopen_lmm_gui_core.so.3.0.0 FALSE)
assert_component_file(
  GuiApplication include/open_lmm/gui/gui_plugin.hpp FALSE)

if(EXISTS "${OPEN_LMM_GUI_BUILD_DIR}/libopen_lmm_iridescence_gui.so")
  assert_component_file(
    GuiPlugins lib/libopen_lmm_iridescence_gui.so.3.0.0 TRUE)
endif()
assert_component_file(
  GuiPlugins lib/libopen_lmm_gui_core.so.3.0.0 FALSE)
assert_component_file(GuiPlugins bin/open_lmm_gui FALSE)

list(LENGTH core_owned_paths core_manifest_count)
message(STATUS
  "GUI package audit passed: core_manifest_count=${core_manifest_count}; gui_manifest_count=${gui_manifest_count}")
