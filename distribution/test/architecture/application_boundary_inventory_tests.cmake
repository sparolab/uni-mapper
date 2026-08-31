if(NOT OPEN_LMM_REPOSITORY_ROOT)
  message(FATAL_ERROR "OPEN_LMM_REPOSITORY_ROOT is required")
endif()

set(manifest_path
  "${OPEN_LMM_REPOSITORY_ROOT}/distribution/test/architecture/manifests/application_boundary_edges.tsv")
if(NOT EXISTS "${manifest_path}")
  message(FATAL_ERROR "application boundary inventory is missing")
endif()

function(assert_contains path)
  if(NOT EXISTS "${OPEN_LMM_REPOSITORY_ROOT}/${path}")
    message(FATAL_ERROR "required boundary file is missing: ${path}")
  endif()
  file(READ "${OPEN_LMM_REPOSITORY_ROOT}/${path}" contents)
  foreach(pattern IN LISTS ARGN)
    string(FIND "${contents}" "${pattern}" found)
    if(found EQUAL -1)
      message(FATAL_ERROR "${path} must contain: ${pattern}")
    endif()
  endforeach()
endfunction()

function(assert_excludes path)
  if(NOT EXISTS "${OPEN_LMM_REPOSITORY_ROOT}/${path}")
    message(FATAL_ERROR "required boundary file is missing: ${path}")
  endif()
  file(READ "${OPEN_LMM_REPOSITORY_ROOT}/${path}" contents)
  foreach(pattern IN LISTS ARGN)
    string(FIND "${contents}" "${pattern}" found)
    if(NOT found EQUAL -1)
      message(FATAL_ERROR "${path} contains forbidden edge: ${pattern}")
    endif()
  endforeach()
endfunction()

function(assert_direct_link path target dependency)
  if(NOT EXISTS "${OPEN_LMM_REPOSITORY_ROOT}/${path}")
    message(FATAL_ERROR "required boundary file is missing: ${path}")
  endif()
  file(READ "${OPEN_LMM_REPOSITORY_ROOT}/${path}" contents)
  string(REGEX MATCH
    "target_link_libraries\\([ \t\r\n]*${target}[^)]*\\)"
    target_link_block "${contents}")
  string(FIND "${target_link_block}" "${dependency}" dependency_found)
  if(NOT target_link_block OR dependency_found EQUAL -1)
    message(FATAL_ERROR
      "${path} does not directly link ${target} to ${dependency}")
  endif()
endfunction()

file(STRINGS "${manifest_path}" manifest_lines)
if(NOT manifest_lines)
  message(FATAL_ERROR "application boundary inventory is empty")
endif()
list(GET manifest_lines 0 manifest_header)
list(REMOVE_AT manifest_lines 0)
set(expected_header
  "consumer\tedge_kind\tsource\tdependency\tclassification\tdisposition\towner\texpires")
if(NOT manifest_header STREQUAL expected_header)
  message(FATAL_ERROR
    "application boundary inventory header changed: ${manifest_header}")
endif()

set(sorted_manifest_lines ${manifest_lines})
list(SORT sorted_manifest_lines)
if(NOT manifest_lines STREQUAL sorted_manifest_lines)
  message(FATAL_ERROR "application boundary inventory rows must be sorted")
endif()

set(manifest_include_keys)
set(private_rows)
set(seen_manifest_rows)
foreach(row IN LISTS manifest_lines)
  list(FIND seen_manifest_rows "${row}" duplicate_row_index)
  if(NOT duplicate_row_index EQUAL -1)
    message(FATAL_ERROR "duplicate application inventory row: ${row}")
  endif()
  list(APPEND seen_manifest_rows "${row}")
  string(REPLACE "\t" ";" fields "${row}")
  list(LENGTH fields field_count)
  if(NOT field_count EQUAL 8)
    message(FATAL_ERROR "inventory row must contain 8 columns: ${row}")
  endif()
  list(GET fields 0 consumer)
  list(GET fields 1 edge_kind)
  list(GET fields 2 source)
  list(GET fields 3 dependency)
  list(GET fields 4 classification)
  list(GET fields 5 disposition)
  list(GET fields 6 owner)
  list(GET fields 7 expires)

  if(NOT consumer MATCHES
      "^(cli|gui|python_native|ros_runtime|ros_visualization)$")
    message(FATAL_ERROR "invalid inventory consumer: ${row}")
  endif()
  if(NOT edge_kind MATCHES "^(include|link|package_component)$")
    message(FATAL_ERROR "invalid inventory edge kind: ${row}")
  endif()
  if(NOT classification MATCHES
      "^(public_installed|adapter_local|external|private)$")
    message(FATAL_ERROR "invalid inventory classification: ${row}")
  endif()
  if(NOT disposition MATCHES "^(keep|remove_p0|temporary_exception)$")
    message(FATAL_ERROR "invalid inventory disposition: ${row}")
  endif()
  if(NOT expires MATCHES "^(none|P0|P4)$")
    message(FATAL_ERROR "invalid inventory expiry: ${row}")
  endif()
  if(owner STREQUAL "" OR owner STREQUAL "none")
    message(FATAL_ERROR "inventory edge lacks an owner: ${row}")
  endif()
  if(disposition STREQUAL "remove_p0" OR expires STREQUAL "P0")
    message(FATAL_ERROR "completed P0 inventory retains a P0 edge: ${row}")
  endif()
  if(classification STREQUAL "private")
    if(NOT disposition STREQUAL "temporary_exception" OR
       NOT expires STREQUAL "P4")
      message(FATAL_ERROR
        "private inventory edge lacks a bounded exception: ${row}")
    endif()
    list(APPEND private_rows "${row}")
  elseif(disposition STREQUAL "temporary_exception")
    message(FATAL_ERROR
      "only private edges may be temporary exceptions: ${row}")
  endif()

  if(edge_kind STREQUAL "include")
    list(APPEND manifest_include_keys
      "${consumer}\t${source}\t${dependency}\t${classification}")
  elseif(edge_kind STREQUAL "link")
    if(consumer STREQUAL "cli")
      set(cmake_path "applications/cli/CMakeLists.txt")
    elseif(consumer STREQUAL "gui")
      set(cmake_path "applications/gui/CMakeLists.txt")
    elseif(consumer STREQUAL "python_native")
      set(cmake_path "bindings/python/CMakeLists.txt")
    else()
      set(cmake_path "ros/CMakeLists.txt")
    endif()
    assert_direct_link("${cmake_path}" "${source}" "${dependency}")
  elseif(edge_kind STREQUAL "package_component")
    assert_contains("${source}" "COMPONENTS ${dependency}")
  endif()
endforeach()

list(LENGTH private_rows private_row_count)
if(NOT private_row_count EQUAL 0)
  message(FATAL_ERROR
    "P4 applications must have no private core edge: ${private_rows}")
endif()

set(scan_specs
  "ros_runtime|ros/ros2"
  "cli|applications/cli"
  "gui|applications/gui"
  "python_native|applications/python"
  "python_native|bindings/python"
  "ros_runtime|applications/ros2")
set(actual_include_keys)
foreach(scan_spec IN LISTS scan_specs)
  string(REPLACE "|" ";" scan_fields "${scan_spec}")
  list(GET scan_fields 0 scan_consumer)
  list(GET scan_fields 1 scan_root)
  set(absolute_scan_root "${OPEN_LMM_REPOSITORY_ROOT}/${scan_root}")
  if(NOT EXISTS "${absolute_scan_root}")
    continue()
  endif()
  file(GLOB_RECURSE source_files LIST_DIRECTORIES false
    "${absolute_scan_root}/*.c"
    "${absolute_scan_root}/*.cc"
    "${absolute_scan_root}/*.cpp"
    "${absolute_scan_root}/*.cxx"
    "${absolute_scan_root}/*.h"
    "${absolute_scan_root}/*.hh"
    "${absolute_scan_root}/*.hpp"
    "${absolute_scan_root}/*.hxx")
  foreach(source_file IN LISTS source_files)
    file(RELATIVE_PATH relative_source
      "${OPEN_LMM_REPOSITORY_ROOT}" "${source_file}")
    if(relative_source MATCHES "/test/")
      continue()
    endif()
    set(source_consumer "${scan_consumer}")
    if(scan_consumer STREQUAL "ros_runtime" AND
       relative_source MATCHES "/gui(_composition)?/")
      set(source_consumer "ros_gui")
    elseif(scan_consumer STREQUAL "ros_runtime" AND
           relative_source MATCHES "/ros_visualization_bridge\\.(cpp|hpp)$")
      set(source_consumer "ros_visualization")
    endif()
    file(STRINGS "${source_file}" include_lines
      REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"][^>\"]+[>\"]")
    foreach(include_line IN LISTS include_lines)
      string(REGEX REPLACE
        "^[ \t]*#[ \t]*include[ \t]*[<\"]([^>\"]+)[>\"].*$"
        "\\1" dependency "${include_line}")
      string(FIND "${include_line}" "\"" quote_index)
      if(dependency MATCHES
          "(^|\\.\\./)(foundation|runtime|plugins/host|domain|storage|visualization|config/document|config/domain)/" OR
         dependency MATCHES "(^|/)open_lmm/src/")
        set(classification private)
      elseif(dependency MATCHES "^open_lmm/")
        set(classification public_installed)
      elseif(NOT quote_index EQUAL -1 OR dependency MATCHES "^adapters/")
        set(classification adapter_local)
      else()
        continue()
      endif()
      list(APPEND actual_include_keys
        "${source_consumer}\t${relative_source}\t${dependency}\t${classification}")
    endforeach()
  endforeach()
endforeach()
list(REMOVE_DUPLICATES actual_include_keys)
list(SORT actual_include_keys)
list(SORT manifest_include_keys)
if(NOT actual_include_keys STREQUAL manifest_include_keys)
  message(FATAL_ERROR
    "application include inventory drifted\n"
    "manifest=[${manifest_include_keys}]\nactual=[${actual_include_keys}]")
endif()

set(future_application_roots
  "applications/cli"
  "applications/gui"
  "applications/python"
  "applications/ros2"
  "bindings/python")
foreach(future_root IN LISTS future_application_roots)
  set(absolute_future_root
    "${OPEN_LMM_REPOSITORY_ROOT}/${future_root}")
  if(NOT EXISTS "${absolute_future_root}")
    continue()
  endif()
  file(GLOB_RECURSE future_cmake_files LIST_DIRECTORIES false
    "${absolute_future_root}/CMakeLists.txt"
    "${absolute_future_root}/*.cmake")
  foreach(future_cmake IN LISTS future_cmake_files)
    file(RELATIVE_PATH relative_future_cmake
      "${OPEN_LMM_REPOSITORY_ROOT}" "${future_cmake}")
    if(relative_future_cmake MATCHES "/test/")
      continue()
    endif()
    file(READ "${future_cmake}" future_cmake_contents)
    foreach(forbidden_cmake_edge IN ITEMS
        "open_lmm/src/"
        "open_lmm_runtime_"
        "open_lmm_map_server"
        "open_lmm_utils"
        "open_lmm_client")
      set(has_forbidden_cmake_edge FALSE)
      if(forbidden_cmake_edge STREQUAL "open_lmm/src/" OR
         forbidden_cmake_edge STREQUAL "open_lmm_runtime_")
        string(FIND "${future_cmake_contents}" "${forbidden_cmake_edge}"
          forbidden_cmake_edge_found)
        if(NOT forbidden_cmake_edge_found EQUAL -1)
          set(has_forbidden_cmake_edge TRUE)
        endif()
      elseif(future_cmake_contents MATCHES
          "(^|[^A-Za-z0-9_])${forbidden_cmake_edge}([^A-Za-z0-9_]|$)")
        set(has_forbidden_cmake_edge TRUE)
      endif()
      if(has_forbidden_cmake_edge)
        message(FATAL_ERROR
          "future application CMake has a private/build-tree edge: "
          "${relative_future_cmake}: ${forbidden_cmake_edge}")
      endif()
    endforeach()
    if(future_cmake_contents MATCHES
        "add_subdirectory\\([^)]*open_lmm")
      message(FATAL_ERROR
        "future application embeds the core superbuild: "
        "${relative_future_cmake}")
    endif()
  endforeach()
endforeach()

assert_contains(
  "applications/cli/src/main.cpp"
  "RuntimeClient"
  "WriteError")
assert_excludes(
  "applications/cli/src/main.cpp"
  "foundation/logging/"
  "InitializeLogging"
  "LogError")
assert_contains(
  "applications/cli/CMakeLists.txt"
  "find_package(open_lmm \${PROJECT_VERSION} EXACT CONFIG REQUIRED"
  "COMPONENTS client"
  "add_executable(open_lmm_batch src/main.cpp)"
  "open_lmm::client")
assert_excludes(
  "applications/cli/CMakeLists.txt"
  "open_lmm_utils"
  "open_lmm_map_server"
  "open_lmm_runtime_service_objects"
  "open_lmm_runtime_composition_objects")

assert_excludes(
  "open_lmm/CMakeLists.txt"
  "applications/cli"
  "src/adapters/batch"
  "open_lmm_batch")

assert_contains(
  "applications/gui/src/iridescence/iridescence_gui.cpp"
  "RecentApplicationLogs"
  "ApplicationLogInfo")
assert_excludes(
  "applications/gui/src/iridescence/iridescence_gui.cpp"
  "foundation/logging/"
  "RecentRuntimeLogs"
  "    LogInfo(profile.str())"
  "spdlog/")
assert_contains(
  "applications/gui/src/iridescence/application_log.cpp"
  "spdlog::sinks::ringbuffer_sink_mt"
  "spdlog::info")
assert_excludes(
  "applications/gui/src/iridescence/application_log.cpp"
  "foundation/logging/")
assert_contains(
  "applications/gui/CMakeLists.txt"
  "find_package(spdlog REQUIRED)"
  "spdlog::spdlog")

assert_contains(
  "bindings/python/CMakeLists.txt"
  "find_package(open_lmm \${PROJECT_VERSION} EXACT CONFIG REQUIRED"
  "COMPONENTS client"
  "target_link_libraries(open_lmm_python_native PRIVATE open_lmm::client)")
assert_excludes(
  "bindings/python/CMakeLists.txt"
  "open_lmm_utils"
  "open_lmm_map_server"
  "add_subdirectory(../../open_lmm")
assert_contains(
  "ros/CMakeLists.txt"
  "find_package(open_lmm \${PROJECT_VERSION} EXACT CONFIG REQUIRED COMPONENTS client)"
  "open_lmm::client"
  "ros_visualization_bridge.cpp")
assert_excludes(
  "ros/CMakeLists.txt"
  "add_subdirectory("
  "open_lmm::utils"
  "open_lmm_map_server"
  "open_lmm::gui"
  "OPEN_LMM_ROS_BUILD_GUI"
  "open_lmm_ros_gui_component")

file(GLOB_RECURSE core_sources LIST_DIRECTORIES false
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.c"
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.cc"
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.cpp"
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.cxx"
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.h"
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.hh"
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.hpp"
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.hxx")
foreach(core_source IN LISTS core_sources)
  file(STRINGS "${core_source}" core_include_lines
    REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]")
  foreach(core_include_line IN LISTS core_include_lines)
    if(core_include_line MATCHES
        "^[ \t]*#[ \t]*include[ \t]*[<\"](applications|bindings|ros)/")
      file(RELATIVE_PATH relative_core_source
        "${OPEN_LMM_REPOSITORY_ROOT}" "${core_source}")
      message(FATAL_ERROR
        "core production source has a reverse adapter edge: "
        "${relative_core_source}: ${core_include_line}")
    endif()
  endforeach()
endforeach()

file(GLOB_RECURSE core_cmake_files LIST_DIRECTORIES false
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/CMakeLists.txt"
  "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src/*.cmake")
foreach(core_cmake IN LISTS core_cmake_files)
  file(READ "${core_cmake}" core_cmake_contents)
  if(core_cmake_contents MATCHES
      "(^|[^A-Za-z0-9_])(applications/|bindings/|\\.\\./ros/)")
    file(RELATIVE_PATH relative_core_cmake
      "${OPEN_LMM_REPOSITORY_ROOT}" "${core_cmake}")
    message(FATAL_ERROR
      "core production CMake has a reverse adapter edge: "
      "${relative_core_cmake}")
  endif()
endforeach()

assert_excludes(
  "open_lmm/CMakeLists.txt"
  "add_subdirectory(../applications"
  "add_subdirectory(../bindings"
  "add_subdirectory(../ros")
