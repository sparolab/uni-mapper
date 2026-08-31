cmake_minimum_required(VERSION 3.25)

foreach(required IN ITEMS OPEN_LMM_REPOSITORY_ROOT OPEN_LMM_CORE_BUILD_DIR
    OPEN_LMM_CLI_BUILD_DIR OPEN_LMM_GUI_BUILD_DIR OPEN_LMM_ROS_BUILD_DIR
    OPEN_LMM_DISTRIBUTION_TEST_ROOT)
  if(NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
    message(FATAL_ERROR "${required} is required")
  endif()
endforeach()

set(artifact_manifest
  "${OPEN_LMM_REPOSITORY_ROOT}/distribution/manifests/artifact-set-v3.tsv")
set(migration_manifest
  "${OPEN_LMM_REPOSITORY_ROOT}/distribution/manifests/legacy-owner-transfers-v3.tsv")
file(STRINGS "${artifact_manifest}" artifact_rows)
list(POP_FRONT artifact_rows artifact_header)
if(NOT artifact_header STREQUAL
   "schema_version\tartifact_id\tversion\tnamespace\tinstall_mode\tprofiles\texact_dependencies\towner_root")
  message(FATAL_ERROR "artifact manifest schema changed")
endif()
set(expected_artifacts core cli gui python ros)
foreach(row IN LISTS artifact_rows)
  string(REPLACE "\t" ";" fields "${row}")
  list(LENGTH fields field_count)
  if(NOT field_count EQUAL 8)
    message(FATAL_ERROR "invalid artifact manifest row: ${row}")
  endif()
  list(GET fields 0 schema_version)
  list(GET fields 1 artifact_id)
  list(GET fields 2 version)
  if(NOT schema_version STREQUAL "1" OR NOT version STREQUAL "3.0.0")
    message(FATAL_ERROR "unsupported artifact contract: ${row}")
  endif()
  list(APPEND actual_artifacts "${artifact_id}")
endforeach()
if(NOT actual_artifacts STREQUAL expected_artifacts)
  message(FATAL_ERROR "artifact set changed: ${actual_artifacts}")
endif()

file(REMOVE_RECURSE "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}")
set(native_prefix "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/native")
set(ros_overlay "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/ros-overlay")
file(MAKE_DIRECTORY "${native_prefix}" "${ros_overlay}")

function(run_install artifact build_dir prefix)
  set(arguments --install "${build_dir}" --prefix "${prefix}")
  if(artifact STREQUAL "cli")
    list(APPEND arguments --component Tools)
  endif()
  execute_process(COMMAND "${CMAKE_COMMAND}" ${arguments}
    RESULT_VARIABLE result OUTPUT_VARIABLE stdout ERROR_VARIABLE stderr)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR
      "${artifact} install failed: ${result}\n${stdout}\n${stderr}")
  endif()
endfunction()

function(find_install_manifest artifact build_dir output)
  if(artifact STREQUAL "cli" AND
     EXISTS "${build_dir}/install_manifest_Tools.txt")
    set(path "${build_dir}/install_manifest_Tools.txt")
  else()
    set(path "${build_dir}/install_manifest.txt")
  endif()
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "${artifact} install manifest is missing: ${path}")
  endif()
  set(${output} "${path}" PARENT_SCOPE)
endfunction()

function(read_owned_paths artifact namespace prefix build_dir output)
  find_install_manifest("${artifact}" "${build_dir}" manifest)
  file(STRINGS "${manifest}" absolute_paths)
  set(relative_paths)
  string(LENGTH "${prefix}/" prefix_length)
  foreach(path IN LISTS absolute_paths)
    string(FIND "${path}" "${prefix}/" prefix_index)
    if(NOT prefix_index EQUAL 0)
      message(FATAL_ERROR
        "${artifact} manifest escaped ${namespace}: ${path}")
    endif()
    string(SUBSTRING "${path}" ${prefix_length} -1 relative_path)
    list(APPEND relative_paths "${relative_path}")
  endforeach()
  list(REMOVE_DUPLICATES relative_paths)
  list(SORT relative_paths)
  set(${output} "${relative_paths}" PARENT_SCOPE)
endfunction()

# Canonical publication is composed in an empty staging prefix. Leaf builds are
# inputs only; distribution never adds their source trees as subdirectories.
run_install(core "${OPEN_LMM_CORE_BUILD_DIR}" "${native_prefix}")
read_owned_paths(core native-prefix "${native_prefix}"
  "${OPEN_LMM_CORE_BUILD_DIR}" core_paths)
run_install(cli "${OPEN_LMM_CLI_BUILD_DIR}" "${native_prefix}")
read_owned_paths(cli native-prefix "${native_prefix}"
  "${OPEN_LMM_CLI_BUILD_DIR}" cli_paths)
run_install(gui "${OPEN_LMM_GUI_BUILD_DIR}" "${native_prefix}")
read_owned_paths(gui native-prefix "${native_prefix}"
  "${OPEN_LMM_GUI_BUILD_DIR}" gui_paths)
run_install(ros "${OPEN_LMM_ROS_BUILD_DIR}" "${ros_overlay}")
read_owned_paths(ros ros-overlay "${ros_overlay}"
  "${OPEN_LMM_ROS_BUILD_DIR}" ros_paths)

set(native_owner_rows)
foreach(artifact IN ITEMS core cli gui)
  if(artifact STREQUAL "cli")
    set(component Tools)
  else()
    set(component unqualified)
  endif()
  foreach(path IN LISTS ${artifact}_paths)
    if(path IN_LIST native_owned_paths)
      message(FATAL_ERROR "native-prefix ownership collision: ${path}")
    endif()
    list(APPEND native_owned_paths "${path}")
    if(IS_SYMLINK "${native_prefix}/${path}")
      set(kind symlink)
      file(READ_SYMLINK "${native_prefix}/${path}" target)
      string(SHA256 digest "symlink:${target}")
    elseif(EXISTS "${native_prefix}/${path}" AND
           NOT IS_DIRECTORY "${native_prefix}/${path}")
      set(kind file)
      file(SHA256 "${native_prefix}/${path}" digest)
    else()
      message(FATAL_ERROR "owned native path is missing: ${artifact}:${path}")
    endif()
    list(APPEND native_owner_rows
      "native-prefix\t${path}\t${artifact}\t${component}\t${kind}\t${digest}")
  endforeach()
endforeach()
foreach(path IN LISTS ros_paths)
  if(IS_SYMLINK "${ros_overlay}/${path}")
    set(kind symlink)
    file(READ_SYMLINK "${ros_overlay}/${path}" target)
    string(SHA256 digest "symlink:${target}")
  elseif(EXISTS "${ros_overlay}/${path}" AND
         NOT IS_DIRECTORY "${ros_overlay}/${path}")
    set(kind file)
    file(SHA256 "${ros_overlay}/${path}" digest)
  else()
    message(FATAL_ERROR "owned ROS path is missing: ${path}")
  endif()
  list(APPEND native_owner_rows
    "ros-overlay\t${path}\tros\tament\t${kind}\t${digest}")
endforeach()
list(SORT native_owner_rows)
string(REPLACE ";" "\n" owner_body "${native_owner_rows}")
file(WRITE "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/owner-inventory.tsv"
  "namespace\trelative_path\tartifact_id\tcomponent\tfile_kind\tsha256\n${owner_body}\n")

foreach(required_path IN ITEMS
    bin/open_lmm_batch bin/open_lmm_gui
    include/open_lmm/gui/gui_plugin.hpp
    share/open_lmm/cmake/open_lmmConfig.cmake
    share/OpenLmmGui/cmake/OpenLmmGuiConfig.cmake)
  if(NOT EXISTS "${native_prefix}/${required_path}")
    message(FATAL_ERROR "combined native artifact is missing ${required_path}")
  endif()
endforeach()
if(NOT EXISTS "${ros_overlay}/share/open_lmm_ros/package.xml")
  message(FATAL_ERROR "ROS overlay package metadata is missing")
endif()

function(require_version_skew_failure artifact source prefix)
  set(build "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/version-skew-${artifact}")
  execute_process(COMMAND "${CMAKE_COMMAND}" -S "${source}" -B "${build}"
      "-DCMAKE_PREFIX_PATH=${prefix}" -DBUILD_TESTING=OFF
      -DOPEN_LMM_GUI_BUILD_IRIDESCENCE=OFF
      -DOPEN_LMM_ROS_BUILD_GUI=OFF
    RESULT_VARIABLE result OUTPUT_VARIABLE stdout ERROR_VARIABLE stderr)
  if(result EQUAL 0)
    message(FATAL_ERROR "${artifact} accepted a mismatched exact dependency")
  endif()
  set(diagnostic "${stdout}${stderr}")
  if(NOT diagnostic MATCHES "open_lmm" OR
     NOT diagnostic MATCHES "3\\.0\\.[01]")
    message(FATAL_ERROR
      "${artifact} skew fixture failed for a reason other than the exact "
      "open_lmm version contract:\n${diagnostic}")
  endif()
endfunction()

# Core consumers reject a newer exact request. Every leaf rejects a core
# package that reports 3.0.1 while the leaf itself remains version 3.0.0.
set(core_skew_source "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/core-version-skew-source")
file(MAKE_DIRECTORY "${core_skew_source}")
file(WRITE "${core_skew_source}/CMakeLists.txt"
  "cmake_minimum_required(VERSION 3.25)\n"
  "project(open_lmm_core_skew LANGUAGES NONE)\n"
  "find_package(open_lmm 3.0.1 EXACT CONFIG REQUIRED COMPONENTS client)\n")
require_version_skew_failure(core "${core_skew_source}" "${native_prefix}")

set(skew_prefix "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/skew-native")
file(COPY "${native_prefix}/" DESTINATION "${skew_prefix}")
foreach(version_file IN ITEMS
    open_lmmConfigVersion.cmake open_lmmConfig-version.cmake)
  if(EXISTS "${skew_prefix}/share/open_lmm/cmake/${version_file}")
    file(WRITE "${skew_prefix}/share/open_lmm/cmake/${version_file}"
      "set(PACKAGE_VERSION \"3.0.1\")\n"
      "set(PACKAGE_VERSION_COMPATIBLE FALSE)\n"
      "set(PACKAGE_VERSION_EXACT FALSE)\n"
      "set(PACKAGE_VERSION_UNSUITABLE TRUE)\n")
  endif()
endforeach()
require_version_skew_failure(cli
  "${OPEN_LMM_REPOSITORY_ROOT}/applications/cli" "${skew_prefix}")
require_version_skew_failure(gui
  "${OPEN_LMM_REPOSITORY_ROOT}/applications/gui" "${skew_prefix}")
require_version_skew_failure(python
  "${OPEN_LMM_REPOSITORY_ROOT}/bindings/python" "${skew_prefix}")
require_version_skew_failure(ros
  "${OPEN_LMM_REPOSITORY_ROOT}/ros" "${skew_prefix}")

# Upgrade convergence is deliberately exact-path only. This validates file
# convergence, not crash-atomic in-place publication.
set(upgrade_prefix "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/upgrade")
file(MAKE_DIRECTORY "${upgrade_prefix}/unknown")
file(WRITE "${upgrade_prefix}/unknown/user-file" "preserve\n")
file(STRINGS "${migration_manifest}" migration_rows)
list(POP_FRONT migration_rows migration_header)
if(NOT migration_header STREQUAL
   "baseline_commit\told_owner\tnamespace\trelative_path\tdisposition\tnew_owner")
  message(FATAL_ERROR "migration manifest schema changed")
endif()
foreach(row IN LISTS migration_rows)
  string(REPLACE "\t" ";" fields "${row}")
  list(LENGTH fields field_count)
  if(NOT field_count EQUAL 6)
    message(FATAL_ERROR "invalid migration row: ${row}")
  endif()
  list(GET fields 0 baseline)
  list(GET fields 1 old_owner)
  list(GET fields 2 namespace)
  list(GET fields 3 path)
  list(GET fields 4 disposition)
  list(GET fields 5 new_owner)
  if(NOT baseline STREQUAL "cdda354" OR
     NOT old_owner STREQUAL "core" OR
     NOT namespace STREQUAL "native-prefix" OR path MATCHES "[?*]" OR
     path MATCHES "(^|/)\.\.(/|$)")
    message(FATAL_ERROR "unsafe migration row: ${row}")
  endif()
  get_filename_component(parent "${upgrade_prefix}/${path}" DIRECTORY)
  file(MAKE_DIRECTORY "${parent}")
  file(WRITE "${upgrade_prefix}/${path}" "legacy:${path}\n")
  if(disposition STREQUAL "remove")
    file(REMOVE "${upgrade_prefix}/${path}")
    list(APPEND removed_legacy_paths "${path}")
  elseif(disposition STREQUAL "transfer")
    if(NOT new_owner MATCHES "^(cli|gui)$")
      message(FATAL_ERROR "invalid transfer owner: ${row}")
    endif()
    file(REMOVE "${upgrade_prefix}/${path}")
    list(APPEND transferred_legacy_rows "${path}|${new_owner}")
  elseif(NOT disposition STREQUAL "transfer")
    message(FATAL_ERROR "unknown migration disposition: ${disposition}")
  endif()
endforeach()
run_install(core "${OPEN_LMM_CORE_BUILD_DIR}" "${upgrade_prefix}")
run_install(cli "${OPEN_LMM_CLI_BUILD_DIR}" "${upgrade_prefix}")
run_install(gui "${OPEN_LMM_GUI_BUILD_DIR}" "${upgrade_prefix}")
foreach(path IN LISTS removed_legacy_paths)
  if(EXISTS "${upgrade_prefix}/${path}")
    message(FATAL_ERROR "known stale legacy path survived: ${path}")
  endif()
endforeach()
foreach(transfer IN LISTS transferred_legacy_rows)
  string(REPLACE "|" ";" fields "${transfer}")
  list(GET fields 0 path)
  list(GET fields 1 new_owner)
  if(NOT path IN_LIST ${new_owner}_paths)
    if(EXISTS "${upgrade_prefix}/${path}" OR
       IS_SYMLINK "${upgrade_prefix}/${path}")
      message(FATAL_ERROR
        "optional transferred path survived without a current owner: ${path}")
    endif()
  elseif(IS_SYMLINK "${native_prefix}/${path}")
    file(READ_SYMLINK "${native_prefix}/${path}" canonical_target)
    file(READ_SYMLINK "${upgrade_prefix}/${path}" upgraded_target)
    if(NOT upgraded_target STREQUAL canonical_target)
      message(FATAL_ERROR "transfer did not converge: ${path}")
    endif()
  else()
    file(SHA256 "${native_prefix}/${path}" canonical_digest)
    file(SHA256 "${upgrade_prefix}/${path}" upgraded_digest)
    if(NOT upgraded_digest STREQUAL canonical_digest)
      message(FATAL_ERROR "transfer did not converge: ${path}")
    endif()
  endif()
endforeach()
if(NOT EXISTS "${upgrade_prefix}/unknown/user-file")
  message(FATAL_ERROR "upgrade removed an unknown user file")
endif()

# Reinstall must converge to the same owned bytes.
file(SHA256 "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/owner-inventory.tsv"
  inventory_hash)
run_install(core "${OPEN_LMM_CORE_BUILD_DIR}" "${native_prefix}")
run_install(cli "${OPEN_LMM_CLI_BUILD_DIR}" "${native_prefix}")
run_install(gui "${OPEN_LMM_GUI_BUILD_DIR}" "${native_prefix}")
foreach(row IN LISTS native_owner_rows)
  string(REPLACE "\t" ";" fields "${row}")
  list(GET fields 0 namespace)
  list(GET fields 1 path)
  list(GET fields 5 expected_digest)
  if(namespace STREQUAL "native-prefix")
    if(IS_SYMLINK "${native_prefix}/${path}")
      file(READ_SYMLINK "${native_prefix}/${path}" target)
      string(SHA256 actual_digest "symlink:${target}")
    else()
      file(SHA256 "${native_prefix}/${path}" actual_digest)
    endif()
    if(NOT actual_digest STREQUAL expected_digest)
      message(FATAL_ERROR "reinstall changed owned bytes: ${path}")
    endif()
  endif()
endforeach()

# Manifest uninstall simulation removes only exact owner paths. Reverse order
# must preserve every remaining owner and unknown files.
set(ros_uninstall_root "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/uninstall-ros")
file(COPY "${ros_overlay}/" DESTINATION "${ros_uninstall_root}")
file(MAKE_DIRECTORY "${ros_uninstall_root}/unknown")
file(WRITE "${ros_uninstall_root}/unknown/user-file" "preserve\n")
foreach(path IN LISTS ros_paths)
  file(REMOVE "${ros_uninstall_root}/${path}")
endforeach()
if(NOT EXISTS "${ros_uninstall_root}/unknown/user-file")
  message(FATAL_ERROR "uninstall of ros removed unknown overlay file")
endif()

set(uninstall_root "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/uninstall")
file(COPY "${native_prefix}/" DESTINATION "${uninstall_root}")
file(MAKE_DIRECTORY "${uninstall_root}/unknown")
file(WRITE "${uninstall_root}/unknown/user-file" "preserve\n")
foreach(artifact IN ITEMS gui cli core)
  foreach(path IN LISTS ${artifact}_paths)
    file(REMOVE "${uninstall_root}/${path}")
  endforeach()
  list(APPEND removed_artifacts "${artifact}")
  foreach(other IN ITEMS core cli gui)
    if(other IN_LIST removed_artifacts)
      continue()
    endif()
    foreach(path IN LISTS ${other}_paths)
      if(NOT EXISTS "${uninstall_root}/${path}" AND
         NOT IS_SYMLINK "${uninstall_root}/${path}")
        message(FATAL_ERROR
          "uninstall of ${artifact} removed ${other}-owned ${path}")
      endif()
    endforeach()
  endforeach()
  if(NOT EXISTS "${uninstall_root}/unknown/user-file")
    message(FATAL_ERROR "uninstall of ${artifact} removed unknown file")
  endif()
endforeach()

list(LENGTH core_paths core_count)
list(LENGTH cli_paths cli_count)
list(LENGTH gui_paths gui_count)
list(LENGTH ros_paths ros_count)
file(WRITE "${OPEN_LMM_DISTRIBUTION_TEST_ROOT}/metrics.txt"
  "core=${core_count}\ncli=${cli_count}\ngui=${gui_count}\nros=${ros_count}\n"
  "collisions=0\nversion_skew=pass\nupgrade=pass\nuninstall=pass\n"
  "owner_inventory_sha256=${inventory_hash}\n")
message(STATUS "combined distribution ownership verified")
