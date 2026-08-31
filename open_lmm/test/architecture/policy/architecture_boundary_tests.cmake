if(NOT OPEN_LMM_SOURCE_DIR)
  message(FATAL_ERROR "OPEN_LMM_SOURCE_DIR is required")
endif()

function(assert_tree_has_no_repository_leaf_edges root)
  if(NOT EXISTS "${OPEN_LMM_SOURCE_DIR}/${root}")
    return()
  endif()
  file(GLOB_RECURSE candidates LIST_DIRECTORIES false
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.c"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.cc"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.cmake"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.cpp"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.cxx"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.h"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.hh"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.hpp"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.hxx"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.inl"
    "${OPEN_LMM_SOURCE_DIR}/${root}/CMakeLists.txt")
  foreach(candidate IN LISTS candidates)
    file(READ "${candidate}" contents)
    foreach(pattern IN ITEMS
        "../applications" "../bindings" "../ros"
        "applications/cli" "applications/gui" "bindings/python")
      string(FIND "${contents}" "${pattern}" found)
      if(NOT found EQUAL -1)
        file(RELATIVE_PATH relative_candidate
          "${OPEN_LMM_SOURCE_DIR}" "${candidate}")
        message(FATAL_ERROR
          "core source must not consume repository leaf source: "
          "${relative_candidate} contains ${pattern}")
      endif()
    endforeach()
  endforeach()
endfunction()

foreach(core_root IN ITEMS cmake include src test/cmake test/package)
  assert_tree_has_no_repository_leaf_edges("${core_root}")
endforeach()

file(GLOB_RECURSE legacy_adapter_files LIST_DIRECTORIES false
  "${OPEN_LMM_SOURCE_DIR}/src/adapters/*")
if(legacy_adapter_files)
  message(FATAL_ERROR
    "core must not own application adapter files: ${legacy_adapter_files}")
endif()

file(READ "${OPEN_LMM_SOURCE_DIR}/CMakeLists.txt" core_cmake)
foreach(forbidden IN ITEMS
    "add_subdirectory(src/adapters" "open_lmm_batch"
    "open_lmm_gui_core" "open_lmm_python_native")
  string(FIND "${core_cmake}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR "core CMake retains application ownership: ${forbidden}")
  endif()
endforeach()

set(component_contract
  "${OPEN_LMM_SOURCE_DIR}/cmake/open_lmm-install-components.txt")
file(READ "${component_contract}" component_contents)
foreach(required IN ITEMS Runtime Development PluginSDK Plugins)
  string(FIND "${component_contents}" "${required}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "core install component is missing: ${required}")
  endif()
endforeach()
foreach(forbidden IN ITEMS Tools Python GuiRuntime GuiApplication ROS)
  string(FIND "${component_contents}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "core install contract must not own ${forbidden}")
  endif()
endforeach()

if(OPEN_LMM_BUILD_DIR)
  foreach(build_contract IN ITEMS
      "${OPEN_LMM_BUILD_DIR}/compile_commands.json"
      "${OPEN_LMM_BUILD_DIR}/OpenLmmConfig.cmake"
      "${OPEN_LMM_BUILD_DIR}/open_lmm-config.cmake")
    if(EXISTS "${build_contract}")
      file(READ "${build_contract}" build_contents)
      foreach(pattern IN ITEMS "/applications/" "/bindings/" "/ros/")
        string(FIND "${build_contents}" "${pattern}" found)
        if(NOT found EQUAL -1)
          message(FATAL_ERROR
            "core build contract contains repository leaf path: "
            "${build_contract}: ${pattern}")
        endif()
      endforeach()
    endif()
  endforeach()
endif()

message(STATUS "core-only architecture ownership verified")
