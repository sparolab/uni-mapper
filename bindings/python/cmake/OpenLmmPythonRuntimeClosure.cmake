function(openlmm_python_install_runtime_closure core_prefix manifest)
  if(NOT EXISTS "${manifest}")
    message(FATAL_ERROR "Python runtime closure manifest is missing: ${manifest}")
  endif()
  set(core_library_directory "${core_prefix}/${CMAKE_INSTALL_LIBDIR}")
  if(NOT IS_DIRECTORY "${core_library_directory}")
    message(FATAL_ERROR
      "installed core library directory is missing: ${core_library_directory}")
  endif()

  file(STRINGS "${manifest}" closure_rows)
  set(logical_names)
  foreach(row IN LISTS closure_rows)
    if(row STREQUAL "" OR row MATCHES "^kind")
      continue()
    endif()
    string(REPLACE "\t" ";" fields "${row}")
    list(LENGTH fields field_count)
    if(NOT field_count EQUAL 6)
      message(FATAL_ERROR "invalid Python runtime closure row: ${row}")
    endif()
    list(GET fields 0 kind)
    list(GET fields 1 logical_name)
    list(GET fields 2 source_relative_path)
    list(GET fields 3 wheel_relative_path)
    list(GET fields 4 required)
    list(GET fields 5 license)
    if(NOT kind MATCHES "^(runtime|descriptor_plugin|remover_plugin|remover_support)$" OR
       NOT source_relative_path STREQUAL "lib/${logical_name}" OR
       NOT wheel_relative_path STREQUAL "open_lmm/.libs/${logical_name}" OR
       NOT required STREQUAL "true" OR
       NOT license STREQUAL "GPL-3.0-only")
      message(FATAL_ERROR "invalid Python runtime closure contract: ${row}")
    endif()
    if(logical_name IN_LIST logical_names)
      message(FATAL_ERROR "duplicate Python runtime closure row: ${logical_name}")
    endif()
    list(APPEND logical_names "${logical_name}")
    foreach(suffix IN ITEMS "" ".3" ".3.0.0")
      set(source_path "${core_prefix}/${source_relative_path}${suffix}")
      if(NOT EXISTS "${source_path}")
        message(FATAL_ERROR
          "Python runtime closure artifact is missing: ${source_path}")
      endif()
      install(FILES "${source_path}"
        DESTINATION open_lmm/.libs COMPONENT Python)
    endforeach()
  endforeach()

  list(SORT logical_names)
  set(expected_logical_names
    libcreate_dufomap.so
    libcreate_erasor.so
    libcreate_free_dom.so
    libcreate_hmm_mos.so
    libcreate_otd.so
    libcreate_scan_context.so
    libcreate_solid.so
    libdufomap.so
    liberasor.so
    libfree_dom.so
    libhmm_mos.so
    libopen_lmm_algorithm_config.so
    libopen_lmm_backend_optimizer.so
    libopen_lmm_client.so
    libopen_lmm_common.so
    libopen_lmm_contracts.so
    libopen_lmm_data_loader.so
    libopen_lmm_descriptor.so
    libopen_lmm_dynamic_remover.so
    libopen_lmm_loop_detector.so
    libopen_lmm_map_server.so
    libopen_lmm_utils.so
    libotd.so)
  list(SORT expected_logical_names)
  if(NOT logical_names STREQUAL expected_logical_names)
    message(FATAL_ERROR
      "Python runtime logical closure changed\n"
      "expected: ${expected_logical_names}\nactual: ${logical_names}")
  endif()

  file(GLOB installed_plugin_entries
    "${core_library_directory}/libcreate_*.so")
  set(installed_plugin_names)
  foreach(plugin_entry IN LISTS installed_plugin_entries)
    get_filename_component(plugin_name "${plugin_entry}" NAME)
    list(APPEND installed_plugin_names "${plugin_name}")
  endforeach()
  list(SORT installed_plugin_names)
  set(expected_plugin_names
    libcreate_dufomap.so
    libcreate_erasor.so
    libcreate_free_dom.so
    libcreate_hmm_mos.so
    libcreate_otd.so
    libcreate_scan_context.so
    libcreate_solid.so)
  list(SORT expected_plugin_names)
  if(NOT installed_plugin_names STREQUAL expected_plugin_names)
    message(FATAL_ERROR
      "wheel core prefix has the wrong plugin capability profile\n"
      "expected: ${expected_plugin_names}\nactual: ${installed_plugin_names}")
  endif()

endfunction()
