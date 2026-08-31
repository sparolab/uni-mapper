# Goal 06 builds a same-image local wheel. Public manylinux and third-party DSO
# closure remain Goal 09 work. Packaging owns DSO staging; the Python adapter
# itself remains a leaf linked only to open_lmm_client.
set(_open_lmm_python_runtime_targets
  open_lmm_contracts
  open_lmm_client
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_descriptor
  open_lmm_alignment
  open_lmm_map_server
  open_lmm_data_loader
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover)
foreach(python_runtime_target IN LISTS _open_lmm_python_runtime_targets)
  if(TARGET ${python_runtime_target})
    install(TARGETS ${python_runtime_target}
      LIBRARY DESTINATION open_lmm/.libs COMPONENT Python
      RUNTIME DESTINATION open_lmm/.libs COMPONENT Python)
  endif()
endforeach()

foreach(python_plugin_target IN LISTS OPEN_LMM_CREATE_LIBRARIES)
  if(TARGET ${python_plugin_target})
    install(TARGETS ${python_plugin_target}
      LIBRARY DESTINATION open_lmm/.libs COMPONENT Python
      RUNTIME DESTINATION open_lmm/.libs COMPONENT Python)
  endif()
endforeach()

foreach(python_plugin_support IN ITEMS
    scan_context solid hmm_mos dufomap otd free_dom erasor)
  if(TARGET ${python_plugin_support})
    install(TARGETS ${python_plugin_support}
      LIBRARY DESTINATION open_lmm/.libs COMPONENT Python
      RUNTIME DESTINATION open_lmm/.libs COMPONENT Python)
  endif()
endforeach()
