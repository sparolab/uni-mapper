
if(OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR)
  add_subdirectory(erasor)
endif()
if(OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM)
  add_subdirectory(free_dom)
endif()

add_library(remover_offline INTERFACE)
foreach(remover_target IN ITEMS erasor free_dom)
  if(TARGET ${remover_target})
    target_link_libraries(remover_offline INTERFACE ${remover_target})
  endif()
endforeach()
