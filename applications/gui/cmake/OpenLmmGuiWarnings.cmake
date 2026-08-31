function(openlmm_gui_enable_warnings target)
  target_compile_options(${target} PRIVATE
    $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall;-Wextra;-Wpedantic>
    $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic>
    $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/W4>)
  if(OPEN_LMM_GUI_WARNINGS_AS_ERRORS)
    set_property(TARGET ${target} PROPERTY COMPILE_WARNING_AS_ERROR ON)
  endif()
endfunction()
