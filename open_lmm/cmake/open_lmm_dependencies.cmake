include_guard(GLOBAL)

include(CMakeFindDependencyMacro)

set(_open_lmm_need_heavy FALSE)
if(NOT open_lmm_FIND_COMPONENTS)
  set(_open_lmm_need_heavy TRUE)
else()
  foreach(_open_lmm_component IN LISTS open_lmm_FIND_COMPONENTS)
    if(_open_lmm_component MATCHES "^(common|config|core|runtime|gui|utils)$")
      set(_open_lmm_need_heavy TRUE)
    endif()
  endforeach()
endif()

if(_open_lmm_need_heavy)
  find_dependency(Eigen3)
  find_dependency(PCL)
  find_dependency(GTSAM)
endif()

if(_open_lmm_need_heavy AND NOT TARGET PCL::PCL)
  add_library(PCL::PCL INTERFACE IMPORTED)
  set_target_properties(PCL::PCL PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
  )
endif()
