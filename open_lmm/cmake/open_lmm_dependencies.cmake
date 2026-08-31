include(CMakeFindDependencyMacro)

set(_open_lmm_need_heavy FALSE)
set(_open_lmm_need_eigen FALSE)
if(NOT open_lmm_FIND_COMPONENTS AND NOT _open_lmm_targets_loaded)
  set(_open_lmm_need_heavy TRUE)
  set(_open_lmm_need_eigen TRUE)
elseif(open_lmm_FIND_COMPONENTS)
  foreach(_open_lmm_component IN LISTS open_lmm_FIND_COMPONENTS)
    if(_open_lmm_component MATCHES "^(common|config|core|runtime|utils)$")
      set(_open_lmm_need_heavy TRUE)
      set(_open_lmm_need_eigen TRUE)
    elseif(_open_lmm_component MATCHES "^(client|gui)$")
      set(_open_lmm_need_eigen TRUE)
    endif()
  endforeach()
endif()

if(_open_lmm_need_eigen)
  find_dependency(Eigen3)
endif()

if(_open_lmm_need_heavy)
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

# ament exports all stable façade targets in one file. Lightweight component
# consumers still have to parse link interfaces for unrequested heavy
# façades, so provide inert parse-only targets without discovering PCL/GTSAM.
# Requested client/gui targets do not link either placeholder.
if(NOT _open_lmm_need_heavy)
  if(NOT TARGET PCL::PCL)
    add_library(PCL::PCL INTERFACE IMPORTED)
  endif()
  if(NOT TARGET gtsam)
    add_library(gtsam INTERFACE IMPORTED)
  endif()
endif()

# ament's generated package config does not call check_required_components().
# Publish the same component contract as the standalone package config so
# component-qualified consumers do not get a false negative before the
# exported-targets extra is loaded.
foreach(_open_lmm_component IN LISTS open_lmm_FIND_COMPONENTS)
  if(_open_lmm_component MATCHES
      "^(contracts|client|plugin_sdk|common|config|descriptor|alignment|core|runtime|utils)$")
    set(open_lmm_${_open_lmm_component}_FOUND TRUE)
  else()
    set(open_lmm_${_open_lmm_component}_FOUND FALSE)
  endif()
endforeach()
