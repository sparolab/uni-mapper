add_library(open_lmm_descriptor SHARED
  descriptor_artifact.cpp
  descriptor_engine.cpp
  generic_descriptor_v2_adapter.cpp
)
target_link_libraries(open_lmm_descriptor
  PUBLIC
    Eigen3::Eigen
    PCL::PCL
    open_lmm_common
)
openlmm_set_global_target_properties(open_lmm_descriptor)
set_target_properties(open_lmm_descriptor PROPERTIES EXPORT_NAME descriptor)
