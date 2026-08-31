add_library(open_lmm_alignment STATIC
  alignment_decision_policy.cpp
  alignment_proposer.cpp
  loop_constraint_builder.cpp
)
target_link_libraries(open_lmm_alignment PUBLIC
  open_lmm_common
  Eigen3::Eigen
  PCL::PCL
)
openlmm_set_global_target_properties(open_lmm_alignment)
set_target_properties(open_lmm_alignment PROPERTIES EXPORT_NAME alignment)
