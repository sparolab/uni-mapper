add_library(open_lmm_runtime_client_objects OBJECT
  runtime_client.cpp
  runtime_retirement_coordinator.cpp
)
target_link_libraries(open_lmm_runtime_client_objects PRIVATE
  open_lmm_contracts
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)
target_include_directories(open_lmm_runtime_client_objects PRIVATE
  "$<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>"
  "$<TARGET_PROPERTY:PCL::PCL,INTERFACE_INCLUDE_DIRECTORIES>"
  "$<TARGET_PROPERTY:gtsam,INTERFACE_INCLUDE_DIRECTORIES>"
)
openlmm_set_global_target_properties(open_lmm_runtime_client_objects)
