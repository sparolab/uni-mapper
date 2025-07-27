
add_subdirectory(online)
add_subdirectory(offline)

add_library(dynamic_removal_factory INTERFACE)
target_link_libraries(dynamic_removal_factory INTERFACE 
  remover_online
  remover_offline
)