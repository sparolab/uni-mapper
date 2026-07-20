
add_subdirectory(erasor)
add_subdirectory(free_dom)

add_library(remover_offline INTERFACE)
target_link_libraries(remover_offline INTERFACE
  erasor
  free_dom
)
