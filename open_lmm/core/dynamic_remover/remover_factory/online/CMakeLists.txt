
add_subdirectory(hmm_mos)
add_subdirectory(dufomap)
add_subdirectory(otd)

add_library(remover_online INTERFACE)
target_link_libraries(remover_online INTERFACE
  hmm_mos
  dufomap
  otd
)