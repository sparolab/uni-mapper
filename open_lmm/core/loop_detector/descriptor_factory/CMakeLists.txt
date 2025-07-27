
add_subdirectory(kdtree)
# add_subdirectory(hashmap)

add_library(descriptor_factory INTERFACE)
target_link_libraries(descriptor_factory INTERFACE 
  descriptor_kdtree
)

