add_subdirectory(state)
add_subdirectory(resources)
add_subdirectory(execution)
add_subdirectory(control)
add_subdirectory(service)
add_subdirectory(composition)
add_subdirectory(client)

include("${CMAKE_CURRENT_LIST_DIR}/runtime_facade.cmake")
