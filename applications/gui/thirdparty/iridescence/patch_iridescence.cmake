set(iridescence_cmake "${IRIDESCENCE_SOURCE_DIR}/CMakeLists.txt")
file(READ "${iridescence_cmake}" iridescence_contents)

# Iridescence is normally configured as a top-level project. When embedded by
# FetchContent, CMAKE_SOURCE_DIR points at OpenLMM and breaks its data paths and
# CPack README/LICENSE checks.
string(REPLACE
  "\${CMAKE_SOURCE_DIR}"
  "\${CMAKE_CURRENT_SOURCE_DIR}"
  iridescence_contents
  "${iridescence_contents}"
)

file(WRITE "${iridescence_cmake}" "${iridescence_contents}")
