set(point_cloud_header
    "${UFOMAP_SOURCE_DIR}/include/ufo/map/point_cloud.hpp")
file(READ "${point_cloud_header}" point_cloud_contents)

# basic_istringstream cannot be assigned from std::string. GCC accepted this
# through a non-standard conversion path at the pinned revision, while Clang
# correctly rejects it. Reset the existing stream before parsing each line.
set(old_statement "\t\tiss = line;")
set(new_statement "\t\tiss.clear();\n\t\tiss.str(line);")
string(FIND "${point_cloud_contents}" "${old_statement}" statement_position)
if(statement_position EQUAL -1)
  string(FIND "${point_cloud_contents}" "${new_statement}" patched_position)
  if(patched_position EQUAL -1)
    message(FATAL_ERROR
      "Pinned UFOMap source no longer contains the expected PTS parser")
  endif()
else()
  string(REPLACE "${old_statement}" "${new_statement}"
         point_cloud_contents "${point_cloud_contents}")
  file(WRITE "${point_cloud_header}" "${point_cloud_contents}")
endif()
