include(FetchContent)

# The upstream project uses Pods and mutates global compiler flags. Fetch its
# pinned sources but expose only the GkCM target used by OpenLMM.
FetchContent_Declare(
  gkcm_source
  GIT_REPOSITORY https://bitbucket.org/jmangelson/gkcm.git
  GIT_TAG        a4aa482d862b1ef608ab9f5f446e58a1e91b994b
)
FetchContent_GetProperties(gkcm_source)
if(NOT gkcm_source_POPULATED)
  FetchContent_Populate(gkcm_source)
endif()

add_library(open_lmm_gkcm STATIC
  ${gkcm_source_SOURCE_DIR}/src/gkcm/gkcm.cpp
)
target_include_directories(open_lmm_gkcm SYSTEM PUBLIC
  ${gkcm_source_SOURCE_DIR}/src
)
target_link_libraries(open_lmm_gkcm PUBLIC Eigen3::Eigen Threads::Threads)
set_target_properties(open_lmm_gkcm PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(GkCM::GkCM ALIAS open_lmm_gkcm)
