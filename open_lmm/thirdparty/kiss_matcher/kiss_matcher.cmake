# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


set(USE_SYSTEM_ROBIN OFF CACHE BOOL "Don't build lapack module")

include(FetchContent)
FetchContent_Declare(
    kiss_matcher
    GIT_REPOSITORY https://github.com/MIT-SPARK/KISS-Matcher.git
    GIT_TAG        c7e2d74a3f6c48a365b5d93070d8c44566820ff5
    PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/kiss_matcher.patch UPDATE_DISCONNECTED 1
)
FetchContent_GetProperties(kiss_matcher)
if(NOT kiss_matcher_POPULATED)
  FetchContent_Populate(kiss_matcher)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${kiss_matcher_SOURCE_DIR}/cpp/kiss_matcher ${kiss_matcher_BINARY_DIR}/kiss_matcher SYSTEM EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
    # consider this 3rdparty headers as source code and fail due the -Werror flag.
    add_subdirectory(${kiss_matcher_SOURCE_DIR}/cpp/kiss_matcher ${kiss_matcher_BINARY_DIR}/kiss_matcher EXCLUDE_FROM_ALL)
    get_target_property(kiss_matcher_include_dirs kiss_matcher INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(kiss_matcher PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${kiss_matcher_include_dirs}")
  endif()
endif()