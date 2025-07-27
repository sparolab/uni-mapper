# MIT License
#
# Copyright (c) 2024 koide3
#   - Originally from the GLIM repository (https://github.com/koide3/GLIM)
#
# Modified in 2025 by Gilhwan Kang
# - Minor changes were made to the original source code.
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

add_library(open_lmm_utils SHARED)
target_sources(open_lmm_utils PRIVATE
  config.cpp
  load_module.cpp
)
target_link_libraries(open_lmm_utils PUBLIC
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)
openlmm_set_global_target_properties(open_lmm_utils)
