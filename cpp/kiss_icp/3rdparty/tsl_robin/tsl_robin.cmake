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
include(FetchContent)

FetchContent_Declare(tessil URL https://github.com/Tessil/robin-map/archive/refs/tags/v1.0.1.tar.gz)
if(NOT tessil_POPULATED)
  set(BUILD_TESTING OFF)
  FetchContent_Populate(tessil)
  add_library(robin_map INTERFACE)
  add_library(tsl::robin_map ALIAS robin_map)
  target_include_directories(robin_map SYSTEM INTERFACE "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/include>")
  list(APPEND headers "${tessil_SOURCE_DIR}/include/tsl/robin_growth_policy.h"
       "${tessil_SOURCE_DIR}/include/tsl/robin_hash.h" "${tessil_SOURCE_DIR}/include/tsl/robin_map.h"
       "${tessil_SOURCE_DIR}/include/tsl/robin_set.h")
  target_sources(robin_map INTERFACE "$<BUILD_INTERFACE:${headers}>")
endif()
