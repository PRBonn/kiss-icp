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
option(BUILD_SHARED_LIBS OFF)
option(TBBMALLOC_BUILD OFF)
option(TBB_EXAMPLES OFF)
option(TBB_STRICT OFF)
option(TBB_TEST OFF)

include(FetchContent)
FetchContent_Declare(tbb URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.8.0.tar.gz)
FetchContent_GetProperties(tbb)
if(NOT tbb_POPULATED)
  FetchContent_Populate(tbb)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${tbb_SOURCE_DIR} ${tbb_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
    # consider this 3rdparty headers as source code and fail due the -Werror flag.
    add_subdirectory(${tbb_SOURCE_DIR} ${tbb_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(tbb_include_dirs tbb INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(tbb PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${tbb_include_dirs}")
  endif()
endif()
