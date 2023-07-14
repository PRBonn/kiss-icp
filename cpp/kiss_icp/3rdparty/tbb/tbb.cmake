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
# option(BUILD_SHARED_LIBS ON)
option(BUILD_SHARED_LIBS OFF)
option(TBBMALLOC_BUILD OFF)
option(TBB_EXAMPLES OFF)
option(TBB_STRICT OFF)
option(TBB_TEST OFF)

include(FetchContent)
FetchContent_Declare(tbb SYSTEM URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.8.0.tar.gz)
FetchContent_MakeAvailable(tbb)

if(${CMAKE_VERSION} VERSION_LESS 3.25)
  get_target_property(tbb_include_dirs tbb INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(tbb PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${tbb_include_dirs}")
endif()
