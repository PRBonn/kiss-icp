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

# Silence timestamp warning
if(CMAKE_VERSION VERSION_GREATER 3.24)
  cmake_policy(SET CMP0135 OLD)
endif()

# CMake arguments for configuring ExternalProjects.
set(ExternalProject_CMAKE_ARGS
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON)

# tsl_robin is fast to fetch, don't ask for system-wise installation
include(${CMAKE_CURRENT_LIST_DIR}/tsl_robin/tsl_robin.cmake)

if(USE_SYSTEM_EIGEN3)
  find_package(Eigen3 QUIET NO_MODULE)
endif()
if(NOT USE_SYSTEM_EIGEN3 OR NOT EIGEN3_FOUND)
  set(USE_SYSTEM_EIGEN3 OFF)
  include(${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake)
endif()

# tbb needs to be statically linked, so, also do it always :)
if(USE_SYSTEM_TBB)
  find_package(TBB QUIET NO_MODULE)
endif()
if(NOT USE_SYSTEM_TBB OR NOT TBB_FOUND)
  set(USE_SYSTEM_TBB OFF)
  include(${CMAKE_CURRENT_LIST_DIR}/tbb/tbb.cmake)
endif()
