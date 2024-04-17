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

set(EIGEN_BUILD_DOC OFF CACHE BOOL "Don't build Eigen docs")
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Don't build Eigen tests")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Don't build Eigen pkg-config")
set(EIGEN_BUILD_BLAS OFF CACHE BOOL "Don't build blas module")
set(EIGEN_BUILD_LAPACK OFF CACHE BOOL "Don't build lapack module")

include(FetchContent)
FetchContent_Declare(eigen URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
                     PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/eigen.patch UPDATE_DISCONNECTED 1)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${eigen_SOURCE_DIR} ${eigen_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
    # consider this 3rdparty headers as source code and fail due the -Werror flag.
    add_subdirectory(${eigen_SOURCE_DIR} ${eigen_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(eigen_include_dirs eigen INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(eigen PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${eigen_include_dirs}")
  endif()
endif()
