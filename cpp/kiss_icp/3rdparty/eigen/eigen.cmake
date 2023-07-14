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

# TODO: Yet another manual release dne by nacho. This should be updated whenever the Eigen team
# release a new version that is not 3.4. That version does not include this necessary changes:
# - https://gitlab.com/libeigen/eigen/-/merge_requests/893/diffs

set(EIGEN_BUILD_DOC OFF CACHE BOOL "Don't build Eigen docs")
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Don't build Eigen tests")
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Don't build Eigen pkg-config")
set(EIGEN_BUILD_BLAS OFF CACHE BOOL "Don't build blas module")
set(EIGEN_BUILD_LAPACK OFF CACHE BOOL "Don't build lapack module")

include(FetchContent)
FetchContent_Declare(eigen SYSTEM URL https://github.com/nachovizzo/eigen/archive/refs/tags/3.4.90.tar.gz)
FetchContent_MakeAvailable(eigen)

if(${CMAKE_VERSION} VERSION_LESS 3.25)
  get_target_property(eigen_include_dirs eigen INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(eigen PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${eigen_include_dirs}")
endif()
