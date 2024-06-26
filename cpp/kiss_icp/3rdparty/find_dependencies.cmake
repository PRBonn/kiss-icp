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

function(find_external_dependency PACKAGE_NAME TARGET_NAME INCLUDED_CMAKE_PATH)
  string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UP)
  set(USE_FROM_SYSTEM_OPTION "USE_SYSTEM_${PACKAGE_NAME_UP}")
  if(${${USE_FROM_SYSTEM_OPTION}})
    find_package(${PACKAGE_NAME} QUIET NO_MODULE)
  endif()
  if(NOT ${${USE_FROM_SYSTEM_OPTION}} OR NOT TARGET ${TARGET_NAME})
    set(${USE_FROM_SYSTEM_OPTION} OFF PARENT_SCOPE)
    include(${INCLUDED_CMAKE_PATH})
  endif()
endfunction()

find_external_dependency("Eigen3" "Eigen3::Eigen" "${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake")
find_external_dependency("Sophus" "Sophus::Sophus" "${CMAKE_CURRENT_LIST_DIR}/sophus/sophus.cmake")
find_external_dependency("TBB" "TBB::tbb" "${CMAKE_CURRENT_LIST_DIR}/tbb/tbb.cmake")
find_external_dependency("tsl-robin-map" "tsl::robin_map" "${CMAKE_CURRENT_LIST_DIR}/tsl_robin/tsl_robin.cmake")
