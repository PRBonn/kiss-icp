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

include(3rdparty/utils.cmake)

# Eigen
find_external_dependecy("USE_SYSTEM_EIGEN3" "Eigen3" "Eigen3::Eigen" "${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake")

# Sophus
find_external_dependecy("USE_SYSTEM_SOPHUS" "Sophus" "Sophus::Sophus" "${CMAKE_CURRENT_LIST_DIR}/sophus/sophus.cmake")

# tbb needs to be statically linked, so, also do it always :)
find_external_dependecy("USE_SYSTEM_TBB" "TBB" "TBB::tbb" "${CMAKE_CURRENT_LIST_DIR}/tbb/tbb.cmake")

# tsl_robin
find_external_dependecy("USE_SYSTEM_TSLMAP" "tsl-robin-map" "tsl::robin_map" "${CMAKE_CURRENT_LIST_DIR}/tsl_robin/tsl_robin.cmake")