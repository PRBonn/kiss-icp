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

if(NOT DOWNLOAD_MISSING_DEPS)
  find_package(Eigen3 REQUIRED) # sudo apt install libeigen3-dev
  find_package(Sophus REQUIRED) # sudo apt install ros-noetic-sophus
  find_package(TBB REQUIRED) # sudo apt install libtbb-dev
  # clone & install from https://github.com/Tessil/robin-map.git into misc_ws, and run `cmake -Bbuild && cmake --build build && sudo cmake --install build`)
  find_package(tsl-robin-map REQUIRED)

else()
  find_package(Eigen3)
  find_package(Sophus)
  find_package(TBB)
  find_package(tsl-robin-map)
endif()

if(TARGET Eigen3::Eigen)
  message("Found package Eigen3 locally")
endif()
if(TARGET Sophus::Sophus)
  message("Found package Sophus locally")
endif()
if(TARGET TBB::tbb)
  message("Found package TBB locally")
endif()
if(TARGET tsl::robin_map)
  message("Found package tsl-robin-map locally")
endif()

if(NOT (TARGET Eigen3::Eigen AND TARGET Sophus::Sophus AND TARGET tsl::robin_map AND TARGET TBB::tbb))
  message(
    WARNING
      "
  Exporting fetched dependencies is currently broken
  I have no idea how to do it automatically ¯\\_(ツ)_/¯
  If you want to do this, please set DOWNLOAD_MISSING_DEPS to OFF in the main CMakeLists.txt and install the dependencies yourself.
  The find_dependencies file lists the corresponding installation instructions.
  ")
endif()

if(DOWNLOAD_MISSING_DEPS)
  if(NOT TARGET Eigen3::Eigen)
    message("Trying to download external dependency Eigen3...")
    find_external_dependency("Eigen3" "Eigen3::Eigen" "${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake")
    message("done.")
    if(NOT TARGET Eigen3::Eigen)
      message(FATAL_ERROR "loading Eigen3::Eigen failed.")
    endif()
  endif()
  if(NOT TARGET Sophus::Sophus)
    message("Trying to download external dependency Sophus...")
    find_external_dependency("Sophus" "Sophus::Sophus" "${CMAKE_CURRENT_LIST_DIR}/sophus/sophus.cmake")
    message("done.")
    if(NOT TARGET Sophus::Sophus)
      message(FATAL_ERROR "loading Sophus::Sophus failed.")
    endif()
  endif()
  if(NOT TARGET TBB::tbb)
    message("Trying to download external dependency TBB...")
    find_external_dependency("TBB" "TBB::tbb" "${CMAKE_CURRENT_LIST_DIR}/tbb/tbb.cmake")
    message("done.")
    if(NOT TARGET TBB::tbb)
      message(FATAL_ERROR "loading TBB:tbb failed.")
    endif()
  endif()
  if(NOT TARGET tsl::robin_map)
    message("Trying to download external dependency tsl-robin-map...")
    find_external_dependency("tsl-robin-map" "tsl::robin_map" "${CMAKE_CURRENT_LIST_DIR}/tsl_robin/tsl_robin.cmake")
    message("done.")
    if(NOT TARGET tsl::robin_map)
      message(FATAL_ERROR "loading tsl::robin_map failed.")
    endif()
  endif()
endif()
