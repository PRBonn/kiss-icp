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
cmake_minimum_required(VERSION 3.16...3.24)
project(kiss_icp VERSION 0.1.1 LANGUAGES CXX)

# Set build options
option(BUILD_PYTHON_BINDINGS "Build the python module" OFF)
option(BUILD_ROS_WRAPPER "Build the ROS wrapper module" OFF)

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build always core library
add_subdirectory(src/cpp/kiss_icp)

if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(src/cpp/kiss_icp_pybind)
endif()

find_package(ament_cmake QUIET)
find_package(catkin QUIET)
if(BUILD_ROS_WRAPPER OR ament_cmake_FOUND OR catkin_FOUND)
  add_subdirectory(src/cpp/kiss_icp_ros)
endif()
