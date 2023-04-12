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
# Simply ignore the warnings when using scikit-build as the build system
if(${SKBUILD})
  set(ignore ${PYTHON_NumPy_INCLUDE_DIRS})
  set(ignore ${Python3_EXECUTABLE})
  set(ignore ${Python3_INCLUDE_DIR})
  set(ignore ${Python3_LIBRARY})
  set(ignore ${Python3_NumPy_INCLUDE_DIRS})
  set(ignore ${Python_EXECUTABLE})
  set(ignore ${Python_INCLUDE_DIR})
  set(ignore ${Python_LIBRARY})
  set(ignore ${Python_NumPy_INCLUDE_DIRS})
  set(ignore ${Python3_FIND_REGISTRY})
  set(ignore ${Python3_ROOT_DIR})
  set(ignore ${Python_FIND_REGISTRY})
  set(ignore ${Python_ROOT_DIR})
endif()

option(USE_SYSTEM_PYBIND11 "Use system pre-installed pybind11" ON)
if(USE_SYSTEM_PYBIND11)
  find_package(pybind11 QUIET)
endif()
if(NOT USE_SYSTEM_PYBIND11 OR NOT pybind11_FOUND)
  set(USE_SYSTEM_PYBIND11 OFF)
  include(${CMAKE_CURRENT_LIST_DIR}/pybind11/pybind11.cmake)
endif()

pybind11_add_module(kiss_icp_pybind MODULE kiss_icp_pybind.cpp)
target_link_libraries(kiss_icp_pybind PRIVATE kiss_icp::core kiss_icp::metrics)
install(TARGETS kiss_icp_pybind DESTINATION . COMPONENT python_bindings)
add_custom_target(install_python_bindings ${CMAKE_COMMAND} -DCMAKE_INSTALL_COMPONENT=python_bindings -P
                                          "${PROJECT_BINARY_DIR}/cmake_install.cmake" DEPENDS kiss_icp_pybind)
