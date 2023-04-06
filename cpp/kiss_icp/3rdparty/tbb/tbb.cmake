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
include(ExternalProject)
include(GNUInstallDirs)
find_package(Threads)

ExternalProject_Add(
  external_tbb
  PREFIX tbb
  URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.8.0.tar.gz
  URL_HASH SHA256=eee380323bb7ce864355ed9431f85c43955faaae9e9bce35c62b372d7ffd9f8b
  UPDATE_COMMAND ""
  CMAKE_ARGS # CMake flags:
             -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
             -DCMAKE_BUILD_TYPE=Release
             # custom flags:
             -DBUILD_SHARED_LIBS=OFF
             -DTBB_EXAMPLES=OFF
             -DTBB_STRICT=OFF
             -DTBB_TEST=OFF)

ExternalProject_Get_Property(external_tbb INSTALL_DIR)
add_library(TBBHelper INTERFACE)
add_dependencies(TBBHelper external_tbb)
target_include_directories(TBBHelper SYSTEM INTERFACE ${INSTALL_DIR}/${CMAKE_INSTALL_INCLUDEDIR})
target_link_directories(TBBHelper INTERFACE ${INSTALL_DIR}/${CMAKE_INSTALL_LIBDIR})
target_link_libraries(TBBHelper INTERFACE tbb Threads::Threads)
add_library(TBB::tbb ALIAS TBBHelper)
