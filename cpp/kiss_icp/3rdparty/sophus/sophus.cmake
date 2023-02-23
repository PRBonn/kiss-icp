# MIT License
#
# # Copyright (c) 2023 Saurabh Gupta, Ignacio Vizzo, Cyrill Stachniss, University of Bonn
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

ExternalProject_Add(
  external_sophus
  PREFIX sophus
  URL https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
             -DSOPHUS_USE_BASIC_LOGGING=ON 
             -DBUILD_SOPHUS_EXAMPLES=OFF
             -DBUILD_SOPHUS_TESTS=OFF 
             -DCMAKE_BUILD_TYPE=Release)

ExternalProject_Get_Property(external_sophus SOURCE_DIR)
add_library(SophusHelper INTERFACE)
add_dependencies(SophusHelper external_sophus)
target_compile_definitions(SophusHelper INTERFACE SOPHUS_USE_BASIC_LOGGING=1)
target_include_directories(SophusHelper SYSTEM INTERFACE $<BUILD_INTERFACE:${SOURCE_DIR}>)
set_property(TARGET SophusHelper PROPERTY EXPORT_NAME Sophus::Sophus)
add_library(Sophus::Sophus ALIAS SophusHelper)
