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
include(FetchContent)

set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "Don't use fmt for Sophus libraru")
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Don't build Sophus tests")
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "Don't build Sophus Examples")

FetchContent_Declare(sophus SYSTEM URL https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz
                     PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/sophus.patch UPDATE_DISCONNECTED 1)

FetchContent_MakeAvailable(sophus)
