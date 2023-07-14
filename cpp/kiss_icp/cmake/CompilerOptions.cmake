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
function(set_global_target_properties target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_definitions(${target} PUBLIC $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:_USE_MATH_DEFINES>)
  target_compile_options(
    ${target}
    PRIVATE # MSVC
            $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/W4>
            $<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/WX>
            # Clang/AppleClang
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-fcolor-diagnostics>
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Werror>
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wall>
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wextra>
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wconversion>
            $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wno-sign-conversion>
            # GNU
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-fdiagnostics-color=always>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Werror>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wextra>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-pedantic>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wcast-align>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wcast-qual>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wconversion>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wdisabled-optimization>
            $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Woverloaded-virtual>)
  set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR})
  get_filename_component(INCLUDE_DIRS ${INCLUDE_DIRS} PATH)
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                             PUBLIC $<BUILD_INTERFACE:${INCLUDE_DIRS}> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
endfunction()
