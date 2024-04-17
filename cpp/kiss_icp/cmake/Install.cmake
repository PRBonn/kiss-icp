# MIT License
#
# Copyright (c) 2024 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch
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

set(PNAME "KissICP")
include(CMakePackageConfigHelpers)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/${PNAME}ConfigVersion.cmake" VERSION ${PACKAGE_VERSION}
                                 COMPATIBILITY AnyNewerVersion)

install(
  TARGETS kiss_icp_core kiss_icp_metrics kiss_icp_pipeline
  EXPORT ${PNAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(EXPORT ${PNAME}Targets FILE ${PNAME}Targets.cmake DESTINATION lib/cmake/${PNAME})

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PNAME}Config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${PNAME}ConfigVersion.cmake" DESTINATION lib/cmake/${PNAME})
