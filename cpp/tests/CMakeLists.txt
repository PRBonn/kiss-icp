cmake_minimum_required(VERSION 3.16...3.26)
project(kiss_icp_tests LANGUAGES CXX)

add_subdirectory(../kiss_icp kiss_icp)
enable_testing()

add_executable(test_sparse_lidar test_sparse_lidar.cpp)
target_link_libraries(test_sparse_lidar PRIVATE kiss_icp_pipeline)
add_test(NAME sparse_lidar COMMAND test_sparse_lidar)
