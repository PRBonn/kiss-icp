cmake_minimum_required(VERSION 3.18.0)
project(kiss_icp_include_example)

find_package(kiss_icp REQUIRED CONFIG)

add_executable(example src/example.cpp)
target_link_libraries(example PRIVATE kiss_icp::kiss_icp_pipeline)
#for other components (not used in example.cpp):
#target_link_libraries(example PRIVATE kiss_icp::kiss_icp_core)
#target_link_libraries(example PRIVATE kiss_icp::kiss_icp_metrics)
