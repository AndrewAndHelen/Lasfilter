project(LASFILTER CXX)
cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_subdirectory(${CMAKE_SOURCE_DIR}/LASlib)
add_subdirectory(${CMAKE_SOURCE_DIR}/filter)