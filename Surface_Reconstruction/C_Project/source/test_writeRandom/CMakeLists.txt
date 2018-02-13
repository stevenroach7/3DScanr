cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test_writeRandom)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(${PCL_DEFINITIONS})
add_library(pcd_writeRandom pcd_writeRandom.cpp)

add_executable (test_writeRandom test_writeRandom.cpp)
target_link_libraries (test_writeRandom ${PCL_LIBRARIES})
