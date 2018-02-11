cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test_writeRandom)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable (test_writeRandom test_writeRandom.cpp)
target_link_libraries (test_writeRandom pcd_writeRandom ${PCL_LIBRARIES})
