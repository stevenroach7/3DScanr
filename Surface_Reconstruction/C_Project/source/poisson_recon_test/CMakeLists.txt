cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


project(POISSON_RECON_TEST)



set(PCL_DIR "/Users/greg/code/pclsuperbuild/PCLSuperBuild-build/PCL-build")


find_package(PCL 1.5 REQUIRED COMPONENTS common io surface features search filters)


include_directories(${PCL_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})


add_definitions(${PCL_DEFINITIONS})


add_executable(poisson_recon_test poisson_recon.cpp)


target_link_libraries(poisson_recon_test ${PCL_LIBRARIES})