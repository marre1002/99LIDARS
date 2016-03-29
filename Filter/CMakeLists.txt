cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(filter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${include})
link_directories(${include})

add_executable (filter filter.cpp include/util.h)
target_link_libraries (filter ${PCL_LIBRARIES})
