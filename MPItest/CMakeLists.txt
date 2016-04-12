cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(my_mpi)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${include})
link_directories(${include})

add_executable(my_mpi mpi_test.cpp)
SET(CMAKE_C_COMPILER mpicc)
SET(CMAKE_CXX_COMPILER mpicxx)
target_link_libraries(my_mpi mpi)
target_link_libraries (my_mpi ${PCL_LIBRARIES})
