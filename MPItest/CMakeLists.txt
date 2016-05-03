cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(my_mpi)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if(OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${dbscan})
link_directories(${dbscan})

add_executable(my_mpi mpi_test.cpp dbscan/clusters.cpp dbscan/clusters.h dbscan/dbscan.cpp dbscan/dbscan.h dbscan/kdtree2.cpp dbscan/kdtree2.hpp dbscan/utils.cpp dbscan/utils.h filter.cpp filter.h segmentation.cpp segmentation.h)
SET(CMAKE_C_COMPILER mpicc)
SET(CMAKE_CXX_COMPILER mpicxx)
target_link_libraries(my_mpi mpi)
target_link_libraries (my_mpi ${PCL_LIBRARIES})
