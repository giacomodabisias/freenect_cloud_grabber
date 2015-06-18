
cmake_minimum_required(VERSION 2.8)
Project (pcl_reader_freenect CXX)
add_executable (pcl_reader_freenect pcl_reader_freenect.cpp )
set(CMAKE_PREFIX_PATH  )

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_library(FREENECT_LIBRARY  NAMES freenect)
include_directories(${CMAKE_PREFIX_PATH}/include/libfreenect )

include_directories(${FREENECT_GRABBER_INCLUDE_DIR} )

target_link_libraries( pcl_reader_freenect  ${PCL_LIBRARIES} ${FREENECT_LIBRARY} )
