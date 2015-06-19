
cmake_minimum_required(VERSION 2.8)
Project (pcl_reader_freenect CXX)
add_executable (pcl_reader_freenect pcl_reader_freenect.cpp )
set(CMAKE_PREFIX_PATH  )

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DHAVE_OMP)
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 



#######find libusb
if (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
  # in cache already
  set(LIBUSB_FOUND TRUE)
else (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
  find_path(LIBUSB_1_INCLUDE_DIR
    NAMES
  libusb.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  PATH_SUFFIXES
    libusb-1.0
  )

  find_library(LIBUSB_1_LIBRARY
    NAMES
      usb-1.0 usb
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(LIBUSB_1_INCLUDE_DIRS
    ${LIBUSB_1_INCLUDE_DIR}
  )
  set(LIBUSB_1_LIBRARIES
    ${LIBUSB_1_LIBRARY}
)

  if (LIBUSB_1_INCLUDE_DIRS AND LIBUSB_1_LIBRARIES)
     set(LIBUSB_1_FOUND TRUE)
  endif (LIBUSB_1_INCLUDE_DIRS AND LIBUSB_1_LIBRARIES)

  if (LIBUSB_1_FOUND)
    if (NOT libusb_1_FIND_QUIETLY)
      message(STATUS "Found libusb-1.0:")
    message(STATUS " - Includes: ${LIBUSB_1_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${LIBUSB_1_LIBRARIES}")
    endif (NOT libusb_1_FIND_QUIETLY)
  else (LIBUSB_1_FOUND)
    if (libusb_1_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find libusb")
    endif (libusb_1_FIND_REQUIRED)
  endif (LIBUSB_1_FOUND)

  # show the LIBUSB_1_INCLUDE_DIRS and LIBUSB_1_LIBRARIES variables only in the advanced view
  mark_as_advanced(LIBUSB_1_INCLUDE_DIRS LIBUSB_1_LIBRARIES)

endif (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
include_directories(${LIBUSB_1_INCLUDE_DIRS} ${LIBUSB_1_INCLUDE_DIRS}/libusb-1.0)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_library(FREENECT_LIBRARY  NAMES freenect)
include_directories(${CMAKE_PREFIX_PATH}/include/libfreenect )

include_directories(${LIBUSB_1_INCLUDE_DIRS} ${LIBUSB_1_INCLUDE_DIRS}/libusb-1.0)
include_directories(${FREENECT_GRABBER_INCLUDE_DIR} )

target_link_libraries( pcl_reader_freenect  ${PCL_LIBRARIES} ${FREENECT_LIBRARY} )
