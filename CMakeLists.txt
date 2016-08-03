cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Release as default
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

project (vicon_pose_correction C CXX)

#set(CMAKE_VERBOSE_MAKEFILE ON)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# all errors
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

# set cmake module search path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)


# find packages
	find_package(Eigen3 REQUIRED)
	include_directories(AFTER "${EIGEN3_INCLUDE_DIR}")
find_package(GTSAM REQUIRED)


# enable SSE2 instructions for dlib
SET(DUSE_SSE2_INSTRUCTIONS ON)
SET(DUSE_SSE4_INSTRUCTIONS ON)
SET(DUSE_AVX_INSTRUCTIONS ON)

# Set output folders
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Create "${CMAKE_BINARY_DIR}/bin"
FILE(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
FILE(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Add the actual source files
add_subdirectory (src)
