##
# Require CMake
##
cmake_minimum_required(VERSION 2.8)

##
# Project
##
project(feature_matching_benchmark)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

##
# Set release mode
##
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel" FORCE)
endif()

##
# Find CoViS (add your own hint paths here if necessary)
##
find_package(COVIS HINTS ../../covis/build ../../covis-new/build ~/libs/CoViS_new/covis/build)

##
# Setup stuff
##
include_directories(${COVIS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(${COVIS_DEFINITIONS})

#include_directories($ENV{HALCONROOT}/include)

##
# Add targets
##

# Lib
add_library(feature_matching_dataset_lib SHARED
            feature_matching_tools.cpp
            feature_matching_features.cpp
            feature_matching_metrics.cpp)
            #halcon/SurfaceModelDetector.cpp 
	    #halcon/SurfaceModelCreator.cpp
	    #halcon/Create3DObjectModel.cpp)
            
target_link_libraries(feature_matching_dataset_lib ${COVIS_LIBRARIES})# $ENV{HALCONROOT}/lib/$ENV{HALCONARCH}/libhalconcpp.so)

# Apps
add_executable(noisify misc/noisify.cpp)
add_executable(delaunay misc/delaunay.cpp)
add_executable(feature_matching_dataset feature_matching_dataset.cpp)
add_executable(feature_matching_recognition feature_matching_recognition.cpp)
add_executable(feature_matching_timing feature_matching_timing.cpp)

target_link_libraries(noisify feature_matching_dataset_lib ${COVIS_LIBRARIES})
target_link_libraries(delaunay feature_matching_dataset_lib ${COVIS_LIBRARIES})
target_link_libraries(feature_matching_dataset feature_matching_dataset_lib ${COVIS_LIBRARIES})
target_link_libraries(feature_matching_recognition feature_matching_dataset_lib ${COVIS_LIBRARIES})
target_link_libraries(feature_matching_timing feature_matching_dataset_lib ${COVIS_LIBRARIES})
