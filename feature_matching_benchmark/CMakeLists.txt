##
# Require CMake
##
cmake_minimum_required(VERSION 2.8)

##
# Project
##
project(feature_matching_benchmark)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
##
# Set release mode
##
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel" FORCE)
endif()

##
# Find CoViS (add your own hint paths here if necessary)
##
find_package(COVIS HINTS ../../covis/build ../../covis-new/build ~/libs/covis/covis/build)

find_package(VTK 6.3 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})

find_package(Halcon REQUIRED)

if (Halcon_FOUND)
  message(STATUS ">> Halcon found")
endif(Halcon_FOUND)
##
# Setup stuff
##
include_directories(${COVIS_INCLUDE_DIRS} ${HALCON_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(${COVIS_DEFINITIONS})

##
# Add targets
##

# Lib
add_library(feature_matching_dataset_lib SHARED
            feature_matching_tools.cpp
            feature_matching_features.cpp
            feature_matching_metrics.cpp)
            
target_link_libraries(feature_matching_dataset_lib ${COVIS_LIBRARIES} ${HALCON_LIBRARIES} ${VTK_LIBRARIES})

# Apps
add_executable(noisify misc/noisify.cpp)
add_executable(delaunay misc/delaunay.cpp)
add_executable(feature_matching_dataset feature_matching_dataset.cpp)
add_executable(feature_matching_recognition feature_matching_recognition.cpp halcon/Create3DObjectModel.cpp halcon/SurfaceModelCreator.cpp halcon/SurfaceModelDetector.cpp)
add_executable(feature_matching_timing feature_matching_timing.cpp)

target_link_libraries(noisify feature_matching_dataset_lib ${COVIS_LIBRARIES})
target_link_libraries(delaunay feature_matching_dataset_lib ${COVIS_LIBRARIES})
target_link_libraries(feature_matching_dataset feature_matching_dataset_lib ${COVIS_LIBRARIES})
target_link_libraries(feature_matching_recognition feature_matching_dataset_lib ${COVIS_LIBRARIES} ${HALCON_LIBRARIES})
target_link_libraries(feature_matching_timing feature_matching_dataset_lib ${COVIS_LIBRARIES})
