##
# Require CMake
##
cmake_minimum_required(VERSION 2.8)

##
# Project
##
project(manual_registration)

##
# Find dependencies
##
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Qt4)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

find_package(VTK)
include(${VTK_USE_FILE})

find_package(Boost 1.48 REQUIRED)

include_directories(${CMAKE_BINARY_DIR})

##
# Add target
##
QT4_WRAP_UI(manual_registration_ui manual_registration.ui)
QT4_WRAP_CPP(manual_registration_moc manual_registration.h)# OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
add_executable(manual_registration ${manual_registration_ui} ${manual_registration_moc} manual_registration.cpp)
target_link_libraries(manual_registration pcl_common pcl_io pcl_visualization pcl_filters pcl_search pcl_sample_consensus pcl_registration pcl_segmentation pcl_features pcl_surface QVTK ${QT_LIBRARIES} ${VTK_LIBRARIES} ${BOOST_LIBRARIES})

# add the install targets
install(TARGETS ${PROJECT_NAME}
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION /usr/local/bin
)

