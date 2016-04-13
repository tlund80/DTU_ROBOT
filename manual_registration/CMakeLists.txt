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
find_package(PCL 1.8 REQUIRED QUIET)
if(PCL_FOUND)
  message(STATUS ">> PCL ${PCL_VERSION} found")
else(PCL_FOUND)
  message(FATAL_ERROR ">> PCL ${PCL_REQUIRED_VERSION} or higher not found!")
endif(PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find the QtWidgets library
#find_package(Qt5Widgets)

find_package(Qt4)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

find_package(VTK 6.3 REQUIRED NO_MODULE)
include(${VTK_USE_FILE})
if(VTK_FOUND)
  message(STATUS ">> VTK ${VTK_VERSION} found")
set(VTK_FOUND TRUE) # VTK uses 1/0 instead of TRUE/FALSE
else(VTK_FOUND)
  set(VTK_FOUND FALSE)
endif(VTK_FOUND)

#find_package(VTK)
#include(${VTK_USE_FILE})

find_package(Boost 1.48 REQUIRED QUIET)
if(Boost_FOUND)
  message(STATUS ">> Boost ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION} found")
else(Boost_FOUND)
  message(FATAL_ERROR ">> Boost ${BOOST_REQUIRED_VERSION} or higher not found!")
endif(Boost_FOUND)

include_directories(${CMAKE_BINARY_DIR})

##
# Add target
##
QT4_WRAP_UI(manual_registration_ui manual_registration.ui)
QT4_WRAP_CPP(manual_registration_moc manual_registration.h)# OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
add_executable(manual_registration ${manual_registration_ui} ${manual_registration_moc} manual_registration.cpp)
target_link_libraries(manual_registration ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${QT_LIBRARIES} ${VTK_LIBRARIES} ${BOOST_LIBRARIES})

# add the install targets
install(TARGETS ${PROJECT_NAME}
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION /usr/local/bin
)

