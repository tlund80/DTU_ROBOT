cmake_minimum_required(VERSION 2.4.6)
project(cal_ti)

set(CMAKE_BUILD_TYPE MinSizeRel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  robot_msgs
  camera_info_manager
  qt_build
)

find_package(OpenCV REQUIRED)

rosbuild_prepare_qt4(QtCore QtGui) # Add qt components to the list here
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

#find_package(LAPACK REQUIRED)
#include_directories(${LAPACK_INCLUDE_DIRS})

find_package(BLAS REQUIRED)
include_directories(${BLAS_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#add dynamic reconfigure api
#generate_dynamic_reconfigure_options(
#)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   StartAcquisition.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Scan.action
#)
#
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   robot_msgs
#   actionlib_msgs   
#   std_msgs
#   sensor_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robovision_communicator
#  CATKIN_DEPENDS roscpp image_transport cv_bridge std_msgs sensor_msgs robot_msgs camera_info_manager
  DEPENDS opencv
)

###########
## Build ##
###########

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/cal_ti/*.hpp)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

SET(LEVMAR_ROOT_DIR "$ENV{DTI_COMMON_ROOT}/Levmar-2.6")
include_directories(${LEVMAR_ROOT_DIR})

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${QT_SOURCES} 
                               ${QT_RESOURCES_CPP}
                               ${QT_FORMS_HPP}
                               ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${LEVMAR_ROOT_DIR}/build/MinSizeRel/liblevmar.a)
target_link_libraries(${PROJECT_NAME} lapack blas)

