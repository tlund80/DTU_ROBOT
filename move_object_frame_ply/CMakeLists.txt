cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(dti_move_object_frame)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (${PROJECT_NAME} move_object_frame.cpp)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})

# add the install targets
install(TARGETS ${PROJECT_NAME}
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION /usr/local/bin
)
