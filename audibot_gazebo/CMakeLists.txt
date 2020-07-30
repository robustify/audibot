cmake_minimum_required(VERSION 3.0.2)
project(audibot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf2_ros
  tf2_geometry_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES audibot_interface_plugin
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
)

add_library(audibot_interface_plugin
  src/AudibotInterfacePlugin.cpp
)
add_dependencies(audibot_interface_plugin
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(audibot_interface_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

### Install Rules
install(
  TARGETS
    audibot_interface_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS
    scripts/tf_prefixer.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

