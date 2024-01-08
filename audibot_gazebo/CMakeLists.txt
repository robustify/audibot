cmake_minimum_required(VERSION 3.5)
project(audibot_gazebo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(audibot_interface_plugin SHARED
  src/AudibotInterfacePlugin.cpp
)
ament_target_dependencies(audibot_interface_plugin
  rclcpp
  gazebo_ros
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  example_interfaces
)
target_include_directories(audibot_interface_plugin PUBLIC
  include
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(audibot_interface_plugin
  ${GAZEBO_LIBRARIES}
)

### Install Rules
install(
  TARGETS
    audibot_interface_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
