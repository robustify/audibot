cmake_minimum_required(VERSION 3.5)
project(audibot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

include_directories(include)

add_library(AudibotInterfacePlugin SHARED
  src/AudibotInterfacePlugin.cpp
)
target_link_libraries(AudibotInterfacePlugin
  gz-sim::gz-sim
  gz-plugin::register
)

ament_export_libraries(AudibotInterfacePlugin)
install(
  TARGETS AudibotInterfacePlugin
  DESTINATION lib
)

install(
  DIRECTORY
    config
    launch
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
