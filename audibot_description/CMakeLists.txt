cmake_minimum_required(VERSION 3.0.2)
project(audibot_description)

find_package(catkin REQUIRED)

catkin_package()

# Extract *.tar.xz files in the mesh folder
FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/meshes" "${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.tar.xz")
FOREACH(filename ${files})
  MESSAGE(STATUS "Extracting file: ${filename}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${filename}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/meshes
  )
ENDFOREACH(filename)

### Install Rules
install(
  DIRECTORY
    meshes
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN *.tar.xz EXCLUDE
)
