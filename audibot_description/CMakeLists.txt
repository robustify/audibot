cmake_minimum_required(VERSION 3.5)
project(audibot_description)

find_package(ament_cmake REQUIRED)

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
  DESTINATION share/${PROJECT_NAME}
  PATTERN *.tar.xz EXCLUDE
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
