cmake_minimum_required(VERSION 3.8)
project(franzi_description)

find_package(ament_cmake REQUIRED)

# Install the complete robot description so package://franzi_description/...
# URIs resolve in RViz and the MoveIt Setup Assistant after colcon build.
install(
  DIRECTORY config meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
