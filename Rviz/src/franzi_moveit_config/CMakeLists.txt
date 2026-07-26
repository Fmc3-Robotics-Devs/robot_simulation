cmake_minimum_required(VERSION 3.22)
project(franzi_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES .setup_assistant README.md DESTINATION share/${PROJECT_NAME})

ament_package()
