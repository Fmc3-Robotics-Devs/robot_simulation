cmake_minimum_required(VERSION 3.22)
project(franzi_engraving_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MachineStatus.msg
  msg/RobotSignals.msg
  msg/DockStatus.msg
  msg/TaskState.msg
  action/PreciseDock.action
  action/DetectMaterial.action
  action/PickMaterial.action
  action/LoadMachine.action
  action/UnloadMachine.action
  action/PlaceProduct.action
  action/MoveToSafePose.action
  srv/SetClamp.srv
  srv/SetDoor.srv
  srv/StartMachining.srv
  srv/ResetFault.srv
  srv/ClearTask.srv
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
