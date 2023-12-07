cmake_minimum_required(VERSION 3.5)
project(robotiq_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "srv/CModelResponse.srv"
  "msg/CModelCommand.msg"
  "msg/CModelStatus.msg"
  DEPENDENCIES
  control_msgs
  std_msgs
)

ament_package()
