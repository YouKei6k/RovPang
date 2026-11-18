cmake_minimum_required(VERSION 3.8)
project(rovpang_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Sign.msg"
  "msg/DeliveryRequest.msg"
  "msg/DeliveryStatus.msg"
  "srv/SetArmPose.srv"
  "action/Pick.action"
  "action/Place.action"
  "action/GoTo.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
