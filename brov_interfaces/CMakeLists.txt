cmake_minimum_required(VERSION 3.8)
project(brov_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AlignedOdometry.msg"
  "msg/BrovState.msg"
  "msg/DesiredState.msg"
  "msg/DvlSample.msg"
  "msg/LocalizationStatus.msg"
  "msg/Observation.msg"
  "msg/OdometrySession.msg"
  "msg/ResolvedMission.msg"
  "msg/Wrench6.msg"
  "srv/InitializePool.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
