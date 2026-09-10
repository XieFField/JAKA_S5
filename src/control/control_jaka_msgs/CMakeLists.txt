cmake_minimum_required(VERSION 3.8)
project(control_jaka_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointMoveStatus.msg"
  "msg/RobotDiagnostic.msg"
  "srv/JogJoint.srv"
  "srv/MoveJoint.srv"
  "srv/SetRapidRate.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
