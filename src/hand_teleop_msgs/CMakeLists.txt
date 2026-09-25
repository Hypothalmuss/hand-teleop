cmake_minimum_required(VERSION 3.8)
project(hand_teleop_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hand.msg"
  "msg/HandState.msg"
  "msg/TeleopTarget.msg"
  "msg/JointCommand.msg"
  "msg/ObjectPoses.msg"
  "msg/LatencySample.msg"
  "srv/ResetEpisode.srv"
  "srv/EpisodeControl.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
