cmake_minimum_required(VERSION 3.8)
project(usv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate Python and C++ code from the definitions below
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Buoy.msg"
  "msg/BuoyArray.msg"
  "msg/ThrusterPwm.msg"
  "msg/NavStatus.msg"
  "msg/Waypoint.msg"
  "msg/WaypointArray.msg"
  "srv/SetWaypoint.srv"
  "srv/SetTask.srv"
  "srv/SetTargetColor.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()