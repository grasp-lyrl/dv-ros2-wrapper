cmake_minimum_required(VERSION 3.14)
project(dv_ros2_msgs VERSION 1.3.0)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Event.msg"
  "msg/EventArray.msg"
  "msg/Trigger.msg"
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
