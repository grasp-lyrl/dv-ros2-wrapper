cmake_minimum_required(VERSION 3.14)
project(dv_ros2_capture VERSION 1.3.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv_ros2_messaging REQUIRED)
find_package(dv-processing 2.0.0 REQUIRED)
find_package(rclcpp_components REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuInfo.msg"
  "msg/CameraDiscovery.msg"
  "srv/SetImuInfo.srv"
  "srv/SetImuBiases.srv"
  "srv/SynchronizeCamera.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs builtin_interfaces
)

include_directories(include)

# Component shared library
set(COMPONENT_SOURCE src/parameters_loader.cpp src/capture_node.cpp)

add_library(capture_node_component SHARED ${COMPONENT_SOURCE})

ament_target_dependencies(capture_node_component
  rclcpp rclcpp_components std_msgs sensor_msgs geometry_msgs
  tf2_ros tf2_msgs builtin_interfaces
  dv_ros2_msgs dv_ros2_messaging
)

target_link_libraries(capture_node_component dv::processing)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(capture_node_component "${cpp_typesupport_target}")

rclcpp_components_register_nodes(capture_node_component "dv_capture_node::CaptureNode")

# Standalone executable
add_executable(capture_node src/main.cpp)
target_link_libraries(capture_node capture_node_component)

install(TARGETS capture_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS capture_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
