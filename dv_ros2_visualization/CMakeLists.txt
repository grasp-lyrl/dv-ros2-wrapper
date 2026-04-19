cmake_minimum_required(VERSION 3.14)
project(dv_ros2_visualization VERSION 1.3.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv_ros2_messaging REQUIRED)
find_package(dv-processing 2.0.0 REQUIRED)

# Component shared library
add_library(visualization_node_component SHARED src/visualization_node.cpp)
ament_target_dependencies(visualization_node_component
  rclcpp rclcpp_components std_msgs sensor_msgs dv_ros2_msgs dv_ros2_messaging
)
target_link_libraries(visualization_node_component dv::processing)

rclcpp_components_register_nodes(visualization_node_component "dv_visualization_node::VisualizationNode")

# IMU visualization component
add_library(imu_visualization_node_component SHARED src/imu_visualization_node.cpp)
ament_target_dependencies(imu_visualization_node_component
  rclcpp rclcpp_components std_msgs sensor_msgs dv_ros2_msgs dv_ros2_messaging
)
target_link_libraries(imu_visualization_node_component dv::processing)

rclcpp_components_register_nodes(imu_visualization_node_component "dv_visualization_node::ImuVisualizationNode")

install(TARGETS visualization_node_component imu_visualization_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
