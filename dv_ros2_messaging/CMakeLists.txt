cmake_minimum_required(VERSION 3.14)
project(dv_ros2_messaging VERSION 1.3.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dv_ros2_msgs REQUIRED)
find_package(dv-processing 2.0.0 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp sensor_msgs dv_ros2_msgs
)
target_link_libraries(${PROJECT_NAME} INTERFACE dv::processing)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_messaging tests/messaging_tests.cpp)
  target_include_directories(test_messaging PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_messaging rclcpp sensor_msgs dv_ros2_msgs)
  target_link_libraries(test_messaging dv::processing)
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs dv_ros2_msgs)

ament_package()
