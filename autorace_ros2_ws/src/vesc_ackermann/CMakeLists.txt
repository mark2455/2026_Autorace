cmake_minimum_required(VERSION 3.8)
project(vesc_ackermann)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(autorace_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ackermann_to_vesc_node
  src/ackermann_to_vesc_node.cpp
  src/ackermann_to_vesc.cpp
)
target_include_directories(ackermann_to_vesc_node PRIVATE include)
ament_target_dependencies(ackermann_to_vesc_node autorace_ros2_compat rclcpp ackermann_msgs std_msgs)

install(TARGETS ackermann_to_vesc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
