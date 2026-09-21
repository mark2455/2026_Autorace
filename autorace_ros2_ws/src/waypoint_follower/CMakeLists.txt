cmake_minimum_required(VERSION 3.8)
project(waypoint_follower)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autorace_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(waypoint_maker REQUIRED)
find_package(coss_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Parking.msg")

add_executable(waypoint_follower_node
  src/main.cpp
  src/process.cpp
  src/pure_pursuit.cpp
)
target_include_directories(waypoint_follower_node PRIVATE include)
ament_target_dependencies(waypoint_follower_node
  autorace_ros2_compat rclcpp ackermann_msgs geometry_msgs nav_msgs sensor_msgs std_msgs tf2 waypoint_maker coss_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_follower_node "${cpp_typesupport_target}")

install(TARGETS waypoint_follower_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
ament_export_include_directories(include)
ament_package()
