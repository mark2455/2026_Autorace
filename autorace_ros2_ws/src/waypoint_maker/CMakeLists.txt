cmake_minimum_required(VERSION 3.8)
project(waypoint_maker)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autorace_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Coss.msg"
  "msg/Lane.msg"
  "msg/State.msg"
  "msg/Waypoint.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(waypoint_loader src/waypoint_loader.cpp src/cubic_spline_planner.cpp)
target_include_directories(waypoint_loader PRIVATE include)
ament_target_dependencies(waypoint_loader autorace_ros2_compat rclcpp std_msgs geometry_msgs nav_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_loader "${cpp_typesupport_target}")

add_executable(waypoint_saver src/waypoint_saver.cpp)
target_include_directories(waypoint_saver PRIVATE include)
ament_target_dependencies(waypoint_saver autorace_ros2_compat rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(waypoint_saver "${cpp_typesupport_target}")

install(TARGETS waypoint_loader waypoint_saver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz data DESTINATION share/${PROJECT_NAME} OPTIONAL)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)
ament_package()
