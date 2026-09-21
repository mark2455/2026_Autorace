cmake_minimum_required(VERSION 3.8)
project(asw_lidar)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autorace_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(coss_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation kdtree)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LidarSetting.msg"
  DEPENDENCIES std_msgs
)

add_executable(asw_lidar_node src/main.cpp)
target_include_directories(asw_lidar_node PRIVATE include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
ament_target_dependencies(asw_lidar_node
  autorace_ros2_compat rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs visualization_msgs
  coss_msgs pcl_conversions laser_geometry cv_bridge
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(asw_lidar_node "${cpp_typesupport_target}" ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS asw_lidar_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME} OPTIONAL)
ament_export_include_directories(include)
ament_package()
