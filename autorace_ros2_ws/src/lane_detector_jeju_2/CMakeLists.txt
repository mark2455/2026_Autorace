cmake_minimum_required(VERSION 3.8)
project(lane_detector_jeju_2)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(autorace_ros2_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lane_detector_jeju_2
  src/main.cpp
  src/LinePointDetector.cpp
  src/LaneDetector.cpp
  src/InToOutLaneDetector.cpp
  src/LaneDetectorNode.cpp
  src/MyException.cpp
)
target_include_directories(lane_detector_jeju_2 PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(lane_detector_jeju_2
  autorace_ros2_compat rclcpp std_msgs sensor_msgs ackermann_msgs cv_bridge
)
target_link_libraries(lane_detector_jeju_2 ${OpenCV_LIBRARIES})

install(TARGETS lane_detector_jeju_2 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
ament_package()
