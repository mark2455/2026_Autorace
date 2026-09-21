cmake_minimum_required(VERSION 3.0.2)
project(final)
find_package(catkin REQUIRED COMPONENTS
pcl_ros
rospy
roscpp
sensor_msgs
cv_bridge
image_transport
geometry_msgs
nav_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
)
include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

add_executable(detection src/detection.cpp)
target_link_libraries(detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(vox src/vox.cpp)
target_link_libraries(vox ${catkin_LIBRARIES})

add_executable(filter src/filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})

add_executable(filter_R src/filter_R.cpp)
target_link_libraries(filter_R ${catkin_LIBRARIES})

add_executable(filter_L src/filter_L.cpp)
target_link_libraries(filter_L ${catkin_LIBRARIES})

add_executable(filter_mean src/filter_mean.cpp)
target_link_libraries(filter_mean ${catkin_LIBRARIES})


add_executable(lane_R src/lane_R.cpp)
target_link_libraries(lane_R ${catkin_LIBRARIES})

add_executable(lane_L src/lane_L.cpp)
target_link_libraries(lane_L ${catkin_LIBRARIES})

add_executable(mean_view src/mean_view.cpp)
target_link_libraries(mean_view ${catkin_LIBRARIES})

add_executable(view src/view.cpp)
target_link_libraries(view ${catkin_LIBRARIES})

add_executable(view_R src/view_R.cpp)
target_link_libraries(view_R ${catkin_LIBRARIES})

add_executable(view_L src/view_L.cpp)
target_link_libraries(view_L ${catkin_LIBRARIES})

add_executable(distance src/distance.cpp)
target_link_libraries(distance ${catkin_LIBRARIES})

add_executable(test1 src/test1.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES})

add_executable(test_del src/test_del.cpp)
target_link_libraries(test_del ${catkin_LIBRARIES})

add_executable(clustering src/clustering.cpp)
target_link_libraries(clustering ${catkin_LIBRARIES})

add_executable(kdtree src/kdtree.cpp)
target_link_libraries(kdtree ${catkin_LIBRARIES})

add_executable(test_vel src/test_vel.cpp)
target_link_libraries(test_vel ${catkin_LIBRARIES})

add_executable(test0812 src/test0812.cpp)
target_link_libraries(test0812 ${catkin_LIBRARIES})

add_executable(test0812_str src/test0812_str.cpp)
target_link_libraries(test0812_str ${catkin_LIBRARIES})

add_executable(del src/del.cpp)
target_link_libraries(del ${catkin_LIBRARIES})

add_executable(pure_pursuit_test_1 src/pure_pursuit_test_1.cpp)
target_link_libraries(pure_pursuit_test_1 ${catkin_LIBRARIES})

add_executable(pure_pursuit_test_2 src/pure_pursuit_test_2.cpp)
target_link_libraries(pure_pursuit_test_2 ${catkin_LIBRARIES})

add_executable(test_del_pid2 src/test_del_pid2.cpp)
target_link_libraries(test_del_pid2 ${catkin_LIBRARIES})

add_executable(test_loop src/test_loop.cpp)
target_link_libraries(test_loop ${catkin_LIBRARIES})

add_executable(test_vel_loop_1 src/test_vel_loop_1.cpp)
target_link_libraries(test_vel_loop_1 ${catkin_LIBRARIES})

add_executable(test_vel_loop_2 src/test_vel_loop_2.cpp)
target_link_libraries(test_vel_loop_2 ${catkin_LIBRARIES})

add_executable(pure_pursuit0812 src/pure_pursuit0812.cpp)
target_link_libraries(pure_pursuit0812 ${catkin_LIBRARIES})


