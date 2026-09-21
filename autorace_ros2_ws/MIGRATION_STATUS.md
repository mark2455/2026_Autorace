# Migration status / first-build checklist

## Status

This is a **source-complete first-pass migration of the identified active 2024 graph**, not a claim that the old competition stack has already been vehicle-validated under ROS2.

### Converted

- catkin metadata -> `ament_cmake` / `ament_python`
- ROS1 custom messages -> ROS2 `rosidl`
- ROS1 launch -> ROS2 Python launch for migrated nodes
- Python `rospy` camera lane nodes -> native `rclpy`
- legacy C++ ROS API -> ROS2-backed compatibility layer (`rclcpp` underneath)
- `LaserScan` projection in `asw_lidar` -> ROS2 `sensor_msgs/msg/PointCloud2`
- old hard-coded waypoint path `/home/wego/catkin_ws/src/data/` -> launch-configurable package data path
- original `src/data/0.csv` copied into `waypoint_maker/data/`
- one out-of-bounds initialization in `waypoint_follower` (`state[6]`) replaced by safe initialization without changing state-machine intent

### Preserved but requires verification

1. **LiDAR Task1 integration:** see `NODE_GRAPH.md`; Task1 topic behavior differs from Task3/Task5.
2. **ASW LiDAR parameters:** active Task1 reads `/track/*`, while the original YAML mainly contained `Task1/*`. Matching values were mirrored to `track.*`. `track.done_count_param=5` and `track.done_size=5` were not present in the supplied YAML and are explicit migration defaults; tune/confirm them.
3. **VESC calibration:** `4614`, `-1.2135`, `0.546` were copied from the supplied `racecar-v2/vesc.yaml`. The customized converter also multiplies speed by `0.1/0.2` and steering by `0.7`; verify on stands before enabling.
4. **GUI perception nodes:** several original nodes call `cv::imshow`/OpenCV windows. They need a graphical session unless those debug windows are removed.
5. **Waypoint CSV:** only `0.csv` was present in the supplied `src/data` folder.

## First build

```bash
cd ~/autorace_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
```

If the whole build stops, isolate packages in this order:

```bash
colcon build --symlink-install --packages-select autorace_ros2_compat coss_msgs
colcon build --symlink-install --packages-select gray_detector red_detector white_detector
colcon build --symlink-install --packages-select lane_detector_jeju_2 morai_lane
colcon build --symlink-install --packages-select waypoint_maker waypoint_follower
colcon build --symlink-install --packages-select asw_lidar
colcon build --symlink-install --packages-select vesc_ackermann autorace_bringup
```

## Before moving the vehicle

1. Run camera/LiDAR/odom drivers and verify types with `ros2 topic info -v`.
2. Launch core with VESC disabled.
3. Inspect `rqt_graph` and `ros2 topic echo /follower_cmd`.
4. Verify mission transitions using `mission_state` and `coss_state` while wheels are off the ground.
5. Only then connect the vehicle command adapter / VESC output.
