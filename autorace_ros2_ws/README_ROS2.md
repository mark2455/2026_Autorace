# AutoRace ROS1 -> ROS2 Humble migration

This workspace is a **ROS2 Humble migration of the active 2024 AutoRace code path** found in the supplied ROS1/catkin archive.
The original archive is not overwritten.

## What was migrated

Core code that was most recently modified around the competition backup was migrated into `src/`:

- `coss_msgs`: custom mission message (`Coss.msg`) -> ROS2 `rosidl`
- `gray_detector`, `red_detector`, `white_detector`: camera event/line detectors
- `lane_detector_jeju_2`: main scale-platform lane detector
- `morai_lane`: left/right sliding-window camera nodes -> native `rclpy`
- `waypoint_maker`: waypoint messages + loader/saver; original `src/data/0.csv` is now installed with the package
- `waypoint_follower`: mission state machine / path and steering controller
- `asw_lidar`: LiDAR mission logic; ROS1 LaserScan->PointCloud conversion changed to ROS2 `PointCloud2`
- `vesc_ackermann`: the customized `follower_cmd`/Ackermann -> VESC motor/servo command converter
- `autorace_bringup`: a ROS2 launch file that starts the migrated core graph
- `autorace_ros2_compat`: a small **source-compatibility layer backed by `rclcpp`**. It allows the large legacy C++ logic to keep `ros::NodeHandle`, `ros::Publisher`, etc. syntax while running on ROS2 middleware. There is no ROS1 master/runtime dependency.

## Why a compatibility layer is used

A direct line-by-line rewrite of every old C++ file to idiomatic `rclcpp::Node` would make it much easier to accidentally change competition logic. The first migration therefore changes the build system, messages, launch system, parameters, callback pointer types and ROS runtime while keeping algorithm bodies as intact as possible. Once the port is validated on the vehicle, the compatibility layer can be removed package-by-package.

## Not bundled as converted source

The following are hardware/vendor or old development packages and should normally be replaced by their ROS2 versions instead of maintaining the ROS1 copies:

- `usb_cam` -> ROS2 `usb_cam`
- `image_pipeline` -> ROS2 `image_proc` / image pipeline packages
- `rplidar_ros` -> ROS2 RPLIDAR driver matching the actual sensor
- `razor_imu_9dof` -> ROS2 IMU driver matching the actual IMU
- full `vesc_driver` / `vesc_to_odom` -> only needed on the old VESC vehicle; the customized Ackermann command converter is included
- `fiducials`, `racecar`, `wego` -> not part of the identified active core path

The old `final` directory is retained under `legacy_ros1_reference/final` for comparison only. It is **not a ROS2 package**. Its old `union.launch` references `c_cam.py`, `c_lidar.py`, and `c_velocity.py`, which were not present in the supplied archive; this is why it was not treated as the final 2024 runtime graph.

## Ubuntu 22.04 / ROS2 Humble build

```bash
# Put/copy this directory somewhere under your home directory first, e.g.
cd ~/autorace_ros2_ws
source /opt/ros/humble/setup.bash

# Install dependencies available through rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

The conversion environment used to prepare this archive did not contain a ROS2 Humble installation, so a real `colcon build` could not be executed here. Python/XML/static checks were run, but **the first build on Ubuntu 22.04 may expose small API/dependency fixes**. See `MIGRATION_STATUS.md` before putting the vehicle on the ground.

## Start the migrated core graph

First start your ROS2 camera, LiDAR and odometry/IMU drivers. The old code expects the following logical inputs by default:

- camera: `/usb_cam/image_rect_color` (`sensor_msgs/msg/Image`)
- LiDAR: `/scan` (`sensor_msgs/msg/LaserScan`)
- odometry: `/odometry/filtered` (`nav_msgs/msg/Odometry`)
- IMU: `/imu` (`sensor_msgs/msg/Imu`)

Then:

```bash
source ~/autorace_ros2_ws/install/setup.bash
ros2 launch autorace_bringup autorace_core.launch.py
```

Topic names can be changed without editing source:

```bash
ros2 launch autorace_bringup autorace_core.launch.py \
  camera_topic:=/camera/image_raw \
  scan_topic:=/scan \
  odom_topic:=/odom \
  imu_topic:=/imu
```

The optional old-camera left/right nodes are disabled by default because they create OpenCV tuning windows:

```bash
ros2 launch autorace_bringup autorace_core.launch.py use_morai_lane:=true
```

The old VESC command converter is also disabled by default. Enable it only on the old VESC vehicle after checking calibration values:

```bash
ros2 launch autorace_bringup autorace_core.launch.py use_vesc_bridge:=true
```

## Inspect the graph

```bash
rqt_graph
```

or from CLI:

```bash
ros2 node list
ros2 topic list
ros2 topic info /follower_cmd -v
```

See `NODE_GRAPH.md` for the recovered architecture.
