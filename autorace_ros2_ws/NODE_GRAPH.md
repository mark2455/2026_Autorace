# Recovered 2024 node/topic architecture

The graph below is derived from the publishers/subscribers in the supplied source, not from filename guessing.

```text
Camera Image (/usb_cam/image_rect_color)
   |\
   | +--> gray_line_detector  --> /gray_detected -----------+
   | +--> red_detector        --> /redzone_flag ------------+
   | +--> white_line_detector --> /stopline_detected --------+
   | +--> lane_detector_jeju_2 --> /lane_detector_jeju_2/    |
   |                              camera_ackermann            |
   |                                                        |
   +----> [optional morai_lane] --> /kuurack_left/right -----+
                                                            |
Odometry (/odometry/filtered) --> waypoint_maker             |
                                  |                           |
                                  +--> final_waypoints -------+
                                  +--> gps_state              |
                                                            v
                         +---------------------------+
IMU (/imu) ------------->|     waypoint_follower     |
LiDAR state (coss_state)->| mission/state/controller |
                         +-------------+-------------+
                                       |
                                       +--> mission_state ----> asw_lidar
                                       |
                                       +--> auto_flag --------> VESC converter (optional)
                                       |
                                       +--> follower_cmd -----> vehicle adapter / VESC converter

LiDAR (/scan) -------------------------------> asw_lidar
                                                |
                                                +--> coss_state
                                                +--> mission_lidar
                                                +--> lidar_ / paths / debug markers
```

## Important topic contracts

| Producer | Topic | Message | Consumer / purpose |
|---|---|---|---|
| camera driver | `/usb_cam/image_rect_color` | `sensor_msgs/msg/Image` | line/lane perception |
| gray detector | `/gray_detected` | `coss_msgs/msg/Coss` | waypoint follower |
| red detector | `/redzone_flag` | `coss_msgs/msg/Coss` | waypoint follower |
| white detector | `/stopline_detected` | `coss_msgs/msg/Coss` | waypoint follower |
| lane detector | `/lane_detector_jeju_2/camera_ackermann` | `ackermann_msgs/msg/AckermannDriveStamped` | waypoint follower |
| morai lane (optional) | `/kuurack_left`, `/kuurack_right` | `ackermann_msgs/msg/AckermannDriveStamped` | waypoint follower |
| waypoint loader | `final_waypoints` | `waypoint_maker/msg/Lane` | waypoint follower |
| odometry | `/odometry/filtered` | `nav_msgs/msg/Odometry` | waypoint loader + follower |
| waypoint follower | `mission_state` | `coss_msgs/msg/Coss` | asw_lidar |
| RPLIDAR/driver | `/scan` | `sensor_msgs/msg/LaserScan` | asw_lidar |
| asw_lidar | `coss_state` | `coss_msgs/msg/Coss` | waypoint follower |
| waypoint follower | `follower_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | vehicle interface |
| waypoint follower | `auto_flag` | `std_msgs/msg/Bool` | old VESC command converter |
| VESC converter (optional) | `commands/motor/speed` | `std_msgs/msg/Float64` | VESC driver |
| VESC converter (optional) | `commands/servo/position` | `std_msgs/msg/Float64` | VESC driver + follower feedback |

## Original-code inconsistency preserved for review

The active `Task1` implementation in `asw_lidar` publishes steering on `lidar_` and completion on `mission_lidar`, while `waypoint_follower` obtains `lidar_steer` through `coss_state`. `Task3` and `Task5` do publish `coss_state`. This appears to be an original integration inconsistency; the migration does **not silently redesign it**. Confirm which Task1 version was actually used before testing mission state 0.
