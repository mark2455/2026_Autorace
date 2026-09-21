from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    use_morai_lane = LaunchConfiguration('use_morai_lane')
    use_vesc_bridge = LaunchConfiguration('use_vesc_bridge')

    asw_params = PathJoinSubstitution([FindPackageShare('asw_lidar'), 'config', 'ros2_params.yaml'])
    vesc_params = PathJoinSubstitution([FindPackageShare('vesc_ackermann'), 'config', 'vesc_conversion.yaml'])
    waypoint_data = PathJoinSubstitution([FindPackageShare('waypoint_maker'), 'data'])

    common_camera_remap = [('/usb_cam/image_rect_color', camera_topic)]

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_rect_color'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('use_morai_lane', default_value='false'),
        DeclareLaunchArgument('use_vesc_bridge', default_value='false'),

        # Camera-derived event detectors
        Node(package='gray_detector', executable='gray_detector_node', name='gray_line_detector',
             output='screen', parameters=[{'gray_pixel_threshold': 40000}], remappings=common_camera_remap),
        Node(package='red_detector', executable='red_detector_node', name='red_detector',
             output='screen', remappings=common_camera_remap),
        Node(package='white_detector', executable='white_detector_node', name='white_line_detector',
             output='screen', parameters=[{'white_pixel_threshold': 20000}], remappings=common_camera_remap),

        # Main lane detector; private camera_ackermann topic becomes
        # /lane_detector_jeju_2/camera_ackermann, matching waypoint_follower.
        Node(
            package='lane_detector_jeju_2', executable='lane_detector_jeju_2', name='lane_detector_jeju_2',
            output='screen', remappings=common_camera_remap,
            parameters=[{
                'resize_width': 1280, 'resize_height': 720, 'gray_bin_thres': 190,
                'detect_line_count': 1, 'detect_y_offset_1': 60,
                'left_detect_offset': 0, 'right_detect_offset': 0,
                'steer_max_angle': 15, 'throttle': 0.5,
                'yaw_factor': 45, 'lateral_factor': 75,
            }]
        ),

        # Optional left/right sliding-window nodes used by some mission states.
        Node(package='morai_lane', executable='lane_right', name='lane_right', output='screen',
             condition=IfCondition(use_morai_lane), remappings=common_camera_remap),
        Node(package='morai_lane', executable='lane_left', name='lane_left', output='screen',
             condition=IfCondition(use_morai_lane), remappings=common_camera_remap),

        # Waypoint source.  0.csv from the original workspace is installed with the package.
        Node(
            package='waypoint_maker', executable='waypoint_loader', name='waypoint_loader_node', output='screen',
            remappings=[('/odometry/filtered', odom_topic)],
            parameters=[{'state_inspection': 0, 'parking_count': -3,
                         'csv_path': ParameterValue(waypoint_data, value_type=str)}]
        ),

        # LiDAR mission logic.  It receives mission_state from the follower and /scan from the sensor.
        Node(
            package='asw_lidar', executable='asw_lidar_node', name='asw_lidar_node', output='screen',
            parameters=[asw_params], remappings=[('/scan', scan_topic)]
        ),

        # Mission state + path/steering controller.
        Node(
            package='waypoint_follower', executable='waypoint_follower_node', name='waypoint_follower_node',
            output='screen',
            parameters=[{'is_dynamic_finished': False, 'is_parking_dist': 3.0}],
            remappings=[('/odometry/filtered', odom_topic), ('/imu', imu_topic)]
        ),

        # Optional original vehicle interface.  Leave false on LIMO/other platforms and adapt follower_cmd instead.
        Node(
            package='vesc_ackermann', executable='ackermann_to_vesc_node', name='ackermann_to_vesc_node',
            output='screen', parameters=[vesc_params], condition=IfCondition(use_vesc_bridge)
        ),
    ])
