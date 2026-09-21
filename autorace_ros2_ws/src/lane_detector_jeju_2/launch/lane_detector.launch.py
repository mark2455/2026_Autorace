from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'resize_width': 1280,
        'resize_height': 720,
        'gray_bin_thres': 190,
        'detect_line_count': 1,
        'detect_y_offset_1': 60,
        'left_detect_offset': 0,
        'right_detect_offset': 0,
        'steer_max_angle': 15,
        'throttle': 0.5,
        'yaw_factor': 45,
        'lateral_factor': 75,
    }
    return LaunchDescription([
        Node(
            package='lane_detector_jeju_2',
            executable='lane_detector_jeju_2',
            name='lane_detector_jeju_2',
            output='screen',
            parameters=[params],
        )
    ])
