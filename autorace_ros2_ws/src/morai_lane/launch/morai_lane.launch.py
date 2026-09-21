from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='morai_lane', executable='lane_right', name='lane_right', output='screen'),
        Node(package='morai_lane', executable='lane_left', name='lane_left', output='screen'),
    ])
