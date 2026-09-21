from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='red_detector', executable='red_detector_node', name='red_detector', output='screen', parameters=[])
    ])
