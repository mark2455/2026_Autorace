from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='white_detector', executable='white_detector_node', name='white_detector', output='screen', parameters=[{'white_pixel_threshold': 20000}])
    ])
