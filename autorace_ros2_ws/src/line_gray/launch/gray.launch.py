from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gray_detector', executable='gray_detector_node', name='gray_detector', output='screen', parameters=[{'gray_pixel_threshold': 40000}])
    ])
