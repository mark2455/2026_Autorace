from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(get_package_share_directory('asw_lidar'), 'config', 'ros2_params.yaml')
    return LaunchDescription([
        Node(
            package='asw_lidar', executable='asw_lidar_node', name='asw_lidar_node',
            output='screen', parameters=[params]
        )
    ])
