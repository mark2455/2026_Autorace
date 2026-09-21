from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_follower',
            executable='waypoint_follower_node',
            name='waypoint_follower_node',
            output='screen',
            parameters=[{
                'is_dynamic_finished': False,
                'is_parking_dist': 3.0,
            }],
        )
    ])
