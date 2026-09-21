from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('min_dist', default_value='0.05'),
        DeclareLaunchArgument('output_csv', default_value='/tmp/autorace_data/data.csv'),
        Node(
            package='waypoint_maker', executable='waypoint_saver', name='waypoint_saver', output='screen',
            parameters=[{'min_dist_': LaunchConfiguration('min_dist'), 'output_csv': LaunchConfiguration('output_csv')}]
        ),
    ])
