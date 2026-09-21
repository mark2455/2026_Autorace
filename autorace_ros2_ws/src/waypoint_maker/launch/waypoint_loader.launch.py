from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_csv_path = PathJoinSubstitution([FindPackageShare('waypoint_maker'), 'data'])
    return LaunchDescription([
        DeclareLaunchArgument('state_inspection', default_value='0'),
        DeclareLaunchArgument('parking_count', default_value='-3'),
        DeclareLaunchArgument('csv_path', default_value=default_csv_path),
        Node(
            package='waypoint_maker', executable='waypoint_loader', name='waypoint_loader_node', output='screen',
            parameters=[{
                'state_inspection': ParameterValue(LaunchConfiguration('state_inspection'), value_type=int),
                'parking_count': ParameterValue(LaunchConfiguration('parking_count'), value_type=int),
                'csv_path': ParameterValue(LaunchConfiguration('csv_path'), value_type=str),
            }]
        ),
    ])
