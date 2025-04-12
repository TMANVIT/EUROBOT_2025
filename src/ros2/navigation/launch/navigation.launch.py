from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

MAP_NAME = 'battlefield'

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    
    default_map_path = PathJoinSubstitution(
        [FindPackageShare('navigation'), 'map', f'{MAP_NAME}.yaml']
    )
    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('navigation'), 'config', 'navigation.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map',
            default_value=default_map_path,
            description='Path to map'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'params_file': nav2_config_path,
                'autostart': 'true',
            }.items()
        ),
        Node(
            package='map_creation',
            executable='map_creator',
            name='map_creator_node',
            output='screen',
        ),
        Node(
            package='waypoint_planner',
            executable='waypoint_planner',
            name='waypoint_planner_node',
            output='screen',
        ),
  
    ])