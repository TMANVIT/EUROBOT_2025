from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

MAP_NAME = 'battlefield' 

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('navigation'), 'map', f'{MAP_NAME}.yaml']
    )

    map_creator_launch_path = PathJoinSubstitution(
        [FindPackageShare('map_creation'), 'launch', 'map_creator_launch.py']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('navigation'), 'config', 'navigation.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='Set use_sim_time'
        ),

        DeclareLaunchArgument(
            name='map',
            default_value=default_map_path,
            description='Path to map'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path,
                'autostart': 'true',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_creator_launch_path)
        ),
    ])