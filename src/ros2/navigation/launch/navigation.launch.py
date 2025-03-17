import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

MAP_NAME = 'battlefield'  # Измените на имя вашей собственной карты

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
            name='sim',
            default_value='false',
            description='Включить use_sim_time в true'
        ),

        DeclareLaunchArgument(
            name='map',
            default_value=default_map_path,
            description='Путь к карте навигации'
        ),

        # Включение запуска Nav2 с настраиваемыми параметрами
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path,
                # Опционально, можно указать, какие узлы автоматически запускать; исключите 'amcl'
                'autostart': 'true',
            }.items()
        ),

        # Опционально, явно запустить lifecycle_manager_localization без amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('sim'),
                'autostart': True,
                'node_names': ['map_server'],  # Управлять только map_server, исключить amcl
            }]
        ),
    ])