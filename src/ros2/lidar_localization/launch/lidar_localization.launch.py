from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    obstacle_detector_params = PathJoinSubstitution(
        [FindPackageShare('obstacle_detector'), 'config', 'obstacle_config.yaml']
    )
    
    lidar_localization_params = PathJoinSubstitution(
        [FindPackageShare('lidar_localization'), 'config', 'lidar_localization.yaml']
    )
    

    return LaunchDescription([
               
        Node(
            package='lidar_localization',
            executable='lidar_localization_node',
            parameters=[lidar_localization_params]
        ),
        
        Node(
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            parameters=[obstacle_detector_params]
        ),
        
        Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "lidar", "robot_predict"],
            name = "obstacle_detector_to_lidar"
        )
        
    ])