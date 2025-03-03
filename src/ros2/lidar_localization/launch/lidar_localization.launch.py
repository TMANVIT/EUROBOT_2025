from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
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
        
        DeclareLaunchArgument(
            name='active_param', 
            default_value="true",
            description='active_param'
        ),
               
        Node(
            package='lidar_localization',
            executable='lidar_localization_node',
            name='lidar_localization_node',
            output='screen',
            parameters=[lidar_localization_params]
        ),
        
        Node(
            package='lidar_localization',
            executable='prediction_pose_node',
            name='prediction_pose_node',
            output='screen',
        ),
        
        Node(
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            name='obstacle_extractor_node',
            output='screen',
            parameters=[obstacle_detector_params]
        ),
        
        Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "robot_predict", "base_footprint"],
            name = "obstacle_detector_to_lidar"
        ),
        
        Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["1.57914", "0.98733", "0", "0", "0", "0", "map", "torch_blue_1"],
            name = "First_torch_blue_team"
        ),
        
        Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["1.56546", "-0.880723", "0", "0", "0", "0", "map", "torch_blue_2"],
            name = "Second_torch_blue_team"
        ),
        
        Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["-1.55888", "0.00659359", "0", "0", "0", "0", "map", "torch_blue_3"],
            name = "Third_torch_blue_team"
        )
        
    ])