from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    obstacle_detector_params = PathJoinSubstitution(
        [FindPackageShare("obstacle_detector"), "config", "obstacle_config.yaml"]
    )

    team = os.getenv("TEAM")
    if team == "0":
        lidar_localization_params = PathJoinSubstitution(
            [FindPackageShare("lidar_localization"), "config", "lidar_localization_yellow.yaml"]
        )
    else:
        lidar_localization_params = PathJoinSubstitution(
            [FindPackageShare("lidar_localization"), "config", "lidar_localization_blue.yaml"]
        )

    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="active_param", default_value="true", description="active_param"
            ),
            # Node(
            #     package="lidar_localization",
            #     executable="lidar_localization_node",
            #     name="lidar_localization_node",
            #     output="screen",
            #     parameters=[lidar_localization_params],
            # ),
            # Node(
            #     package="lidar_localization",
            #     executable="prediction_pose_node",
            #     name="prediction_pose_node",
            #     output="screen",
            # ),
            # Node(
            #     package="lidar_localization",
            #     executable="lidar_obstacle_node",
            #     name="lidar_obstacle_node",
            #     output="screen",
            # ),
            Node(
                package="obstacle_detector",
                executable="obstacle_extractor_node",
                name="obstacle_extractor_node",
                output="screen",
                parameters=[obstacle_detector_params],
            ),
        ]
    )
