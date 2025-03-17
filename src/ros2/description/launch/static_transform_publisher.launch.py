from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "world", "map"],
                name = "world_to_map_static"
            ),
            # ## Link map to odom. TODO change this transform to camera initial pose.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "1.57", "0", "0", "map", "odom"],
                name="map_to_odom_static",
            ),         
            
            ## Link to torches, which depend on robot color. TODO Switch color in Makefile
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
            ),
            ## ===================================================================================
            ## LIDAR_LOCALIZATION
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "3.14", "0", "0", "lidar_link", "laser"],
                name="lidar_link_to_laser",
            ),
        ]
    )
