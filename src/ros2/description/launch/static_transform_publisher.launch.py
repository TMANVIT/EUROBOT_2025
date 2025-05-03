from launch import LaunchDescription
from launch_ros.actions import Node
import os

TEAM = os.getenv("TEAM")

def generate_launch_description():
    
    
    if TEAM == "0":
        torch1_node = Node(
                package="tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["1.59048", "0.09193703", "0", "0", "0", "0", "map", "torch_yellow_1"],
                name = "First_torch_yellow_team"
            )
        torch2_node = Node(
                package="tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["-1.57", "-0.959966", "0", "0", "0", "0", "map", "torch_yellow_2"],
                name = "Second_torch_yellow_team"
            )
        torch3_node = Node(
                package="tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["-1.57", "0.959966", "0", "0", "0", "0", "map", "torch_yellow_3"],
                name = "Third_torch_yellow_team"
            )

        init_pose = Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["-0.28", "-0.80", "0", "-1.57", "0", "0", "map", "odom"],
            name = "init_pose"
        )

    else:
        torch1_node = Node(
                package="tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["1.5122", "1.0041", "0", "0", "0", "0", "map", "torch_blue_1"],
                name = "First_torch_blue_team"
            )
        torch2_node = Node(
                package="tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["1.56546", "-0.880723", "0", "0", "0", "0", "map", "torch_blue_2"],
                name = "Second_torch_blue_team"
            )
        torch3_node = Node(
                package="tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["-1.6032", "-0.014313", "0", "0", "0", "0", "map", "torch_blue_3"],
                name = "Third_torch_blue_team"
            )

        init_pose = Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0.28", "-0.80", "0", "-1.57", "0", "0", "map", "odom"],
            name = "init_pose"
        )

    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "world", "map"],
                name = "world_to_map_static"
            ),       
            
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            #     name="initial_map_to_odom",
            # ),
            
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
                name="initial_odom_to_base_footprint",
            ),
            
            ## ===================================================================================
            ## LIDAR_LOCALIZATION
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "3.14", "0", "0", "lidar_link", "laser"],
                name="lidar_link_to_laser",
            ),
            torch1_node,
            torch2_node,
            torch3_node,
            init_pose
        ]
    )
