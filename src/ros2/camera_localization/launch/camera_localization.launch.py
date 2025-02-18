from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pub_node = Node(
        package='camera_localization',
        executable='camera_localization_node',
    )
    world_to_map_transform = Node(
        package="tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "world", "map"],
        name = "world_to_map_static"
    )
  
    ld = LaunchDescription()


    ld.add_action(pub_node)
    ld.add_action(world_to_map_transform)

    return ld
