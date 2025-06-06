from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pub_node = Node(
        package='camera_localization',
        executable='camera_localization_node',
    )
    camera_odom_node = Node(
        package='camera_localization',
        executable='camera_odom_node',
    )
  
    ld = LaunchDescription()

    # ld.add_action(camera_odom_node)
    ld.add_action(pub_node)
    return ld
