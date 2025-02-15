from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os.path
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():

    pub_node = Node(
        package='camera_localization',
        executable='camera_localization_node',
    )
  
    ld = LaunchDescription()


    ld.add_action(pub_node)

    return ld
