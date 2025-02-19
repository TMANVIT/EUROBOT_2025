from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os.path
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    package_path = get_package_share_directory('camera')

    default_config_path = os.path.join(
        package_path, 'config', 'camera_calibration_config.yaml')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz_cfg', 'eurobot.rviz')
    
    camera_config_path = os.path.join(
        package_path, 'config', 'camera_config.yaml')
    
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    bve_pose_node = Node(
        package='camera',
        executable='camera_bve_pose',
    )
    
    image_raw_pub_node = Node(
        package='camera',
        executable='image_raw_publisher',
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    # ld.add_action(bve_pose_node)
    ld.add_action(rviz_node)
    ld.add_action(image_raw_pub_node)

    return ld
