from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('navigation'), 'launch', 'navigation.launch.py']
    )
    strategy_params = PathJoinSubstitution(
        [FindPackageShare("strategy"), "config", "yellow_plan.yaml"]
    )

    return LaunchDescription([
        Node(
            package='strategy',
            executable='strategy_node',
            name='strategy_node',
            output='screen',
            parameters=[strategy_params],
            
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch_path)),
    ])