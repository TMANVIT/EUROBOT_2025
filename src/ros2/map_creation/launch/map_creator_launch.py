from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_creation',       # Обновлено на имя пакета
            executable='map_creator',
            name='map_creator_node',
            output='screen',
        ),
    ])