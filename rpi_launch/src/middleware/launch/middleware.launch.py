from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    
    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_s3_launch.py']
    )   
        
    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='Pico 1', 
            default_value='/dev/ttyACM0',
            description='Pico 1'
        ),
        
        DeclareLaunchArgument(
            name='micro_ros_baudrate', 
            default_value='115200',
            description='micro-ROS baudrate'
        ),
        
        DeclareLaunchArgument(
            name='lidar_baudrate',
            default_value='115200',
            description='Baudrate for the Lidar serial communication'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("Pico 1"), '--baudrate', LaunchConfiguration("micro_ros_baudrate")]
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
            launch_arguments={
                'frame_id': 'lidar',
                'serial_baudrate': LaunchConfiguration("lidar_baudrate")
            }.items(),
        ),
    ])