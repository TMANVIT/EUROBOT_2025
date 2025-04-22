from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_s3_launch.py']
    )   
    servo_params = PathJoinSubstitution(
        [FindPackageShare("servo_control"), "config", "servo_control.yaml"]
    )
        
    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='Pico 1', 
            default_value='/dev/ttyACM0',
            description='Pico 1'
        ),
        
        DeclareLaunchArgument(
            name='Pico 2', 
            default_value='/dev/ttyACM1',
            description='Pico 2'
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
        
        DeclareLaunchArgument(
            name='angle_compensate',
            default_value='false',
            description='Angle compensate parameter'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("Pico 1"), '--baudrate', LaunchConfiguration("micro_ros_baudrate")]
        ),
        
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("Pico 2"), '--baudrate', LaunchConfiguration("micro_ros_baudrate")]
        ),
        
        Node(
                package="imu",
                executable="imu_node",
                name="imu_node",
                output="screen",
                parameters=[{'port_name': '/dev/ttyUSB0'}]
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
            launch_arguments={
                'frame_id': 'laser',
                # 'serial_baudrate': LaunchConfiguration("lidar_baudrate"),
                'angle_compensate': LaunchConfiguration("angle_compensate")
            }.items(),
        ),
    ])