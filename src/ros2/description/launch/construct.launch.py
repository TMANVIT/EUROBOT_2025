from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("description"), "urdf", "robot.xacro"]
    )

    camera_launch_path = PathJoinSubstitution(
        [FindPackageShare('camera'), 'launch', 'camera_launch.launch.py']
    )
    
    camera_localization_launch_path = PathJoinSubstitution(
        [FindPackageShare('camera_localization'), 'launch', 'camera_localization.launch.py']
    )

    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),
        
        DeclareLaunchArgument(
            name='publish_joints', 
            default_value='true',
            description='Launch joint_states_publisher'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints")),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ]
        ),
        
        Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"],
            name = "map_to_odom_static"
        ),
        
        # Node(
        #     package="tf2_ros",
        #     executable = "static_transform_publisher",
        #     arguments = ["0", "0", "0", "0", "0", "1", "base_link", "odom"],
        #     name = "base_link_to_odom_static"
        # ),
        
        Node(
            package="tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
            name = "base_footprint_to_odom_static"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_localization_launch_path)
        ),  
    ])