from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    
    micro_ros_launch_path = PathJoinSubstitution(
        [FindPackageShare('robot_bringup'), 'launch', 'micro_ros_agent.launch.py']
    )   
    
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("robot_base"), "config", "ekf.yaml"]
    )
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(micro_ros_launch_path),
        # ),
    ])