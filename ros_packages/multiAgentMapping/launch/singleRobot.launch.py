from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    # Declare package paths
    pkg_share = FindPackageShare('multi_agent_mapping')

    # Declare all launch arguments
    robot_prefix = LaunchConfiguration('robotPrefix')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments with default values
    launch_args = [
        DeclareLaunchArgument(
            'robotPrefix',
            default_value='a',
            description='Robot namespace prefix'
        ),
        DeclareLaunchArgument(
            'params_file',
            description='Full path to the ROS2 parameters file to use'
        )
    ]

    # LIO-SAM Configuration
    lio_sam_group = GroupAction(
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                namespace=robot_prefix,
                arguments=[
                    '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                    'map', 'odom'
                ],
                parameters=[params_file],
                output='screen'
            ),
            # Robot State Publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_prefix,
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': Command(['xacro', ' ', PathJoinSubstitution([pkg_share, 'config', 'robot.urdf.xacro'])]),
                    'use_sim_time': True
                }],
                output='screen'
            ),
            # Load parameters
            Node(
                package='multi_agent_mapping',
                executable='multi_agent_mapping_imuPreintegration',
                namespace=robot_prefix,
                name='multi_agent_mapping_imuPreintegration',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='multi_agent_mapping',
                executable='multi_agent_mapping_imageProjection',
                namespace=robot_prefix,
                name='multi_agent_mapping_imageProjection',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='multi_agent_mapping',
                executable='multi_agent_mapping_featureExtraction',
                namespace=robot_prefix,
                name='multi_agent_mapping_featureExtraction',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='multi_agent_mapping',
                executable='multi_agent_mapping_mapOptimization',
                namespace=robot_prefix,
                name='multi_agent_mapping_mapOptimization',
                parameters=[params_file],
                output='screen'
            )
        ]
    )

    return LaunchDescription(launch_args + [
        lio_sam_group
    ])