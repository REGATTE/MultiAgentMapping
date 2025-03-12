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

    # Declare launch arguments with default values
    launch_args = [
        DeclareLaunchArgument(
            'robotPrefix',
            default_value='a',
            description='Robot namespace prefix'
        )
    ]

    # LIO-SAM Configuration
    lio_sam_group = GroupAction(
        actions=[
            # Load parameters
            Node(
                package='multi_agent_mapping',
                executable='multi_agent_mapping_imuPreintegration',
                namespace=robot_prefix,
                name='multi_agent_mapping_imuPreintegration',
                parameters=[PathJoinSubstitution([pkg_share, 'config', f'params_{robot_prefix}.yaml'])],
                output='screen'
            ),
            Node(
                package='multi_agent_mapping',
                executable='multi_agent_mapping_imageProjection',
                namespace=robot_prefix,
                name='multi_agent_mapping_imageProjection',
                parameters=[PathJoinSubstitution([pkg_share, 'config', f'params_{robot_prefix}.yaml'])],
                output='screen'
            ),
            Node(
                package='multi_agent_mapping',
                executable='multi_agent_mapping_featureExtraction',
                namespace=robot_prefix,
                name='multi_agent_mapping_featureExtraction',
                parameters=[PathJoinSubstitution([pkg_share, 'config', f'params_{robot_prefix}.yaml'])],
                output='screen'
            ),
            Node(
                package='multi_agent_mapping',
                executable='multi_agent_mapping_mapOptimization',
                namespace=robot_prefix,
                name='multi_agent_mapping_mapOptimization',
                parameters=[PathJoinSubstitution([pkg_share, 'config', f'params_{robot_prefix}.yaml'])],
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
                    'tf_prefix': robot_prefix
                }],
                output='screen'
            )
        ]
    )

    return LaunchDescription(launch_args + [
        lio_sam_group
    ])