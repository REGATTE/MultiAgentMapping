import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('multi_agent_mapping')
    robot_namespace_ = LaunchConfiguration('namespace')
    param_file_name = LaunchConfiguration('params')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params',
        default_value='params.yaml',
        description='Name of the ROS 2 parameter file to use. Example: params_scout_1_1.yaml'
    )

    robot_namespace_declare = DeclareLaunchArgument(
        'namespace',
        default_value='/scout_1_1'
    )

    print("URDF file path: {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        robot_namespace_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=robot_namespace_,
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[PathJoinSubstitution([share_dir, 'config', param_file_name])],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_namespace_,
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        Node(
            package='multi_agent_mapping',
            executable='multi_agent_mapping_imuPreintegration',
            name='multi_agent_mapping_imuPreintegration',
            namespace=robot_namespace_,
            parameters=[PathJoinSubstitution([share_dir, 'config', param_file_name])],
            output='screen'
        ),
        Node(
            package='multi_agent_mapping',
            executable='multi_agent_mapping_imageProjection',
            name='multi_agent_mapping_imageProjection',
            namespace=robot_namespace_,
            parameters=[PathJoinSubstitution([share_dir, 'config', param_file_name])],
            output='screen'
        ),
        Node(
            package='multi_agent_mapping',
            executable='multi_agent_mapping_featureExtraction',
            name='multi_agent_mapping_featureExtraction',
            namespace=robot_namespace_,
            parameters=[PathJoinSubstitution([share_dir, 'config', param_file_name])],
            output='screen'
        ),
        Node(
            package='multi_agent_mapping',
            executable='multi_agent_mapping_mapOptimization',
            name='multi_agent_mapping_mapOptimization',
            namespace=robot_namespace_,
            parameters=[PathJoinSubstitution([share_dir, 'config', param_file_name])],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_namespace_,
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
