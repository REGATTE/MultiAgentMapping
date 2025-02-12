import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Shared package directory
    share_dir = get_package_share_directory('multi_agent_mapping')

    # Declare launch arguments
    params_1 = LaunchConfiguration('params_1')
    params_2 = LaunchConfiguration('params_2')

    declare_params_1 = DeclareLaunchArgument(
        'params_1',
        default_value='params_scout_1_1.yaml',
        description='Parameter file for scout_1_1'
    )

    declare_params_2 = DeclareLaunchArgument(
        'params_2',
        default_value='params_scout_2_2.yaml',
        description='Parameter file for scout_2_2'
    )

    # Launch Loop Visualization Node
    loop_visualization_node = Node(
        package='multi_agent_mapping',
        executable='multi_agent_mapping_loopVisualizationNode',
        name='distmapping_loopVisualizationNode',
        output='screen',
        parameters=[{'number_of_robots': 3}]
    )

    # Launch Scout 1
    scout_1 = GroupAction([
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ get_package_share_directory('isaac_sim_pointcloud_full_publisher'), 'launch', 'full_pcd_pub.launch.py'])),
        launch_arguments={'robot_namespace': 'scout_1_1', 'config_file': 'velodyne_vls_128.yaml'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share_dir, 'launch', 'single_robot.launch.py'])),
            launch_arguments={'namespace': '/scout_1_1', 'params': params_1, 'rviz_config': 'scout_1_1.rviz'}.items(),
        ),
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_a_to_b',
            output='screen',
            arguments=[PathJoinSubstitution([share_dir, 'config', 'DomainBridge', 'a_to_b.yaml'])],
            additional_env={'ROS_DOMAIN_ID': '1'}
        )
    ])

    # Launch Scout 2
    scout_2 = GroupAction([
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ get_package_share_directory('isaac_sim_pointcloud_full_publisher'), 'launch', 'full_pcd_pub.launch.py'])),
        launch_arguments={'robot_namespace': 'scout_2_2', 'config_file': 'velodyne_vls_128.yaml'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share_dir, 'launch', 'single_robot.launch.py'])),
            launch_arguments={'namespace': '/scout_2_2', 'params': params_2, 'rviz_config': 'scout_2_2.rviz'}.items(),
        ),
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge_b_to_a',
            output='screen',
            arguments=[PathJoinSubstitution([share_dir, 'config', 'DomainBridge', 'b_to_a.yaml'])],
            additional_env={'ROS_DOMAIN_ID': '2'}
        )
    ])

    return LaunchDescription([
        loop_visualization_node,
        declare_params_1,
        declare_params_2,
        scout_1,
        scout_2
    ])