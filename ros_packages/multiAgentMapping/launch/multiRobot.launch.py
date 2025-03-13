from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare('multi_agent_mapping')
    
    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'number_of_robots',
            default_value='3',
            description='Number of robots in the system'
        ),
         DeclareLaunchArgument(
            'bag',
            default_value='/home/regastation/Desktop/Datasets/S3E_V1/S3E_Campus_Road_1/S3E_Campus_Road_1.db3',
            description='Path to bag file'
        )
    ]

    # RViz node
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'rviz', 'globalViz_3.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dcl_rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Loop visualization node
    loop_viz_node = Node(
        package='multi_agent_mapping',
        executable='multi_agent_mapping_loopVisualizationNode',
        name='loop_visualization_node',
        output='screen'
    )

    # Include single robot launches
    robot_launches = []
    for robot_prefix in ['a', 'b', 'c']:
        params_file = PathJoinSubstitution([
            pkg_share,
            'config',
            f'params_{robot_prefix}.yaml'
        ])
        
        single_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share,
                    'launch',
                    'singleRobot.launch.py'
                ])
            ]),
            launch_arguments={
                'robotPrefix': robot_prefix,
                'params_file': params_file
            }.items()
        )
        robot_launches.append(single_robot_launch)
    
    # Define remappings for bag playback
    remappings = [
        ('/Alpha/velodyne_points', '/a/velodyne_points'),
        ('/Bob/velodyne_points', '/b/velodyne_points'),
        ('/Carol/velodyne_points', '/c/velodyne_points'),
        ('/Alpha/imu/data', '/a/imu/data'),
        ('/Bob/imu/data', '/b/imu/data'),
        ('/Carol/imu/data', '/c/imu/data')
    ]

    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '-s', 'sqlite3', '--remap'] + 
            [f"{old}:={new}" for old, new in remappings],
        output='screen'
    )

    return LaunchDescription(
        launch_args +  
        [rviz_node, loop_viz_node] + 
        robot_launches +
        [bag_player]
    )

