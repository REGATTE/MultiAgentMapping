from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_x',
        description='Namespace of the robot'
    )

    n_scan_arg = DeclareLaunchArgument(
        'N_SCAN',
        default_value='128',
        description='Number of LiDAR beams (e.g., 16, 32, 64, 128)'
    )

    horizon_scan_arg = DeclareLaunchArgument(
        'Horizon_SCAN',
        default_value='2048',
        description='Horizontal resolution (number of points per scan)' 
    )

    # Node configuration
    lidar_ring_converter_node = Node(
        package='pcl_ring_publisher',
        executable='ring_publisher',
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'N_SCAN': LaunchConfiguration('N_SCAN'),
            'Horizon_SCAN': LaunchConfiguration('Horizon_SCAN'),
            'input_topic': 'scan3D'
        }]
    )

    return LaunchDescription([
        robot_namespace_arg,
        n_scan_arg,
        horizon_scan_arg,
        lidar_ring_converter_node
    ])
