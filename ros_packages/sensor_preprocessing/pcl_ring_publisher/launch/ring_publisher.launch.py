from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch argument for robot namespace
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_x',
        description='Namespace of the robot'
    )

    # Node configuration
    ring_publisher_node = Node(
        package='pcl_ring_publisher',
        executable='ring_publisher',
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'input_topic': 'scan3D'
        }]
    )

    return LaunchDescription([
        robot_namespace_arg,
        ring_publisher_node
    ])
