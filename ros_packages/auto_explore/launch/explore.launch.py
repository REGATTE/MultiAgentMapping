from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the 'namespace' argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='scout_x_x',
        description='Namespace for the robot'
    )

    # Use the namespace argument
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        namespace_arg,
        Node(
            package='auto_explore',
            executable='explore',
            name='explore',
            parameters=[{'namespace': namespace}],
            output='screen',
        ),
    ])
