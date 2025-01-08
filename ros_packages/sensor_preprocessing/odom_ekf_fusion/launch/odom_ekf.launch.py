from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch argument for robot namespace
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='scout_1_1',
        description='Namespace of the robot to select EKF config file'
    )

    # Get the robot namespace
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Base config file path (static)
    package_name = 'project_config'
    base_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'Odom_ekf'
    )

    # Define the EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=robot_namespace,
        name=[robot_namespace, '_ekf_filter_node'],  # Prefix the name with namespace
        output='screen',
        parameters=[{
            'config_file': [base_config_path, '/', robot_namespace, '.yaml']  # Use substitutions properly
        }],
        remappings=[('/odometry/filtered', '/odom/processed')],
    )

    return LaunchDescription([
        robot_namespace_arg,
        ekf_node
    ])
