from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml


def generate_launch_description():
    # Declare launch arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='scout_1_1',
        description='Namespace for the robot and topics'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='scout_1_1.yaml',
        description='Configuration YAML file for EKF node'
    )

    # Get launch configurations
    robot_namespace = LaunchConfiguration('robot_namespace')
    config_file = LaunchConfiguration('config_file')

    # Resolve the config file path at generation time
    package_name = 'project_config'
    config_directory = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'Odom_ekf'
    )
    full_config_path = os.path.join(config_directory, 'scout_1_1.yaml')  # Replace with static filename or parameter if dynamic

    # Debugging output for config path
    print(f"Config file path: {full_config_path}")

    # Load parameters from the YAML file
    with open(full_config_path, 'r') as file:
        params = yaml.safe_load(file)

    # Extract the actual parameters
    parameters = params['ekf_filter_node']['ros__parameters']

    # Define the EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=robot_namespace,  # Dynamic namespace
        name='ekf_filter_node',  # Static node name
        output='screen',
        parameters=[parameters],  # Pass YAML parameters directly
    )

    # Return the launch description
    return LaunchDescription([
        robot_namespace_arg,
        config_file_arg,
        ekf_node
    ])
