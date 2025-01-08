from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml


def generate_launch_description():
    # Declare launch argument for robot namespace
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='scout_1_1',
        description='Namespace of the robot to select EKF config file'
    )

    # Get the robot namespace
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Locate the config file
    package_name = 'project_config'
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'Odom_ekf',
        'scout_1_1.yaml'  # Hardcode this for now
    )

    # Debugging output for config file path
    print(f"Config file path: {config_file_path}")

    # Load the YAML file into a dictionary
    with open(config_file_path, 'r') as file:
        params = yaml.safe_load(file)

    # Extract the actual parameters
    parameters = params['ekf_filter_node']['ros__parameters']

    # Define the EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=robot_namespace,
        name='ekf_filter_node',  # Static name
        output='screen',
        parameters=[parameters],  # Pass parameters directly
    )

    # Return the launch description
    return LaunchDescription([
        robot_namespace_arg,
        ekf_node
    ])
