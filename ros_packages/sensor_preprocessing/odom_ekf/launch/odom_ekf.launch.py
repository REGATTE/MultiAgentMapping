import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Define file paths directly
    robot_config_file = '/home/regastation/workspaces/masters_ws/src/MultiAgentMapping/initial_positions.json'
    world_frame = 'world'

    ekf_nodes = []

    # Load robot namespaces from JSON file
    with open(robot_config_file, 'r') as file:
        robots_config = json.load(file)

    for robot_id, robot_data in robots_config.items():
        namespace = robot_data['namespace']
        odom_topic = 'odom'
        imu_topic = 'imu'
        base_frame = 'base_link'
        odom_frame = 'odom'
        filtered_odom_topic = 'odom/filtered'

        # Push namespace and create a robot-specific EKF node
        ekf_nodes.append(
            GroupAction(
                actions=[
                    PushRosNamespace(namespace),
                    Node(
                        package='robot_localization',
                        executable='ekf_node',
                        name='odom_ekf_filter_node',
                        output='screen',
                        parameters=[
                            {
                                'use_sim_time': True,
                                'frequency': 30.0,
                                'sensor_timeout': 0.1,
                                'two_d_mode': True,
                                'transform_time_offset': 0.0,
                                'transform_timeout': 0.0,
                                'print_diagnostics': True,
                                'debug': True,
                                'debug_out_file': '/home/debug/odom_ekf_debug.txt',
                                'publish_tf': True,
                                'publish_acceleration': False,
                                'reset_on_time_jump': True,
                                'odom_frame': odom_frame,
                                'base_link_frame': base_frame,
                                'world_frame': world_frame,
                                'odom0': odom_topic,
                                'odom0_config': [True, True, True,
                                                 False, False, False,
                                                 True, True, True,
                                                 False, False, True,
                                                 False, False, False],
                                'odom0_queue_size': 2,
                                'odom0_nodelay': False,
                                'odom0_differential': False,
                                'odom0_relative': True,
                                'odom0_pose_rejection_threshold': 5.0,
                                'odom0_twist_rejection_threshold': 1.0,
                                'imu0': imu_topic,
                                'imu0_config': [False, False, False,
                                                True, True, True,
                                                False, False, False,
                                                True, True, True,
                                                True, True, True],
                                'imu0_queue_size': 7,
                                'imu0_nodelay': False,
                                'imu0_differential': False,
                                'imu0_relative': True,
                                'imu0_pose_rejection_threshold': 0.8,
                                'imu0_twist_rejection_threshold': 0.8,
                                'imu0_linear_acceleration_rejection_threshold': 0.8,
                                'imu0_remove_gravitational_acceleration': True,
                                'use_control': False,
                                'stamped_control': False,
                                'control_timeout': 0.2,
                                'control_config': [True, False, False, False, False, True],
                                'acceleration_limits': [1.3, 0.0, 0.0, 0.0, 0.0, 3.4],
                                'deceleration_limits': [1.3, 0.0, 0.0, 0.0, 0.0, 4.5],
                                'acceleration_gains': [0.8, 0.0, 0.0, 0.0, 0.0, 0.9],
                                'deceleration_gains': [1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                                'process_noise_covariance': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                            0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                            0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                            0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                'initial_estimate_covariance': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                            }
                        ]
                    )
                ]
            )
        )

    return LaunchDescription(ekf_nodes)
