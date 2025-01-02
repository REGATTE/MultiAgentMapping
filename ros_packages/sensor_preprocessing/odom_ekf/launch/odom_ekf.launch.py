import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Define file paths directly
    robot_config_file = '/home/regastation/workspaces/masters_ws/src/MultiAgentMapping/initial_positions.json'
    ekf_config_file = '/home/regastation/workspaces/masters_ws/src/MultiAgentMapping/ros_packages/sensor_preprocessing/odom_ekf/config/ekf.yaml'
    world_frame = 'world'

    ekf_nodes = []

    # Load robot namespaces from JSON file
    with open(robot_config_file, 'r') as file:
        robots_config = json.load(file)

    for robot_id, robot_data in robots_config.items():
        namespace = robot_data['namespace']
        odom_topic = f'{namespace}/odom'
        imu_topic = f'{namespace}/imu'
        base_frame = f'{namespace}/base_link'
        odom_frame = f'{namespace}/odom'
        filtered_odom_topic = f'{namespace}/odom/filtered'

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
                            ekf_config_file,
                            {
                                'odom0': odom_topic,
                                'odom0_config': [True, True, True,
                                                 False, False, False,
                                                 True, True, True,
                                                 False, False, True,
                                                 False, False, False],
                                'imu0': imu_topic,
                                'imu0_config': [False, False, False,
                                                True, True, True,
                                                False, False, False,
                                                True, True, True,
                                                True, True, True],
                                'odom_frame': odom_frame,
                                'base_link_frame': base_frame,
                                'world_frame': world_frame
                            }
                        ],
                        remappings=[
                            ('/odom/raw', odom_topic),
                            ('/IMU/processed', imu_topic),
                            ('/odom/filtered', filtered_odom_topic),
                            ('base_link', base_frame),
                            ('odom', odom_frame)
                        ]
                    )
                ]
            )
        )

    return LaunchDescription(ekf_nodes)
