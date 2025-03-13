from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Bag file configuration
    bag_file = PathJoinSubstitution([
        '/home/regastation/Desktop/Datasets/S3E_V2/S3E_campus_Road_2',
        'S3E_Campus_Road_2.db3'
    ])

    # Define remappings
    remappings = [
        ('/Alpha/velodyne_points', '/a/points'),
        ('/Bob/velodyne_points', '/b/points'),
        ('/Carol/velodyne_points', '/c/points'),
        ('/Alpha/imu/data', '/a/imu/data'),
        ('/Bob/imu/data', '/b/imu/data'),
        ('/Carol/imu/data', '/c/imu/data'),
        ('/Alpha/fix', '/a/fix'),
        ('/Bob/fix', '/b/fix'),
        ('/Carol/fix', '/c/fix')
    ]

    # Create the bag play process with remappings
    bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, '--remap'] + 
            [f"{old}:={new}" for old, new in remappings],
        output='screen'
    )

    return LaunchDescription([
        bag_process
    ])