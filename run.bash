#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

## ==============================================================================
## Robot scout_1_1
# Launch the Isaac Sim point cloud publisher in a new terminal
echo "Launching Isaac Sim Point Cloud Full Publisher for Scout_1_1 in a new terminal..."
gnome-terminal -- bash -c "
    export ROS_DOMAIN_ID=1; 
    ros2 launch isaac_sim_pointcloud_full_publisher full_pcd_pub.launch.py robot_namespace:=scout_1_1 config_file:=velodyne_vls_128.yaml; 
    exec bash
"

# Launch the LIO-SAM node in another new terminal
echo "Launching LIO-SAM with the specified parameters for Scout_1_1 in a new terminal..."
gnome-terminal -- bash -c "
    export ROS_DOMAIN_ID=1; 
    ros2 launch lio_sam run_params.launch.py params:=params_scout_1_1.yaml; 
    exec bash
"

## ==============================================================================
## Robot scout_2_2
# Launch the Isaac Sim point cloud publisher in a new terminal
echo "Launching Isaac Sim Point Cloud Full Publisher for Scout_2_2 in a new terminal..."
gnome-terminal -- bash -c "
    export ROS_DOMAIN_ID=2; 
    ros2 launch isaac_sim_pointcloud_full_publisher full_pcd_pub.launch.py robot_namespace:=scout_2_2 config_file:=velodyne_vls_128.yaml; 
    exec bash
"

# Launch the LIO-SAM node in another new terminal
echo "Launching LIO-SAM with the specified parameters for Scout_2_2 in a new terminal..."
gnome-terminal -- bash -c "
    export ROS_DOMAIN_ID=2; 
    ros2 launch lio_sam run_params.launch.py params:=params_scout_2_2.yaml; 
    exec bash
"

## ==============================================================================
## Robot scout_3_3
# Launch the Isaac Sim point cloud publisher in a new terminal
echo "Launching Isaac Sim Point Cloud Full Publisher for Scout_3_3 in a new terminal..."
gnome-terminal -- bash -c "
    export ROS_DOMAIN_ID=3; 
    ros2 launch isaac_sim_pointcloud_full_publisher full_pcd_pub.launch.py robot_namespace:=scout_3_3 config_file:=velodyne_vls_128.yaml; 
    exec bash
"

# Launch the LIO-SAM node in another new terminal
echo "Launching LIO-SAM with the specified parameters for Scout_3_3 in a new terminal..."
gnome-terminal -- bash -c "
    export ROS_DOMAIN_ID=3; 
    ros2 launch lio_sam run_params.launch.py params:=params_scout_3_3.yaml; 
    exec bash
"

## ==============================================================================
## Robot scout_4_4
# Launch the Isaac Sim point cloud publisher in a new terminal
echo "Launching Isaac Sim Point Cloud Full Publisher for Scout_4_4 in a new terminal..."
gnome-terminal -- bash -c "
    export ROS_DOMAIN_ID=4; 
    ros2 launch isaac_sim_pointcloud_full_publisher full_pcd_pub.launch.py robot_namespace:=scout_4_4 config_file:=velodyne_vls_128.yaml; 
    exec bash
"

# Launch the LIO-SAM node in another new terminal
echo "Launching LIO-SAM with the specified parameters for Scout_4_4 in a new terminal..."
gnome-terminal -- bash -c "
    export ROS_DOMAIN_ID=4; 
    ros2 launch lio_sam run_params.launch.py params:=params_scout_4_4.yaml; 
    exec bash
"