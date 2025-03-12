#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

## ==============================================================================
## Robot scout_1_1
# Launch the Isaac Sim point cloud publisher in a new terminal
echo "Launching Isaac Sim Point Cloud Full Publisher for Scout_1_1 in a new terminal..."
gnome-terminal -- bash -c "
    ros2 launch isaac_sim_pointcloud_full_publisher full_pcd_pub.launch.py robot_namespace:=a config_file:=velodyne_vls_128.yaml; 
    exec bash
"

# Launch the distributed mapping node in another new terminal
echo "Launching distributed mapping with the specified parameters for Scout_1_1 in a new terminal..."
gnome-terminal -- bash -c "
    ros2 launch multi_agent_mapping run_params.launch.py params:=params_scout_1_1.yaml  namespace:=/a rviz_config:=scout_1_1.rviz;
    exec bash
"

gnome-terminal -- bash -c "
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/a/cmd_vel;
    exec bash
"

## ==============================================================================
## Robot scout_2_2
# Launch the Isaac Sim point cloud publisher in a new terminal
echo "Launching Isaac Sim Point Cloud Full Publisher for Scout_2_2 in a new terminal..."
gnome-terminal -- bash -c "
    ros2 launch isaac_sim_pointcloud_full_publisher full_pcd_pub.launch.py robot_namespace:=b config_file:=velodyne_vls_128.yaml; 
    exec bash
"

# Launch the distributed mapping node in another new terminal
echo "Launching distributed mapping with the specified parameters for Scout_2_2 in a new terminal..."
gnome-terminal -- bash -c "
    ros2 launch multi_agent_mapping run_params.launch.py params:=params_scout_2_2.yaml  namespace:=/b rviz_config:=scout_2_2.rviz;  
    exec bash
"

gnome-terminal -- bash -c "
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/b/cmd_vel;
    exec bash
"

## ==============================================================================
## Distributed
echo "Launching RVIZ"
gnome-terminal -- bash -c "
    ros2 run rviz2 rviz2 -d /home/regastation/workspaces/masters_ws/src/MultiAgentMapping/ros_packages/multiAgentMapping/config/rviz/globalViz_2.rviz;
    exec bash
"

echo "LoopVisualizationNode"
gnome-terminal -- bash -c "
    ros2 run multi_agent_mapping multi_agent_mapping_loopVisualizationNode;
    exec bash
"

