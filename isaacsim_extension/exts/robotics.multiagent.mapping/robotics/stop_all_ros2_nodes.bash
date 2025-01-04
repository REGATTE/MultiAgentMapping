#!/bin/bash

# List all active ROS 2 nodes
ros2 node list

# Stop all ROS 2 nodes by killing related processes
pkill -f ros2
killall -9 python3
killall -9 ros2

# Stop ROS 2 daemon
ros2 daemon stop

# Start ROS2 daemon
ros2 daemon start

# Verify if any nodes are still running
ros2 node list

echo "All ROS 2 nodes have been terminated."
