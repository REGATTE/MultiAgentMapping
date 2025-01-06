# Ring Publisher ROS 2 Package

## Overview
This ROS 2 package adds a `ring` field to point cloud data published by a LiDAR sensor. It is designed to work with multi-beam LiDARs such as the Ouster OS1-128, which has 128 beams and 2048 points per scan.

The package allows specifying a robot namespace through parameters, enabling the use of multiple robots in a simulation or real-world setup.

---

## Dependencies
- ROS 2 (Humble or later)
- rclcpp
- sensor_msgs
- std_msgs
- launch
- launch_ros

---

## Topics
| Topic Name                                      | Type                                         | Role       |
|-------------------------------------------------|---------------------------------------------|------------|
| `/robot_x/scan3D`                               | `sensor_msgs/msg/PointCloud2`               | Subscriber |
| `/robot_x/scan3D_with_rings`                    | `sensor_msgs/msg/PointCloud2`               | Publisher  |

Note: Replace `robot_x` with the appropriate namespace for the robot.

---

## Parameters
| Parameter Name      | Default Value | Description                                                     |
|---------------------|---------------|-----------------------------------------------------------------|
| `robot_namespace`   | `robot_x`     | Namespace of the robot used in the topic names.                 |
| `input_topic`       | `scan3D`      | Input topic for subscribing to the raw point cloud data.        |

---

## Supported LiDAR Configuration
This package is tested with the **Ouster OS1-128** LiDAR:
- **Beams**: 128
- **Points per Scan**: 2048
- **Ring Field**: Calculated from beam index (0–127) and repeated cyclically.

---

## Building the Package
```bash
colcon build --packages-select pcl_ring_publisher
```

---

## Running the Node
To launch the node with a specific robot namespace:
```bash
ros2 launch pcl_ring_publisher ring_publisher.launch.py robot_namespace:=scout_x
```

---

## Example Visualization
To visualize the output point cloud with the `ring` field:
```bash
rviz2
```
- Add a **PointCloud2** display.
- Set the topic to `/scout_x/scan3D_with_rings`.
- Use **Color Transformer** -> **AxisColor** to view different ring IDs.

---

## License
This package is released under the Apache 2.0 License.

---
