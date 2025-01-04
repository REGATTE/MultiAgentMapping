# tf_static_publisher

## Overview
The **tf_static_publisher** package publishes **static transforms** between the `world` frame and each robot's `odom` frame. The robot positions and orientations are loaded automatically from a JSON file stored at:
```
/home/regastation/workspaces/masters_ws/src/Multi-Agent_Mapping/initial_positions.json
```

This package integrates seamlessly with `tf_relay` and `tf_republisher` to manage transforms in multi-robot systems.

---

## Dependencies
- ROS 2 (Galactic or newer)
- `rclcpp`
- `tf2_ros`
- `geometry_msgs`
- `nlohmann_json`

---

## Topics

### **Input Topics**
This node does **not** subscribe to any input topics. Instead, it reads robot position and rotation data from the JSON file.

### **Output Topics**
1. **`/tf_static`** (type: `tf2_msgs/msg/TFMessage`)
   - Publishes static transforms for each robot in the format:
     ```
     world -> <robot_namespace>/odom
     ```
   - Example transform for `scout_1`:
     ```
     world -> scout_1/odom
     ```

---

## JSON File Format
The JSON file should define robot namespaces, positions, and rotations as shown below:
```json
{
    "robot_1": {
        "namespace": "scout_1",
        "position": {
            "x": 3.373275817938868,
            "y": -3.0545295297177084,
            "z": -6.869831537606653e-05
        },
        "rotation": {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 1
        }
    }
}
```

---

## Running the Package
1. **Build the Package:**
   ```bash
   colcon build --packages-select tf_static_publisher
   ```

2. **Source the Environment:**
   ```bash
   source install/setup.bash
   ```

3. **Run the Node:**
   ```bash
   ros2 run tf_static_publisher static_tf_publisher
   ```

---

## Verification
1. **TF Tree Visualization:**
   ```bash
   ros2 run tf2_tools view_frames
   ```
2. **Inspect Specific Transform:**
   ```bash
   ros2 run tf2_ros tf2_echo world scout_1/odom
   ```

---

## License
This package is distributed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0).

---

## Contact
For issues or contributions, contact **regastation**.

