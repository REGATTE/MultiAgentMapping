# MultiAgentMapping

## Installation

### Clone the Repository
```bash
cd ros2_ws/src
git clone https://github.com/REGATTE/MultiAgentMapping.git
```

### Install LIO-SAM

1. Clone the LIO-SAM repository:
```bash
cd ros2_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2
```

2. Install required dependencies:
```bash
sudo apt install ros-<ros2-version>-perception-pcl \
   ros-<ros2-version>-pcl-msgs \
   ros-<ros2-version>-vision-opencv \
   ros-<ros2-version>-xacro
```
> Replace `<ros2-version>` with your ROS 2 distribution (e.g., `humble` or `foxy`).

3. Ensure Eigen version 3.4.0 is installed.

4. Install GTSAM:
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### Build the Workspace
```bash
cd ros2_ws
colcon build --symlink-install
```

---

## Extension Setup

### Install Isaac Sim Extension
1. Open Isaac Sim.
2. Go to `Window > Extensions`.
3. Click on the **three stripes menu** in the top-right corner and select **Settings**.
4. In the settings panel, click the **green (+) button** and add the absolute path to the `isaacsim_extension/exts` directory.
5. The extension should now be loaded.

---

## Running the Project

### Launch Isaac Sim and Load Robots
1. Start Isaac Sim using your preferred method.
2. Activate the extension.
3. Choose the environment and robot setup: `scout_1_1`, `scout_2_2`, `scout_3_3`, or `scout_4_4`.

### Run LIO-SAM
1. Set the ROS 2 domain ID for the desired robot:
```bash
export ROS_DOMAIN_ID=x
```
Replace `x` with the robot's ID number. For example, for `scout_1_1`, use:
```bash
export ROS_DOMAIN_ID=1
```

2. Launch LIO-SAM with the corresponding configuration file:
```bash
ros2 launch lio_sam run.launch.py params_file:=/path/to/Config/LIO_SAM/scout_1_1.yaml
```
> Replace the path with the actual path to your YAML configuration file.

---

## Notes
- Ensure all robots have distinct ROS domain IDs to avoid conflicts during multi-robot operation.
- Verify parameter file paths and configurations for each robot before launching.
- Isaac Sim must be running and properly connected to ROS 2 for simulations to work correctly.

---
