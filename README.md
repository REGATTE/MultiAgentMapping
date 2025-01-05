# MultiAgentMapping

## Installation

```bash
cd ros2_ws/src
git clone https://github.com/REGATTE/MultiAgentMapping.git
```

**Install LIO-SAM**

```bash
cd ros2_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2

sudo apt install ros-<ros2-version>-perception-pcl \
  	   ros-<ros2-version>-pcl-msgs \
  	   ros-<ros2-version>-vision-opencv \
  	   ros-<ros2-version>-xacro
```

**Eigen == 3.4.0**

Install GTSAM

```
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

```
cd ros2_ws
colcon build --symlink-install
```

## Extension

### Install Extension

Open Isaac Sim, and go to `Window > Extensions`.

Click on the `3 stipes > setting`. In the panel that opens, click on the `green (+) button` and enter the absolute path to the `isaacsim_extension/exts`.

This will load your extension.