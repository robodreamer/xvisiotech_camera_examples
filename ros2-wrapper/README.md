# ROS2 Wrapper for Xvisio Devices

## Installation Instructions

The following instructions were verified with ROS2 Galactic on **Ubutnu 20.04**.

### Dependencies
#### Install ROS2 packages [ros-galactic-desktop](http://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)


#### Install ROS2 dependences
  ```bash
sudo apt install ros-galactic-desktop
  ```
### Install xv_ROS2_wrapper source
```bash

mkdir -p ~/ros2_ws/src
cp xv_ros2_msgs ~/ros2_ws/src
cp xv_sdk_ros2 ~/ros2_ws/src

#build
cd ~/ros2_ws
colcon build  --packages-select xv_ros2_msgs
colcon build  --packages-select xv_sdk_ros2 --cmake-args -DXVSDK_INCLUDE_DIRS="/usr/include/xvsdk" -DXVSDK_LIBRARIES="/usr/lib/libxvsdk.so"
```
**Note:**
Please install Xvisio SDK before building.

## Usage Instructions

### Start the node
To start the node in ROS2, plug in the Xvisio device, then type the following command:

```bash
source /opt/ros/galactic/setup.bash
cd ~/ros2_ws
. install/setup.bash
# To launch with "ros2 run"
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py
```

You can use the `ros2 topic list` command to view the topics provided by the node, use `ros2 service list` to view the services provided by the node.

## Known Issues

* We support Ubuntu Linux Bionic Beaver 18.04 and Ubuntu Linux Focal Fossa 20.04, but not support Ubuntu 22.04.

## Todo
* Support rviz to display images, point clouds, and slam trajectories.
* Support SGBM function（depth image and point clouds）.
* Support RGB function.
* Support RGBD funciton.
* Support cslam function.

