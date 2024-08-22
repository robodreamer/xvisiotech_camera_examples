# XvisioTech Camera Examples

This repository contains the driver and examples for using [XvisioTech](https://www.xvisiotech.com/) cameras, specifically the XR-50 and DS-80 models. It provides the necessary instructions to install the driver, set up the ROS2 package, and get started with the camera.

## Prerequisites

Before proceeding with the installation, ensure your system is running Ubuntu22.04 and ROS2 Humble.

## Installation

### Updating Ubuntu

Ensure your Ubuntu distribution is up to date, including the latest stable kernel:

```bash
sudo apt-get update
```

### Dependency Installation

Install the dependent libraries required for the camera driver:

```bash
sudo apt-get update
sudo apt-get install -y tree g++ cmake cmake-curses-gui pkg-config autoconf libtool libudev-dev libjpeg-dev zlib1g-dev libopencv-dev rapidjson-dev libeigen3-dev libboost-thread-dev libboost-filesystem-dev libboost-system-dev libboost-program-options-dev libboost-date-time-dev liboctomap-dev
```


### USB Permissions

Set up USB permissions by downloading the permissions file and copying it to the correct directory:

1. Download `99-xvisio.rules`.
2. Open a terminal in the directory where you downloaded the file and run:

```bash
sudo cp 99-xvisio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The camera driver is now installed and ready to use. You can proceed to the next section to set up the ROS2 package and start using the camera.



### Driver Installation

Install the XvisioTech camera driver using the provided Debian package:

```bash
sudo dpkg -i XVSDK_jammy_amd64_022324.deb
```

Test the installation by running the following command:

```bash
sudo all_stream
```

This command will display the output of the camera in the terminal. If the camera is connected and working properly, you should see the camera's output in the terminal (pose and IMU signals) and camera images.


### Update Firmware

To update the camera firmware, use the latest firmware image file in the `firmware` folder and run the following command (choose the correct firmware file for your camera model):
```
./xvisioUpdateImg <firmware_file>
```

Wait until `Done` is displayed in the terminal to complete the firmware update. After the update is complete, restart the camera by disconnecting and reconnecting the USB cable. Note that this step needs to be done only once.

## Setting Up ROS2 Package

To set up the ROS2 package for the XvisioTech camera, follow these steps:

1. Create a ros2 workspace (copy the package into existing one) and build the package:

```bash
mkdir -p ~/ros2_ws/src
cp xvsdk-ros2 ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select xv_ros2_msgs
colcon build --packages-select xv_sdk_ros2
source ~/ros2_ws/install/setup.bash
```

2. Launch the ROS2 node:

```bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py
```


## Usage

To verify the installation and setup, you can listen to the camera's pose topic or visualize it using RViz:

- Listen to pose topic:

```bash
ros2 topic echo /xv_sdk/slam/pose
```

- Visualize the camera pose in RViz:

```bash
rviz2 ./xvisio_camera.rviz
```

The RViz window will open and display the camera's pose in the 3D space. You can also view the images from the camera (fish-eye stereo and RGB images) in the RViz window.


## Maintainer

For any issues or inquiries regarding the camera driver and examples, please contact:

Andy Park <andypark.purdue@gmail.com>

## References

For more information on the XvisioTech cameras and their features, please refer to the official documentation:

[https://www.xvisiotech.com/doc/](https://www.xvisiotech.com/doc/)

