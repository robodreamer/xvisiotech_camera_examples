# XvisioTech Camera Examples

This repository contains the driver and examples for using [XvisioTech](https://www.xvisiotech.com/) cameras, specifically the XR-50 and DS-80 models. It provides the necessary instructions to install the driver, set up the ROS2 package, and get started with the camera.

## Instructions (Docker Container) -- Recommended

### Prerequisites

Before proceeding with the installation, ensure you have Docker installed on your system. If you don't have Docker installed, you can follow the instructions provided on [the official Docker website](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository). Make sure to use `apt-get` to install it, instead of `docker-desktop` GUI app installation, as it may not work with the provided scripts.

#### Docker Setup:

If you haven't done this before, add the user to the docker group:

```bash
sudo usermod -aG docker $USER
```

Log out and log back in for the group change to take effect. Alternatively, run:

```bash
newgrp docker
```

Note that to be able to run examples with GUIs, make sure graphics forwarding (x11) is enabled in your docker settings. We have added this in the provided scripts for convenience, but you may need to enable it manually if you are using a different setup.
```bash
xhost +local:docker
```

Make sure the device is connected to the host machine and the camera is powered on. If you have just received the camera, you may need to update the firmware to the latest version. Please refer to the "Update Firmware" section below for instructions.

### Installation

Start the Docker container with the necessary dependencies and the camera driver installed:

```bash
./scripts/start_container.sh
```

Once the docker image is built and the container is started, you can proceed with setting up the ROS2 package in the container.

Test the driver installation by running the following command:
```bash
sudo all_stream
```

This command will display the output of the camera in the terminal. If the camera is connected and working properly, you should see the camera's output in the terminal (pose and IMU signals) and camera images.

To set up the ROS2 package for the XvisioTech camera, follow these steps in the container:
```bash
source ./scripts/setup_ros2_driver.sh
```
Make sure to `source` the script to set up the ROS2 package in the current shell. Just running the script without `source` will not set up the ROS2 package in the current shell.

### Usage

This script will build the ROS2 package. You can then launch the ROS2 node using the following command in the container:
```bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py
```

To verify the installation and setup, you can listen to the camera's pose topic or visualize it using RViz (in a new shell) as follows:

Open a new shell in the container currently running the ROS2 node:
```bash
./scripts/open_shell.sh
```

You can listen to pose topic in the new shell:
```bash
ros2 topic echo /xv_sdk/<camera_serial_id>/pose
```
<camera_serial_id> is the serial number of the camera that shows up in the topic list (e.g., `SNXR501G10002222006246`).


You can also visualize the camera pose in RViz (in the new shell):
```bash
rviz2 -d ./xvisio-camera-test.rviz
```

### Optional Features

The XvisioTech camera driver supports several optional features that can be enabled or disabled through launch parameters. By default, most features are disabled to reduce CPU usage and bandwidth. You can enable specific features based on your requirements.

To enable a feature, add the corresponding parameter to the launch command:

```bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py <parameter_name>:=true
```

Available parameters:

- **fisheye_enable**: Enable fisheye camera images (default: false)
- **rgb_enable**: Enable RGB camera images (default: false)
- **tof_enable**: Enable Time-of-Flight depth camera (default: false)
- **rgbd_enable**: Enable RGB-D combined data (default: false)
- **button_states_enable**: Enable button state publishing (default: false)
- **slam_pose_enable**: Enable SLAM pose publishing (default: true)

For example, to enable fisheye cameras and RGB images (for DS-80 cameras):

```bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py fisheye_enable:=true rgb_enable:=true
```

Enabling these features will make the corresponding topics available for subscription. Disabling them will prevent the topics from being created, reducing CPU usage and network bandwidth.

## Update Firmware

If you just received the camera, you may need to update the firmware to the latest version.

To update the camera firmware, use the latest firmware image file in the `firmware` folder and run the following command inside the container shell (choose the correct firmware file for your camera model):
```
./xvisioUpdateImg <firmware_file>
```

Wait until `Done` is displayed in the terminal to complete the firmware update. After the update is complete, restart the camera by disconnecting and reconnecting the USB cable. Note that this step needs to be done only once.


## Instructions with Manual Installation

This section is only for reference. We recommend using the Docker container as it is easier to install and use.

### Prerequisites

Before proceeding with the installation, ensure your system is running Ubuntu22.04 and ROS2 Humble.

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
rviz2 -d ./xvisio-camera-test.rviz
```

The RViz window will open and display the camera's pose in the 3D space. You can also view the images from the camera (fish-eye stereo and RGB images) in the RViz window.


## Maintainer

For any issues or inquiries regarding the camera driver and examples, please contact:

Andy Park <andypark.purdue@gmail.com>

## References

For more information on the XvisioTech cameras and their features, please refer to the official documentation:

[https://www.xvisiotech.com/doc/](https://www.xvisiotech.com/doc/)

[Purchasing Xvisio Camera Modules (XR-50, DS-80) on Mouser Electronics Website](https://www.mouser.com/c/optoelectronics/cameras-accessories/cameras-camera-modules/?m=Xvisio&product=Camera%20Modules&product%20type=AI%2C%20CV%2C%20VSLAM)

## Documentation

For detailed information about changes between versions, please see the [Changelog](CHANGELOG.md).