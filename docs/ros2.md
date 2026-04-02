# ROS2 Driver (Docker)

The repository also includes a full **ROS2 Humble driver** for XvisioTech cameras, delivered via Docker. This is the recommended path if you need ROS topics, RViz visualization, or integration into a ROS2 workspace.

!!! note "Just want Python scripting?"
    If your goal is ROS-free Python scripting, use the [Python SDK](quickstart.md) instead — it requires no Docker or ROS installation.

All ROS2-related files are grouped under `ros2/` in the repository:

```
ros2/
├── xvsdk-ros2/          ← ROS2 packages (xv_ros2_msgs, xv_sdk_ros2)
├── docker/              ← Dockerfile and devcontainer config
└── scripts/
    ├── start_container.sh
    ├── open_shell.sh
    └── setup_ros2_driver.sh
```

## Prerequisites

- Docker installed — follow the [official Docker install guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) (use `apt-get`, not Docker Desktop)
- XvisioTech camera connected via USB and powered on

**Add your user to the docker group (one-time):**

```bash
sudo usermod -aG docker $USER
newgrp docker   # or log out and back in
```

**Enable X11 forwarding for GUI apps:**

```bash
xhost +local:docker
```

## Quickstart

### 1. Start the container

```bash
./ros2/scripts/start_container.sh
```

This builds the Docker image (first run only) and starts the container with the XVSDK driver pre-installed.

### 2. Verify the driver

Inside the container:

```bash
sudo all_stream
```

You should see pose and IMU output in the terminal if the camera is connected.

### 3. Build and launch the ROS2 driver

```bash
source ./ros2/scripts/setup_ros2_driver.sh
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py
```

!!! warning "Use `source`, not `bash`"
    Run with `source` so the ROS2 environment is set up in the current shell.

### 4. Open a second shell in the container

```bash
./ros2/scripts/open_shell.sh
```

### 5. Inspect topics

```bash
# Listen to pose
ros2 topic echo /xv_sdk/<serial_id>/pose

# Visualize in RViz
rviz2 -d ./xvisio-camera-test.rviz
```

Replace `<serial_id>` with your camera's serial number (e.g. `SNXR501G10002222006246`) as shown in the topic list.

## ROS2 Topics

| Topic | Type | Default |
|-------|------|---------|
| `.../pose` | `geometry_msgs/PoseStamped` | Enabled |
| `.../button_states` | `xv_ros2_msgs/ButtonMsg` | Enabled |
| `.../fisheye/left` | `sensor_msgs/Image` | Disabled |
| `.../fisheye/right` | `sensor_msgs/Image` | Disabled |
| `.../rgb` | `sensor_msgs/Image` | Disabled |
| `.../tof` | `sensor_msgs/Image` | Disabled |
| `.../imu` | `sensor_msgs/Imu` | Disabled |
| `.../orientation` | `xv_ros2_msgs/OrientationStamped` | Disabled |
| `.../slam_path` | `nav_msgs/Path` | Disabled |

## Launch Parameters

Enable optional streams by passing parameters to the launch file:

```bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py fisheye_enable:=true rgb_enable:=true
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `slam_pose_enable` | `true` | 6DoF SLAM pose |
| `button_states_enable` | `true` | Controller button states |
| `fisheye_enable` | `false` | Stereo fisheye images |
| `rgb_enable` | `false` | RGB camera images |
| `tof_enable` | `false` | Time-of-Flight depth |
| `rgbd_enable` | `false` | RGB-D combined |
| `imu_enable` | `false` | IMU data |
| `orientation_enable` | `false` | Orientation data |
| `event_enable` | `false` | Event data |
| `slam_path_enable` | `false` | SLAM trajectory path |

Disabling unused topics reduces CPU usage and network bandwidth.

## Manual Installation (without Docker)

<details>
<summary>Expand for manual ROS2 Humble setup on Ubuntu 22.04</summary>

**1. Install dependencies:**

```bash
sudo apt-get update
sudo apt-get install -y tree g++ cmake cmake-curses-gui pkg-config autoconf libtool \
  libudev-dev libjpeg-dev zlib1g-dev libopencv-dev rapidjson-dev libeigen3-dev \
  libboost-thread-dev libboost-filesystem-dev libboost-system-dev \
  libboost-program-options-dev libboost-date-time-dev liboctomap-dev
```

**2. Install the XVSDK driver:**

```bash
sudo dpkg -i drivers/XVSDK_jammy_amd64_20250227.deb
```

**3. Install udev rules:**

```bash
sudo cp drivers/99-xvisio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**4. Build the ROS2 packages:**

```bash
mkdir -p ~/ros2_ws/src
cp -r ros2/xvsdk-ros2/xv_ros2_msgs ~/ros2_ws/src/
cp -r ros2/xvsdk-ros2/xv_sdk_ros2  ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select xv_ros2_msgs
colcon build --packages-select xv_sdk_ros2
source ~/ros2_ws/install/setup.bash
```

**5. Launch:**

```bash
ros2 launch xv_sdk_ros2 xv_sdk_node_launch.py
```

</details>

## Firmware Update

If you just received the camera or need to update the firmware:

```bash
# Inside the container or on the host with the XVSDK installed
./xvisioUpdateImg firmware/<firmware_file>
```

Choose the correct file for your model:

| Model | Latest firmware |
|-------|----------------|
| XR-50 | `firmware/xr50_20250109.img` |
| DS-80 | `firmware/ds80_20250113.img` |

Wait for `Done` in the terminal, then reconnect the USB cable to complete the update.
