# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## Python SDK (pip package: `xvisio`)

### [0.2.0] - 2025-01-09

Initial release of the ROS-free Python SDK for Xvisio devices.

#### Added
- **Core Python API**
  - `xvisio.discover()` - Discover connected Xvisio devices
  - `xvisio.open()` - Open device with context manager support
  - `Device.enable(slam=True, imu=True)` - Enable SLAM and/or IMU streams
  - `Device.pose()` - Get raw pose from SDK
  - `Device.pose_world_aligned()` - Get pose aligned with world frame
  - `Device.pose_relative()` - Get pose relative to reference
  - `Device.imu()` - Get IMU data (accelerometer, gyroscope)
  - `Device.poses(rate_hz)` - Iterator for continuous pose streaming

- **CLI Commands**
  - `xvisio-setup` - System setup (udev rules, driver installation)
  - `xvisio-examples --list` - List available example scripts
  - `xvisio-examples --copy` - Copy examples to local directory

- **Examples** (included in package)
  - `demo_pose_imu.py` - Basic pose + IMU demo
  - `demo_pose_visualization.py` - 3D visualization with viser
  - `demo_pose_transforms.py` - Pose transform methods demo
  - `benchmark_pose_rate.py` - Pose update rate benchmark

- **Optional Dependencies**
  - `xvisio[examples]` - Install with viser for visualization examples
  - `xvisio[visualization]` - Same as examples

- **Documentation**
  - Installation guide for pip users (`PIP_INSTALL.md`)
  - Publishing guide (`PUBLISHING.md`)

#### Technical Details
- Built with nanobind for high-performance Python bindings
- scikit-build-core for CMake-based wheel building
- Compatible with Python 3.10, 3.11, 3.12
- Linux x86_64 support (requires XVSDK driver)

---

## ROS2 Driver

### [Unreleased] - 2025-03-03

### Added
- Optional feature flags to reduce CPU usage and bandwidth
- Parameter logging for better debugging
- Support for making various topics optional (RGB, ToF, RGBD, fisheye, event, orientation, trajectory)
- Support for Seer wireless controller

### Changed
- Improved variable naming consistency throughout the codebase
- Replaced std::cout with ROS2 logging for better integration
- Updated README with optional feature documentation
- Muted spamming messages for controller data
- Updated permission for ttyUSB device for Seer controller
- Changed button states to be enabled by default
- Fixed event stream initialization to support button states

### Fixed
- Fixed SGBM publisher methods to properly check for fisheye_enable parameter
- Fixed UVC frame errors when fisheye cameras are disabled
- Fixed firmware update dependency to run inside the container
- Cleaned up driver logging messages
- Fixed button states not being published when event_enable was false

## [1.1.0] - 2025-01-09

### Added
- Docker container setup for easy installation
- Latest firmware for DS-80 camera
- Publish 4 button states on XR-50 camera with adapter

### Changed
- Updated Docker setup instructions in README
- Code cleanup to address warning messages from build
- Added purchasing link to the references

## [1.0.0] - 2024-03-06

### Added
- Initial release of the XvisioTech camera driver for ROS2
- Support for XR-50 and DS-80 camera models
- Basic SLAM functionality
- IMU data publishing
- ROS2 launch files and configuration

