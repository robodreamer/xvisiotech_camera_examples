# Changelog

All notable changes to the XvisioTech Camera ROS2 Driver will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - 2025-03-03

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

### Fixed
- Fixed SGBM publisher methods to properly check for fisheye_enable parameter
- Fixed UVC frame errors when fisheye cameras are disabled
- Fixed firmware update dependency to run inside the container
- Cleaned up driver logging messages

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

