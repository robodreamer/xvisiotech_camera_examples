# API Reference

The `xvisio` package exposes a small, focused public API centered on the `Device` class.

## Module-Level Functions

| Function | Description |
|----------|-------------|
| [`xvisio.open()`](device.md) | Open a device (first available or by serial number) |
| [`xvisio.discover()`](device.md) | List all connected Xvisio devices |
| [`xvisio.open_controller()`](device.md) | Open a Seer wireless controller device |
| [`xvisio.discover_controllers()`](device.md) | List connected Seer controller devices |

## Device Class

| Method | Description |
|--------|-------------|
| [`dev.enable()`](device.md#enable) | Enable SLAM and/or IMU streams |
| [`dev.close()`](device.md#close) | Stop all streams and release device |
| [`dev.pose()`](device.md#pose) | Raw 6DoF pose from SLAM |
| [`dev.pose_world_aligned()`](device.md#pose_world_aligned) | Pose rotated into world frame |
| [`dev.pose_relative()`](device.md#pose_relative) | Pose delta from reference |
| [`dev.reset_pose_reference()`](device.md#reset_pose_reference) | Set current pose as relative origin |
| [`dev.imu()`](device.md#imu) | Latest IMU sample |
| [`dev.poses()`](device.md#poses) | Iterator over pose updates |
| [`dev.imus()`](device.md#imus) | Iterator over IMU samples |
| [`dev.enable_controller()`](device.md#enable_controller) | Start Seer wireless controller |
| [`dev.controller()`](device.md#controller) | Raw left/right controller data |
| [`dev.controller_world_aligned()`](device.md#controller_world_aligned) | Controller data in world frame |
| [`dev.controller_relative()`](device.md#controller_relative) | Controller delta from reference |
| [`dev.reset_controller_reference()`](device.md#reset_controller_reference) | Set current controller pose as relative origin |

## Data Types

| Type | Description |
|------|-------------|
| [`Pose`](types.md#pose) | 6DoF pose: position, quaternion, timestamps, confidence |
| [`ImuSample`](types.md#imusample) | IMU measurement: accel (m/s²), gyro (rad/s), timestamps |
| [`ControllerData`](types.md#controllerdata) | Controller pose + button/joystick states |
| [`DeviceInfo`](types.md#deviceinfo) | Discovered device: serial number, model |

## Pose Transform Summary

The SDK provides three pose representations for both camera and controller:

| Method | Description | Use case |
|--------|-------------|----------|
| `pose()` / `controller()` | Raw SDK output | Debugging, custom transforms |
| `pose_world_aligned()` / `controller_world_aligned()` | Rotated to standard world frame (x forward, z up) | Visualization, ROS integration |
| `pose_relative()` / `controller_relative()` | Delta from a reference pose | Teleoperation, relative motion |
