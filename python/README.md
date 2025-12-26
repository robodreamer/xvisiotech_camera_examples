# Xvisio Python SDK

ROS-free Python SDK for Xvisio XR-50 devices.

## Quick Start

### 1. One-time host setup (requires sudo)

```bash
sudo ./scripts/setup_host.sh
```

This installs:
- udev rules for device access
- XVSDK driver package

### 2. Install Python package

```bash
pixi run install
```

### 3. Run demo

```bash
pixi run demo_pose_imu
```

### 4. Run tests

- Unit tests (no hardware needed):

```bash
pixi run test
```

- Hardware smoke test (requires a connected device):

```bash
XVISIO_HARDWARE=1 pixi run test
```

## Usage

```python
import xvisio

# Open device (context manager handles cleanup)
with xvisio.open() as dev:
    # Enable streams
    dev.enable(slam=True, imu=True)

    # Get pose
    pose = dev.pose()
    print(f"Position: {pose.position}")
    print(f"Quaternion: {pose.quat_wxyz}")

    # Get IMU
    imu = dev.imu()
    print(f"Accel: {imu.accel}, Gyro: {imu.gyro}")

    # Iterate over poses
    for pose in dev.poses(rate_hz=200):
        print(pose.position, pose.confidence)
```

## API Reference

### `xvisio.open(serial_number=None) -> Device`

Open a device. If `serial_number` is `None`, opens the first available device.

### `xvisio.discover() -> list[DeviceInfo]`

Discover all available devices.

### `Device.enable(slam=True, imu=False)`

Enable device streams.

### `Device.pose(prediction_s=0.0) -> Pose`

Get current pose with optional prediction time (raw pose from SDK).

### `Device.pose_world_aligned(prediction_s=0.0) -> Pose`

Get current pose aligned with world frame. Applies rotation offset to align camera frame with world frame. This matches the transform used in the ROS2 teleop handler.

### `Device.pose_relative(prediction_s=0.0) -> Pose`

Get current pose relative to the reference pose. Returns delta pose from the last call to `reset_pose_reference()`. If `reset_pose_reference()` hasn't been called, automatically initializes on first call. Useful for teleoperation scenarios.

### `Device.reset_pose_reference()`

Reset the reference pose for relative pose calculations. Call this when you want to start tracking relative to the current pose.

### `Device.pose_at(host_timestamp_s) -> Pose`

Get pose at a specific timestamp.

### `Device.poses(rate_hz=None) -> Iterator[Pose]`

Iterator over pose updates.

### `Device.imu() -> ImuSample`

Get current IMU sample.

### `Device.imus(rate_hz=None) -> Iterator[ImuSample]`

Iterator over IMU samples.

## Data Types

- `Pose`: position (3-tuple), quat_wxyz (4-tuple), host_timestamp_s, edge_timestamp_us, confidence
- `ImuSample`: accel (3-tuple), gyro (3-tuple), host_timestamp_s, edge_timestamp_us
- `DeviceInfo`: serial_number, model

## Troubleshooting

- **`ImportError: Failed to import xvisio native module`**:
  - Run `pixi run install`
  - Confirm XVSDK is installed: `ls /usr/lib/libxvsdk.so`
- **No devices found**:
  - Run `sudo ./scripts/setup_host.sh`
  - Unplug/replug the device
  - If you were just added to `plugdev`, log out/in
- **Build succeeds but stub generation fails**:
  - This is normal if SuiteSparse libraries aren't installed
  - The module works fine without `.pyi` files (they're just for IDE autocomplete)
  - To enable stub generation: `sudo apt-get install libsuitesparse-dev`
  - This installs all SuiteSparse dependencies (`libspqr2`, `libcxsparse3`, etc.)

