# Device API

## Module-Level Functions

### `xvisio.open`

```python
xvisio.open(
    serial_number: str | None = None,
    timeout_s: float = 2.0,
    controller_only: bool = False,
) -> Device
```

Open a device by serial number. If `serial_number` is `None` or empty, opens the first available device.

**Raises:** `RuntimeError` if no device is found or device cannot be opened.

```python
import xvisio

# Open first available device
with xvisio.open() as dev:
    dev.enable(slam=True)
    pose = dev.pose_world_aligned()

# Open by serial number
dev = xvisio.open("SNXR501G10002222006246")
```

---

### `xvisio.discover`

```python
xvisio.discover(timeout_s: float = 2.0) -> list[DeviceInfo]
```

Discover all connected Xvisio devices. Returns an empty list if none are found.

```python
devices = xvisio.discover()
for d in devices:
    print(d.serial_number, d.model)
```

---

### `xvisio.open_controller`

```python
xvisio.open_controller(port: str = "/dev/ttyUSB0", timeout_s: float = 2.0) -> Device
```

Open a Seer wireless controller device and start controller streaming. Convenience wrapper around `open()` + `enable_controller()`.

```python
dev = xvisio.open_controller()          # auto-detects on /dev/ttyUSB0
dev = xvisio.open_controller("/dev/ttyUSB1")
```

---

### `xvisio.discover_controllers`

```python
xvisio.discover_controllers(timeout_s: float = 2.0) -> list[DeviceInfo]
```

Discover Seer wireless controller devices only.

---

## Device Class

The `Device` class is the main interface for interacting with an Xvisio device. Always obtain one via `xvisio.open()` or `xvisio.open_controller()` — do not construct directly.

`Device` supports the context manager protocol for automatic cleanup:

```python
with xvisio.open() as dev:
    ...
# dev.close() called automatically
```

---

### Properties

| Property | Type | Description |
|----------|------|-------------|
| `serial_number` | `str` | Device serial number |

---

### Lifecycle

#### `enable`

```python
dev.enable(slam: bool = True, imu: bool = False)
```

Enable device streams. Call once after opening.

```python
dev.enable(slam=True)           # SLAM only
dev.enable(slam=True, imu=True) # SLAM + IMU
dev.enable(imu=True)            # IMU only (no SLAM)
```

**Raises:** `RuntimeError` if a stream fails to start.

#### `close`

```python
dev.close()
```

Stop all active streams and release the device. Called automatically when using the context manager.

---

### Camera Pose

#### `pose`

```python
dev.pose(prediction_s: float = 0.0) -> Pose
```

Get the current 6DoF pose as reported by the SLAM algorithm (raw SDK coordinate frame).

**Raises:** `RuntimeError` if SLAM is not enabled.

#### `pose_world_aligned`

```python
dev.pose_world_aligned(prediction_s: float = 0.0) -> Pose
```

Get pose rotated into a standard world frame. Applies the rotation offset:

```
R_world = R_offset @ R_camera @ R_offset.T
pos_world = R_offset @ pos_camera
```

where `R_offset = [[0,0,1],[1,0,0],[0,1,0]]` (camera z+ → world x+, etc.).

This matches the transform used in the ROS2 teleop handler.

#### `pose_relative`

```python
dev.pose_relative(prediction_s: float = 0.0) -> Pose
```

Get pose relative to the last call to `reset_pose_reference()`. Auto-initializes on first call. Useful for teleoperation (delta poses).

#### `reset_pose_reference`

```python
dev.reset_pose_reference()
```

Store the current pose as the origin for subsequent `pose_relative()` calls.

#### `pose_at`

```python
dev.pose_at(host_timestamp_s: float) -> Pose
```

Get the pose at a specific host timestamp.

#### `poses`

```python
dev.poses(rate_hz: float | None = None) -> Iterator[Pose]
```

Iterator that yields pose updates continuously. Use `rate_hz` to throttle; `None` yields as fast as possible. Stops on `KeyboardInterrupt`.

```python
for pose in dev.poses(rate_hz=30):
    print(pose.position)
```

---

### IMU

#### `imu`

```python
dev.imu(timeout_s: float = 0.5) -> ImuSample
```

Get the latest IMU sample. Waits up to `timeout_s` for data to arrive after `enable(imu=True)`.

**Raises:** `RuntimeError` if IMU is not enabled or no data arrives within the timeout.

#### `imus`

```python
dev.imus(rate_hz: float | None = None) -> Iterator[ImuSample]
```

Iterator that yields IMU samples continuously.

---

### Controller

#### `enable_controller`

```python
dev.enable_controller(port: str = "/dev/ttyUSB0")
```

Start the Seer wireless controller on the given serial port. Called automatically by `xvisio.open_controller()`.

**Raises:** `RuntimeError` if the controller cannot be started.

#### `disable_controller`

```python
dev.disable_controller()
```

Stop the Seer wireless controller.

#### `controller`

```python
dev.controller() -> tuple[ControllerData | None, ControllerData | None]
```

Get the latest left and right controller data in the raw SDK coordinate frame. Either value can be `None` if no data has arrived yet.

```python
left, right = dev.controller()
if right:
    print(right.position, right.key_trigger)
```

#### `controller_left` / `controller_right`

```python
dev.controller_left()  -> ControllerData | None
dev.controller_right() -> ControllerData | None
```

Convenience accessors for a single controller side.

#### `controller_world_aligned`

```python
dev.controller_world_aligned() -> tuple[ControllerData | None, ControllerData | None]
```

Controller data with orientation rotated into world frame (using the Seer controller rotation offset).

#### `controller_relative`

```python
dev.controller_relative() -> tuple[ControllerData | None, ControllerData | None]
```

Controller data as delta from the last `reset_controller_reference()` call. Auto-initializes independently per side on first observation.

#### `reset_controller_reference`

```python
dev.reset_controller_reference(side: Literal["left", "right", "both"] = "both") -> bool
```

Store the current controller pose(s) as the origin for `controller_relative()`.

| `side` | Effect |
|--------|--------|
| `"both"` | Reset left and right simultaneously (default) |
| `"left"` | Reset left controller only |
| `"right"` | Reset right controller only |

Returns `True` if all requested sides were reset; `False` if a side had no data yet.

---

## Pose Transform Summary

| Variant | Method | Coordinate frame |
|---------|--------|-----------------|
| Raw | `pose()` / `controller()` | SDK default (camera-centric) |
| World-aligned | `pose_world_aligned()` / `controller_world_aligned()` | Standard world (x forward, z up) |
| Relative | `pose_relative()` / `controller_relative()` | Delta from reference pose |

## Next Steps

- [Data Types](types.md) — `Pose`, `ImuSample`, `ControllerData`, `DeviceInfo`
- [Pose Transforms Example](../examples/pose_transforms.md) — Side-by-side comparison of all three pose types
