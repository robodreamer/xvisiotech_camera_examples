# Data Types

All types are importable from `xvisio`:

```python
from xvisio import Pose, ImuSample, ControllerData, DeviceInfo
```

---

## Pose

6DoF pose from SLAM. Returned by `dev.pose()`, `dev.pose_world_aligned()`, and `dev.pose_relative()`.

| Field | Type | Description |
|-------|------|-------------|
| `position` | `tuple[float, float, float]` | (x, y, z) in meters |
| `quat_wxyz` | `tuple[float, float, float, float]` | Quaternion (w, x, y, z) |
| `host_timestamp_s` | `float` | Host-side timestamp in seconds |
| `edge_timestamp_us` | `int` | Device-side timestamp in microseconds |
| `confidence` | `float` | SLAM tracking confidence (0.0–1.0) |

```python
pose = dev.pose_world_aligned()

x, y, z = pose.position
w, qx, qy, qz = pose.quat_wxyz

print(f"pos=({x:.3f}, {y:.3f}, {z:.3f})  conf={pose.confidence:.2f}")
```

!!! note "Quaternion convention"
    All quaternions use **w-first** order: `(w, x, y, z)`. The alias `pose.quaternion` is provided for compatibility and returns the same value as `pose.quat_wxyz`.

---

## ImuSample

IMU measurement. Returned by `dev.imu()`.

| Field | Type | Description |
|-------|------|-------------|
| `accel` | `tuple[float, float, float]` | Linear acceleration (x, y, z) in m/s² |
| `gyro` | `tuple[float, float, float]` | Angular velocity (x, y, z) in rad/s |
| `host_timestamp_s` | `float` | Host-side timestamp in seconds |
| `edge_timestamp_us` | `int` | Device-side timestamp in microseconds |

```python
imu = dev.imu()
ax, ay, az = imu.accel   # m/s²
gx, gy, gz = imu.gyro    # rad/s
```

---

## ControllerData

Seer wireless controller pose and button state. Returned by `dev.controller()` and variants.

| Field | Type | Description |
|-------|------|-------------|
| `type` | `str` | `"left"` or `"right"` |
| `position` | `tuple[float, float, float]` | (x, y, z) in meters |
| `quat_wxyz` | `tuple[float, float, float, float]` | Quaternion (w, x, y, z) |
| `host_timestamp_s` | `float` | Host-side timestamp in seconds |
| `key_trigger` | `int` | Trigger button state |
| `key_side` | `int` | Side (grip) button state |
| `rocker_x` | `int` | Joystick x-axis |
| `rocker_y` | `int` | Joystick y-axis |
| `key` | `int` | Additional button bitmask |

```python
left, right = dev.controller()
if right:
    print(f"pos={right.position}")
    print(f"trigger={right.key_trigger}  side={right.key_side}")
    print(f"rocker=({right.rocker_x}, {right.rocker_y})  key={right.key}")
```

---

## DeviceInfo

Information about a discovered device. Returned by `xvisio.discover()`.

| Field | Type | Description |
|-------|------|-------------|
| `serial_number` | `str` | Unique device serial number |
| `model` | `str` | Device model name (e.g. `"XR-50"`) |

```python
devices = xvisio.discover()
for d in devices:
    print(f"{d.serial_number}  ({d.model})")
```
