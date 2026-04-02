# Quickstart

Get streaming pose and IMU data from your XvisioTech device in under 5 minutes.

## 1. Install

```bash
sudo ./scripts/setup_host.sh   # one-time: installs udev rules + XVSDK driver
pip install xvisio
```

## 2. Discover Devices

```python
import xvisio

devices = xvisio.discover()
print(devices)
# [DeviceInfo(serial_number='SNXR501G10002222006246', model='XR-50')]
```

## 3. Stream Pose and IMU

```python
import xvisio
import time

with xvisio.open() as dev:
    dev.enable(slam=True, imu=True)
    time.sleep(2.0)  # wait for SLAM to initialize

    for _ in range(10):
        pose = dev.pose_world_aligned()
        imu  = dev.imu()
        print(f"pos={pose.position}  conf={pose.confidence:.2f}")
        print(f"accel={imu.accel}  gyro={imu.gyro}")
        time.sleep(0.1)
```

## 4. Stream Controller Data

```python
import xvisio
import time

dev = xvisio.open_controller()   # auto-detects on /dev/ttyUSB0

for _ in range(50):
    left, right = dev.controller()
    if right:
        print(f"right pos={right.position}  trigger={right.key_trigger}")
    time.sleep(0.02)

dev.close()
```

## 5. Use the Iterator API

For continuous streaming, use the built-in iterators:

```python
import xvisio

with xvisio.open() as dev:
    dev.enable(slam=True)

    for pose in dev.poses(rate_hz=30):
        print(pose.position, pose.confidence)
        # press Ctrl+C to stop
```

## Next Steps

- [API Reference](api/index.md) — Full Device, Pose, and controller API
- [Pose Transforms Example](examples/pose_transforms.md) — Raw vs world-aligned vs relative poses
- [Pose + IMU Example](examples/pose_imu.md) — Complete demo script with error handling
- [3D Visualization Example](examples/visualization.md) — Live pose visualization in the browser
