# Pose + IMU Example

Minimal demo showing world-aligned pose and IMU streaming. Includes automatic fallback to Seer controller-only mode.

## Overview

- Discovers and opens the first available device (camera or controller)
- Enables SLAM and IMU streams
- Prints world-aligned pose and IMU data at ~10 Hz for 10 seconds
- Falls back to controller-only mode if no camera is found

## Code

```python
#!/usr/bin/env python3
import argparse
import xvisio
import time

def main():
    parser = argparse.ArgumentParser(description="Xvisio pose + IMU demo")
    parser.add_argument("--controller-port", default="/dev/ttyUSB0")
    args = parser.parse_args()

    # 1. Discover devices
    devices = xvisio.discover()
    controller_devices = []
    if not devices:
        controller_devices = xvisio.discover_controllers()
        if not controller_devices:
            print("ERROR: No Xvisio devices found!")
            return

    # 2. Controller-only fallback
    if not devices and controller_devices:
        dev = xvisio.open_controller(port=args.controller_port)
        start = time.time()
        while time.time() - start < 10.0:
            left, right = dev.controller()
            for name, c in [("L", left), ("R", right)]:
                if c:
                    print(f"{name} pos={c.position}  trigger={c.key_trigger}")
            time.sleep(0.1)
        dev.close()
        return

    # 3. Camera path
    with xvisio.open() as dev:
        dev.enable(slam=True, imu=True)
        time.sleep(2.0)   # wait for SLAM to initialize

        start = time.time()
        while time.time() - start < 10.0:
            pose = dev.pose_world_aligned()
            imu  = dev.imu()
            elapsed = time.time() - start

            print(f"{elapsed:5.2f}s  pos={pose.position}  conf={pose.confidence:.2f}")
            print(f"         accel={imu.accel}  gyro={imu.gyro}")
            time.sleep(0.1)

if __name__ == "__main__":
    main()
```

## Explanation

1. **Device discovery** — `xvisio.discover()` returns a list of camera devices; `xvisio.discover_controllers()` returns Seer controller devices. If only a controller is found the script falls back to controller mode.
2. **`dev.enable(slam=True, imu=True)`** — Starts both the SLAM tracker and IMU callback in a single call. Either stream can be enabled independently.
3. **`time.sleep(2.0)`** — SLAM uses fisheye cameras that need a few frames to initialize. Moving the device slightly while waiting helps convergence.
4. **`dev.pose_world_aligned()`** — Returns pose in a standard world frame (x forward, z up) rather than the raw camera-centric frame. See [Pose Transforms](pose_transforms.md) for the difference.
5. **`dev.imu()`** — Returns the latest accelerometer (m/s²) and gyroscope (rad/s) sample.
6. **Context manager** — `with xvisio.open() as dev:` ensures streams are stopped and the device is released even if an exception occurs.

## Next Steps

- [Pose Transforms Example](pose_transforms.md) — Compare raw, world-aligned, and relative poses
- [3D Visualization Example](visualization.md) — Visualize the pose live in a browser
- [Device API](../api/device.md) — Full API reference
