# Pose Transforms Example

Demonstrates the three pose representations provided by the SDK: raw, world-aligned, and relative.

## Overview

- Prints 5 raw pose samples (SDK coordinate frame)
- Prints 5 world-aligned pose samples (standard world frame)
- Sets a reference pose, then prints 10 relative delta poses
- Also supports controller-only mode with configurable duration and rate

## Code

```python
#!/usr/bin/env python3
import argparse
import time
import xvisio

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller-port",     default="/dev/ttyUSB0")
    parser.add_argument("--controller-duration", type=float, default=10.0)
    parser.add_argument("--controller-rate",     type=float, default=5.0)
    args = parser.parse_args()

    devices = xvisio.discover()
    if not devices:
        controller_devices = xvisio.discover_controllers()
        if not controller_devices:
            print("ERROR: No devices found")
            return
        # Controller-only mode
        dev = xvisio.open_controller(port=args.controller_port)
        sleep_s = 1.0 / args.controller_rate
        end = time.time() + args.controller_duration
        while time.time() < end:
            left, right = dev.controller()
            for name, c in [("Left", left), ("Right", right)]:
                if c:
                    print(f"{name}  pos={c.position}  quat={c.quat_wxyz}")
            time.sleep(sleep_s)
        dev.close()
        return

    with xvisio.open() as dev:
        dev.enable(slam=True)
        time.sleep(2.0)

        # 1. Raw poses
        print("Raw poses (SDK frame):")
        for i in range(5):
            pose = dev.pose()
            print(f"  [{i+1}] pos={pose.position}  conf={pose.confidence:.2f}")
            time.sleep(0.2)

        # 2. World-aligned poses
        print("\nWorld-aligned poses:")
        for i in range(5):
            pose = dev.pose_world_aligned()
            print(f"  [{i+1}] pos={pose.position}  quat={pose.quat_wxyz}")
            time.sleep(0.2)

        # 3. Relative poses
        print("\nRelative poses (delta from reference):")
        dev.reset_pose_reference()
        for i in range(10):
            pose = dev.pose_relative()
            print(f"  [{i+1}] pos={pose.position}  quat={pose.quat_wxyz}")
            time.sleep(0.2)

if __name__ == "__main__":
    main()
```

## Explanation

1. **`dev.pose()`** — Raw SDK output. Position and orientation are in the camera's native coordinate frame. Use this when you need the unmodified SLAM output or want to apply a custom transform.

2. **`dev.pose_world_aligned()`** — Rotates both position and orientation using a fixed offset matrix so that the result follows a standard right-handed world frame (x forward, z up). This matches the transform applied in the ROS2 teleop handler and is the most common representation for robotics.

3. **`dev.reset_pose_reference()` + `dev.pose_relative()`** — Stores the current pose as an origin, then each subsequent call returns the *delta* from that origin. On the first call after reset the result is (0, 0, 0) / identity quaternion. Use this for teleoperation where you care about how much the device has moved, not where it is in absolute space.

## Pose Transform Math

```
# World-aligned
pos_world = R_offset @ pos_raw
R_world   = R_offset @ R_raw @ R_offset.T

# Relative
pos_rel = R_offset @ R_init.T @ (pos_raw - pos_init)
R_rel   = R_offset @ R_init.T @ R_raw @ R_offset.T
```

where `R_offset = [[0,0,1],[1,0,0],[0,1,0]]` for the XR-50 camera.

## Next Steps

- [Device API](../api/device.md) — `reset_controller_reference()` for per-side controller delta poses
- [Pose + IMU Example](pose_imu.md) — Streaming with IMU
- [3D Visualization Example](visualization.md) — See the difference between pose types live
