# Xvisio Python SDK

**ROS-free Python SDK for XvisioTech 6DoF SLAM cameras and Seer wireless controllers**

The `xvisio` package provides direct Python access to pose (SLAM), IMU, and controller data from XvisioTech devices — no ROS required. Built on a C++ core via [Nanobind](https://github.com/wjakob/nanobind) for low-overhead, high-rate data streaming.

## Supported Devices

| Device | Pose (SLAM) | IMU | Controller |
|--------|-------------|-----|------------|
| XR-50 | Yes | Yes | — |
| DS-80 | Yes | Yes | — |
| Seer Wireless Controller | — | — | Yes |

## Features

- **SLAM Pose Tracking** — 6DoF position and orientation at up to 200 Hz
- **IMU Streaming** — Accelerometer and gyroscope data (m/s², rad/s)
- **Wireless Controller** — 6DoF pose + buttons and joystick for both hands
- **Pose Transforms** — Raw, world-aligned, and relative delta poses out of the box
- **Context Manager** — Automatic resource cleanup with `with xvisio.open() as dev:`
- **Iterator API** — `dev.poses()` and `dev.imus()` for streaming loops
- **Type Hints** — Full PEP 561 support with `.pyi` stubs

## Quick Start

```python
import xvisio

# Camera (XR-50 / DS-80)
with xvisio.open() as dev:
    dev.enable(slam=True, imu=True)

    pose = dev.pose_world_aligned()
    imu  = dev.imu()

    print(pose.position, pose.confidence)
    print(imu.accel, imu.gyro)
```

```python
import xvisio

# Seer Wireless Controller
dev = xvisio.open_controller()

left, right = dev.controller()
if right:
    print(right.position, right.quat_wxyz)
    print(f"trigger={right.key_trigger}, rocker=({right.rocker_x}, {right.rocker_y})")

dev.close()
```

## Installation

```bash
# One-time host setup (udev rules + XVSDK driver)
sudo ./scripts/setup_host.sh

# Install Python package
pip install xvisio
```

See the [Installation Guide](installation.md) for full details.

## Documentation

- [Installation Guide](installation.md) — Host setup, pip install, pixi development workflow
- [Quickstart](quickstart.md) — Get streaming data in under 5 minutes
- [API Reference](api/index.md) — Complete API documentation
- [Examples](examples/index.md) — Runnable example scripts with explanations

## License

MIT License — Copyright (c) 2026 Andy Park <andypark.purdue@gmail.com>
