# XvisioTech Camera Examples

Python SDK and ROS2 driver for [XvisioTech](https://www.xvisiotech.com/) devices:

- **XR-50** — 6DoF SLAM tracking camera with stereo fisheye, RGB, and IMU
- **DS-80** — 6DoF SLAM tracking camera with ToF depth sensor
- **Seer Wireless Controller** — 6DoF motion controller with buttons and joystick

**Full documentation:** https://robodreamer.github.io/xvisiotech_camera_examples/

---

## Quick start (Python SDK)

The `xvisio` Python package provides direct access to pose, IMU, and controller data — no ROS required.

**1. One-time host setup** (installs udev rules and the XVSDK driver, requires Ubuntu 22.04):

```bash
git clone https://github.com/robodreamer/xvisiotech_camera_examples.git
cd xvisiotech_camera_examples
sudo ./scripts/setup_host.sh
```

**2. Install into a virtual environment:**

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install xvisio
```

If you prefer a one-shot Ubuntu installer that also creates the virtual environment for you, use:

```bash
bash scripts/install_xvisio_linux.sh
```

**3. Run:**

```python
import xvisio

with xvisio.open() as dev:
    dev.enable(slam=True, imu=True)
    pose = dev.pose()
    imu = dev.imu()
    print(pose.position, imu.accel)
```

For detailed installation options, troubleshooting, and the full API reference, see the **[documentation site](https://robodreamer.github.io/xvisiotech_camera_examples/)**.

Common install issues:

- If `pip` reports `THESE PACKAGES DO NOT MATCH THE HASHES FROM THE REQUIREMENTS FILE`, that usually comes from a user or system `pip` policy such as `require-hashes` or an injected constraints file. `xvisio` does not ship hashed requirements in this repo. Use a clean virtual environment and inspect `pip config debug` if the error persists.
- If `scripts/install_xvisio_linux.sh` says a venv already exists but cannot activate it, remove the broken environment and rerun the script, or update to a version of the script that recreates incomplete virtual environments automatically.

---

## ROS2 driver (Docker)

For ROS2 integration (RViz, ROS topics, launch files), use the Docker-based workflow described in [docs/ros2.md](docs/ros2.md).

---

## Firmware update

If you have just received a camera, update the firmware using the image files in `firmware/` and the XvisioTech upgrade tool. See the [XvisioTech documentation](https://www.xvisiotech.com/doc/) for instructions.

---

## Maintainer

Andy Park <andypark.purdue@gmail.com>
