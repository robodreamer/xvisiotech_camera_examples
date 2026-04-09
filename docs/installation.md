# Installation

## Prerequisites

- Ubuntu 22.04 or 24.04 (x86_64)
- Python 3.10+
- XvisioTech device connected via USB (camera) or USB dongle (Seer controller)

## Linux (Ubuntu/Debian)

### One-shot installer

From a clone of this repository (recommended):

```bash
# PyPI install into ./.venv in your current directory
bash scripts/install_xvisio_linux.sh

# Use a specific Python and venv path
bash scripts/install_xvisio_linux.sh --python python3.12 --venv ./.venv

# Editable install from the repo
bash scripts/install_xvisio_linux.sh --editable

# Skip host setup if already done
bash scripts/install_xvisio_linux.sh --skip-host-setup
```

Run `bash scripts/install_xvisio_linux.sh --help` for all options.

Without cloning, you can download the script and run it from an empty project folder
(it will clone the repo to a temp dir to fetch the XVSDK driver, then install **xvisio from PyPI**):

```bash
curl -fsSL -O https://gist.githubusercontent.com/robodreamer/fb7db4b86c9e63f8737da32ee6f9e98f/raw/33e032d5e9576964131b8bd49e3fae4ad5d501d9/install_xvisio_linux.sh
bash install_xvisio_linux.sh
```

!!! note "Why a clone is needed"
    Unlike pure-Python packages, the host setup requires the **XVSDK driver `.deb`** from `drivers/`.
    When run without a clone, the installer clones the repo to a temporary directory automatically.

The manual steps below are equivalent if you prefer not to use the script.

---

### Manual steps

**1. One-time host setup** (installs udev rules and the XVSDK driver — auto-selected for your Ubuntu version):

```bash
git clone https://github.com/robodreamer/xvisiotech_camera_examples.git
cd xvisiotech_camera_examples
sudo ./scripts/setup_host.sh
```

!!! note "Group membership"
    If the script adds you to the `plugdev` group, log out and log back in once for the change to take effect.

**2. Install the Python package:**

=== "pip (recommended)"

    ```bash
    pip install xvisio

    # Optional: includes viser for 3D visualization examples
    pip install xvisio[examples]
    ```

    View the package at [pypi.org/project/xvisio/](https://pypi.org/project/xvisio/).

=== "Source checkout"

    ```bash
    pip install -e . --no-build-isolation
    ```

=== "Pixi (development)"

    [Pixi](https://pixi.sh) manages the full build toolchain, making it the easiest path for contributors.

    ```bash
    # Install pixi
    curl -fsSL https://pixi.sh/install.sh | bash

    # Build and install the Python package
    pixi run install

    # Run a demo
    pixi run demo_pose_imu
    ```

**3. Verify:**

```bash
python3 -c "import xvisio; print(xvisio.discover())"
```

With a device connected this prints something like:

```
[DeviceInfo(serial_number='SNXR501G10002222006246', model='XR-50')]
```

## Running Examples After pip Install

Examples are bundled with the pip package. Use the `xvisio-examples` CLI:

```bash
# List available examples
xvisio-examples --list

# Copy examples to a local directory
xvisio-examples --copy
cd xvisio_examples
python demo_pose_imu.py
```

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `No Xvisio devices found` | udev rules not installed | Run `sudo ./scripts/setup_host.sh` |
| `Permission denied` on USB | Not in `plugdev` group | Log out and back in after setup script |
| `ImportError: libxvsdk.so` | XVSDK not installed | Run `sudo ./scripts/setup_host.sh` |
| `Failed to start SLAM` | SLAM still initializing | Wait 2 s after `enable()`, or move device slightly |
| `No data available within 0.5s timeout` (IMU) | IMU callback not firing | Ensure `enable(imu=True)` was called first |
| `ERROR: Could not locate driver install assets` | Running `xvisio-setup` from a wheel-only install | Use `sudo ./scripts/setup_host.sh` from a source clone instead |
