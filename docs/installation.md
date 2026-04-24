# Installation

## Prerequisites

- Ubuntu 22.04 (x86_64)
- Python 3.10+
- XvisioTech device connected via USB (camera) or USB dongle (Seer controller)

## Linux (Ubuntu/Debian)

### One-shot installer

From a fresh folder, download the public installer script and run it:

```bash
curl -fsSL -O https://gist.githubusercontent.com/robodreamer/fb7db4b86c9e63f8737da32ee6f9e98f/raw/install_xvisio_linux.sh
bash install_xvisio_linux.sh

# Or install xvisio plus EmbodiK solver/example support for Panda teleop
bash install_xvisio_linux.sh --teleop
```

The standalone installer creates `./.venv`, installs `xvisio` from PyPI, and
downloads only the host setup assets required for Xvisio hardware support
(`setup_host.sh`, the udev rules, and the XVSDK `.deb`). It does not clone the
full repository by default. If you prefer the previous full-checkout behavior
for host setup assets, pass `--host-assets clone`:

```bash
bash install_xvisio_linux.sh --host-assets clone
```

From a clone of this repository:

```bash
# PyPI install into ./.venv in your current directory
bash scripts/install_xvisio_linux.sh

# Use a specific Python and venv path
bash scripts/install_xvisio_linux.sh --python python3.12 --venv ./.venv

# Also install EmbodiK solver/example support for Panda teleop
bash scripts/install_xvisio_linux.sh --teleop

# Editable install from the repo
bash scripts/install_xvisio_linux.sh --editable

# Skip host setup if already done
bash scripts/install_xvisio_linux.sh --skip-host-setup
```

Run `bash scripts/install_xvisio_linux.sh --help` for all options.

The manual steps below are equivalent if you prefer not to use the script.

---

### Manual steps

**1. One-time host setup** (installs udev rules, SuiteSparse, and the XVSDK `.deb`):

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

    # Optional shorthand: adds the EmbodiK solver and example dependencies for teleop
    # See "Running the EmbodiK Panda Teleop Example" below if EmbodiK builds from source.
    pip install "xvisio[teleop]"
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

## Running the EmbodiK Panda Teleop Example

The EmbodiK teleop example (`03_teleop_ik.py`) can reuse the same virtual
environment that contains `xvisio`. The one-shot installer can add EmbodiK's
native solver package, example dependencies, and Linux build prerequisites to
that environment. During `--teleop`, the installer uses a clean PyPI `pin`
build environment and persists the matching `cmeel.prefix/lib` runtime library
path into the venv activation script so EmbodiK can find COAL/Pinocchio shared
libraries after `source .venv/bin/activate`:

```bash
# Fresh xvisio + teleop install
bash scripts/install_xvisio_linux.sh --teleop

# Or add teleop support to an existing xvisio venv from the manual steps above
bash scripts/install_xvisio_linux.sh --skip-host-setup --teleop

source .venv/bin/activate
embodik-examples --copy
cd embodik_examples
python 03_teleop_ik.py --robot panda
```

If you are installing manually instead of using the script, mirror the EmbodiK
Linux installer setup before installing the solver package:

```bash
sudo apt-get install -y \
  build-essential cmake ninja-build pkg-config \
  libeigen3-dev liburdfdom-dev

pip install pin scikit-build-core nanobind cmake ninja
export CMAKE_PREFIX_PATH="$(python -c 'import pinocchio, pathlib; print(pathlib.Path(pinocchio.__file__).resolve().parents[4])')"
pip install --no-build-isolation "embodik[examples]"
```

If your environment can install a prebuilt EmbodiK wheel, `pip install
"xvisio[teleop]"` is the shorter equivalent for adding the EmbodiK solver and
example dependencies, but the one-shot `--teleop` path is recommended when you
have local ROS, Pinocchio, Boost, or COAL builds in your shell environment.

For Seer wireless controller input, the EmbodiK example opens the controller
through a serial device. It defaults to `/dev/ttyUSB0`; pass another
`--controller-port` only if your controller enumerates differently:

```bash
python 03_teleop_ik.py --robot panda --controller-port /dev/ttyUSB1
```

For tracking-camera input, no `/dev/ttyUSB*` controller port is needed; use the
tracking-camera-specific command or options provided by the EmbodiK teleop
example you are running.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `No Xvisio devices found` | udev rules not installed | Run `sudo ./scripts/setup_host.sh` |
| `Permission denied` on USB | Not in `plugdev` group | Log out and back in after setup script |
| `ImportError: libxvsdk.so` | XVSDK not installed | Run `sudo ./scripts/setup_host.sh` |
| `Failed to start SLAM` | SLAM still initializing | Wait 2 s after `enable()`, or move device slightly |
| `No data available within 0.5s timeout` (IMU) | IMU callback not firing | Ensure `enable(imu=True)` was called first |
| `ERROR: Could not locate driver install assets` | Running `xvisio-setup` from a wheel-only install | Use `sudo ./scripts/setup_host.sh` from a source clone instead |
