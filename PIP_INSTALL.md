# Installation Guide (pip install)

The `xvisio` package can be installed via `pip` from PyPI or from source checkout (for development).

## Quick Installation (pip install)

```bash
# Install from PyPI
pip install xvisio

# Install with visualization examples (includes viser)
pip install xvisio[examples]

# Run system setup (one-time, requires sudo)
sudo xvisio-setup

# Test installation
python3 -c "import xvisio; print(xvisio.discover())"
```

View the package at [pypi.org/project/xvisio/](https://pypi.org/project/xvisio/).

## Installation from Source Checkout

For development or if you need the latest unreleased code:

### 1. Clone the Repository

```bash
git clone https://github.com/xvisiotech/xvisiotech_camera_examples.git
cd xvisiotech_camera_examples
```

### 2. Run System Setup (One-time, requires sudo)

Set up system-level dependencies (udev rules, XVSDK driver):

```bash
sudo ./scripts/setup_host.sh
```

This script:
- Installs udev rules for USB device access
- Adds your user to the `plugdev` group (you may need to log out/in)
- Installs SuiteSparse libraries
- Installs the XVSDK driver `.deb` package

**Note**: If you were added to the `plugdev` group, log out and log back in for the change to take effect.

### 3. Install the Python Package

```bash
pip install -e . --no-build-isolation
```

### 4. Verify Installation

```python
import xvisio

# Discover devices
devices = xvisio.discover()
print(f"Found {len(devices)} device(s)")

# Use the device
with xvisio.open() as dev:
    dev.enable(slam=True, imu=True)
    pose = dev.pose_world_aligned()
    print(f"Position: {pose.position}")
```

## Running Examples

Examples are included in the pip package. To access them:

### Option 1: Use the `xvisio-examples` command (recommended)

**List available examples:**
```bash
xvisio-examples --list
```

**Copy examples to a local directory for editing:**
```bash
xvisio-examples --copy

# Then run examples from the copied directory
cd xvisio_examples
python demo_pose_imu.py
```

**Show examples location:**
```bash
xvisio-examples
# Then run directly: python /path/to/examples/demo_pose_imu.py
```

### Option 2: Clone the Repository

If examples aren't found in the package, you can clone the repository:

```bash
# Clone just to get examples (you already have xvisio installed)
git clone https://github.com/xvisiotech/xvisiotech_camera_examples.git
cd xvisiotech_camera_examples
python examples/demo_pose_imu.py
```

### Available Examples

- **`demo_pose_imu.py`** - Basic pose + IMU demo (no extra dependencies)
- **`demo_pose_visualization.py`** - 3D visualization (requires `viser`)
- **`demo_pose_transforms.py`** - Demonstrates different pose transform methods
- **`benchmark_pose_rate.py`** - Benchmark pose update rates

### Installing Example Dependencies

Some examples require additional dependencies. Install them with:

```bash
# Install with visualization/example dependencies
pip install xvisio[examples]

# Or install viser separately
pip install viser
```

## Integration into Other Projects

### Adding as a Dependency

**requirements.txt:**
```
xvisio>=0.2.0
```

**pyproject.toml:**
```toml
[project]
dependencies = [
    "xvisio>=0.2.0",
]
```

**With visualization examples:**
```toml
[project]
dependencies = [
    "xvisio[examples]>=0.2.0",
]
```

### Example: Using in Another Project

```python
# my_project/main.py
import xvisio

def track_device():
    with xvisio.open() as dev:
        dev.enable(slam=True, imu=True)

        for pose in dev.poses(rate_hz=10):
            # Use world-aligned pose
            world_pose = dev.pose_world_aligned()
            print(f"World position: {world_pose.position}")

            # Get IMU data
            imu = dev.imu()
            print(f"Acceleration: {imu.accel}")

if __name__ == "__main__":
    track_device()
```

## What Gets Installed

When you run `pip install xvisio` (from TestPyPI or source):

1. **Python Package**: `xvisio` module with all Python code
2. **CLI Command**: `xvisio-setup` for system setup
3. **Build Artifacts**: C++ extension module (built during installation from source distribution)

The package includes:
- Python API (`xvisio.open()`, `xvisio.discover()`, etc.)
- C++ bindings (built with nanobind)
- System setup script (accessible via `xvisio-setup`)
- Driver assets (udev rules, `.deb` file) for `xvisio-setup` command

## System Setup Required

**Important**: After installing via `pip`, you must run system setup:

```bash
sudo xvisio-setup
```

This command:
- Installs udev rules for USB device access
- Installs the XVSDK driver `.deb` package
- Installs SuiteSparse libraries (if needed)
- Adds your user to the `plugdev` group (you may need to log out/in)

**Why separate?** Pip cannot install system packages, `.deb` files, or modify udev rules. The `xvisio-setup` command handles all system-level setup automatically.

**Note**: The `xvisio-setup` command locates driver assets from the installed package. Driver files (`scripts/setup_host.sh`, `ubuntu-drivers/`) are included in the source distribution. If `xvisio-setup` cannot find them, ensure you installed from a source distribution (not a wheel), or manually run `sudo ./scripts/setup_host.sh` from a source checkout.

## Troubleshooting

### `xvisio-setup` command not found

If the command is not available after installation:
- Make sure you installed with `pip install xvisio` (not `pip install -e .`)
- Check that the package was installed correctly: `pip show xvisio`
- Try running: `python -m xvisio.cli setup_host`

### System setup script not found

If `xvisio-setup` can't find the setup script:
- The script should be included in the package
- Check installation: `pip show -f xvisio | grep setup`
- Fallback: Use the manual setup script from the repository

### Build fails during pip install

If the build fails:
- Ensure you have build tools: `apt-get install build-essential cmake`
- Ensure you have XVSDK installed: `ls /usr/lib/libxvsdk.so`
- Try installing build dependencies first: `pip install scikit-build-core nanobind`

## Installation Methods Summary

| Method | Use Case | Command |
|--------|----------|---------|
| **PyPI** | Production use | `pip install xvisio` |
| **PyPI + Examples** | With visualization dependencies | `pip install xvisio[examples]` |
| **Source Checkout** | Development, contributing | `pip install -e . --no-build-isolation` |

**All methods require**: `sudo xvisio-setup` after installation for system-level setup.

For development with pixi, see the main [README.md](README.md).

