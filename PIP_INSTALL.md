# Pip Installation Guide

The `xvisio` package can now be installed via `pip install`, making it easy to integrate into other projects.

## Installation Steps

### 1. Install Python Package

```bash
pip install xvisio
```

This installs:
- The Python package (`xvisio`)
- Python dependencies (numpy, spatialmath-python)
- CLI command (`xvisio-setup`)

### 2. Run System Setup (One-time, requires sudo)

After installing the Python package, you need to set up system-level dependencies:

```bash
sudo xvisio-setup
```

This command:
- Installs udev rules for USB device access
- Adds your user to the `plugdev` group (you may need to log out/in)
- Installs SuiteSparse libraries
- Installs the XVSDK driver `.deb` package

**Note**: If you were added to the `plugdev` group, log out and log back in for the change to take effect.

### 3. Verify Installation

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

## Integration into Other Projects

### Adding as a Dependency

Add `xvisio` to your project's `requirements.txt` or `pyproject.toml`:

**requirements.txt:**
```
xvisio>=0.1.0
```

**pyproject.toml:**
```toml
[project]
dependencies = [
    "xvisio>=0.1.0",
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

When you run `pip install xvisio`:

1. **Python Package**: `xvisio` module with all Python code
2. **CLI Command**: `xvisio-setup` for system setup
3. **Build Artifacts**: C++ extension module (built during installation)

The package includes:
- Python API (`xvisio.open()`, `xvisio.discover()`, etc.)
- C++ bindings (built with nanobind)
- System setup script (accessible via `xvisio-setup`)

## Limitations

**System-level setup still required**: The `pip install` command installs the Python package, but system-level setup (udev rules, driver installation) must be done separately via `sudo xvisio-setup`. This is because:

- Pip cannot install system packages (apt-get)
- Pip cannot install `.deb` files
- Pip cannot modify udev rules or user groups

However, the `xvisio-setup` command makes this process simple and automated.

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

## Development vs Production

- **Production**: Use `pip install xvisio` + `sudo xvisio-setup`
- **Development**: Use pixi (see main README.md)

The pip installation is optimized for end users who want to use the package in their projects, while pixi is better for development and contributing to the package itself.

