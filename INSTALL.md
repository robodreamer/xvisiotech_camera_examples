# Installation Guide

## Quick Installation (pip)

The `xvisio` package can be installed via pip:

```bash
pip install xvisio
```

However, **system-level setup is still required** for device access. After installing the Python package, run:

```bash
sudo xvisio-setup
```

This will:
- Install udev rules for USB device access
- Add your user to the `plugdev` group (you may need to log out/in)
- Install SuiteSparse libraries (required for stub generation)
- Install the XVSDK driver `.deb` package

## Development Installation (pixi)

For development, we recommend using pixi:

```bash
# Install pixi
curl -fsSL https://pixi.sh/install.sh | bash

# Clone repository
git clone <repository-url>
cd xvisiotech_camera_examples

# Run host setup (one-time, requires sudo)
sudo ./scripts/setup_host.sh

# Install Python package
pixi run install

# Run demo
pixi run demo_pose_imu
```

## Manual Installation

If you prefer manual installation:

1. **Install system dependencies:**
   ```bash
   sudo apt-get update
   sudo apt-get install -y libsuitesparse-dev
   ```

2. **Install XVSDK driver:**
   ```bash
   sudo dpkg -i ubuntu-drivers/XVSDK_jammy_amd64_20250227.deb
   sudo apt-get -f install -y  # Fix any dependency issues
   ```

3. **Install udev rules:**
   ```bash
   sudo cp ubuntu-drivers/99-xvisio.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. **Add user to plugdev group:**
   ```bash
   sudo usermod -aG plugdev $USER
   # Log out and log back in
   ```

5. **Install Python package:**
   ```bash
   pip install -e . --no-build-isolation
   ```

## Verification

After installation, verify everything works:

```python
import xvisio

# Discover devices
devices = xvisio.discover()
print(f"Found {len(devices)} device(s)")

# Open device and test
with xvisio.open() as dev:
    dev.enable(slam=True, imu=True)
    pose = dev.pose()
    print(f"Pose: {pose.position}")
```

## Troubleshooting

- **`ImportError: Failed to import xvisio native module`**: Run `pip install -e . --no-build-isolation` to rebuild
- **No devices found**: Make sure you've run `sudo xvisio-setup` and logged out/in if you were added to `plugdev`
- **Permission denied**: Check that udev rules are installed and you're in the `plugdev` group

