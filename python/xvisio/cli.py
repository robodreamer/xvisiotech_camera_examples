#!/usr/bin/env python3
"""Command-line interface for xvisio package setup."""

import os
import sys
import subprocess
import shutil
from pathlib import Path


def get_package_data_dir():
    """Get the directory where package data files are installed."""
    # Try to find the package installation directory
    try:
        import xvisio
        package_dir = Path(xvisio.__file__).parent.parent
    except ImportError:
        # Fallback: assume we're in development mode
        package_dir = Path(__file__).parent.parent.parent

    # Look for ubuntu-drivers directory
    ubuntu_drivers = package_dir / "ubuntu-drivers"
    if ubuntu_drivers.exists():
        return ubuntu_drivers

    # Try alternative locations (for pip-installed packages)
    for possible_dir in [
        package_dir / "xvisio" / "ubuntu-drivers",
        package_dir / "share" / "xvisio" / "ubuntu-drivers",
        Path("/usr/share/xvisio/ubuntu-drivers"),
    ]:
        if possible_dir.exists():
            return possible_dir

    return None


def get_setup_script():
    """Get the path to the setup_host.sh script."""
    try:
        import xvisio
        package_dir = Path(xvisio.__file__).parent.parent
    except ImportError:
        package_dir = Path(__file__).parent.parent.parent

    # Look for scripts directory
    scripts_dir = package_dir / "scripts"
    if scripts_dir.exists():
        script = scripts_dir / "setup_host.sh"
        if script.exists():
            return script

    # Try alternative locations
    for possible_dir in [
        package_dir / "xvisio" / "scripts",
        package_dir / "share" / "xvisio" / "scripts",
        Path("/usr/share/xvisio/scripts"),
    ]:
        script = possible_dir / "setup_host.sh"
        if script.exists():
            return script

    return None


def setup_host():
    """Run the host setup script to install system dependencies."""
    print("=== Xvisio Host Setup ===")
    print()

    # Check if running as root
    if os.geteuid() != 0:
        print("ERROR: This command requires sudo privileges.")
        print("Please run: sudo xvisio-setup")
        print()
        print("Alternatively, run the setup script directly:")
        script = get_setup_script()
        if script:
            print(f"  sudo {script}")
        sys.exit(1)

    # Find the setup script
    script = get_setup_script()
    if not script or not script.exists():
        print("ERROR: Could not find setup_host.sh script.")
        print("This may indicate an incomplete installation.")
        print()
        print("Please ensure the package was installed correctly:")
        print("  pip install xvisio")
        print()
        print("Or run the setup script manually if you have the source:")
        print("  sudo ./scripts/setup_host.sh")
        sys.exit(1)

    # Check if ubuntu-drivers directory exists
    ubuntu_drivers = get_package_data_dir()
    if not ubuntu_drivers or not ubuntu_drivers.exists():
        print("ERROR: Could not find ubuntu-drivers directory.")
        print("This may indicate an incomplete installation.")
        sys.exit(1)

    # Update the script to use the correct paths
    print(f"Found setup script: {script}")
    print(f"Found drivers directory: {ubuntu_drivers}")
    print()

    # Run the setup script
    print("Running host setup...")
    try:
        # Set environment variables for the script
        env = os.environ.copy()
        env["XVISIO_DRIVERS_DIR"] = str(ubuntu_drivers)

        # Run the script
        result = subprocess.run(
            ["bash", str(script)],
            env=env,
            check=True
        )

        print()
        print("=== Setup Complete ===")
        print()
        print("Next steps:")
        print("1. If you were added to the plugdev group, log out and log back in")
        print("2. Connect your XR-50 device via USB")
        print("3. Test the installation:")
        print("   python -c 'import xvisio; print(xvisio.discover())'")

    except subprocess.CalledProcessError as e:
        print(f"ERROR: Setup script failed with exit code {e.returncode}")
        sys.exit(1)
    except FileNotFoundError:
        print("ERROR: Could not find bash. This is unusual.")
        sys.exit(1)


if __name__ == "__main__":
    setup_host()

