#!/bin/bash
# Host setup script for Xvisio XR-50 device
# This script requires sudo privileges to install udev rules and the XVSDK driver

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Allow override via environment variable (for pip-installed packages)
if [ -n "${XVISIO_DRIVERS_DIR}" ]; then
    DRIVERS_DIR="${XVISIO_DRIVERS_DIR}"
else
    DRIVERS_DIR="${REPO_ROOT}/ubuntu-drivers"
fi

UDEV_RULES_SRC="${DRIVERS_DIR}/99-xvisio.rules"
UDEV_RULES_DEST="/etc/udev/rules.d/99-xvisio.rules"
DEB_FILE="${DRIVERS_DIR}/XVSDK_jammy_amd64_20250227.deb"

echo "=== Xvisio Host Setup ==="
echo ""

# Check if running as root or with sudo
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo privileges."
    echo "Please run: sudo $0"
    exit 1
fi

# Step 1: Install udev rules
echo "[1/4] Installing udev rules..."
if [ ! -f "${UDEV_RULES_SRC}" ]; then
    echo "ERROR: udev rules file not found at ${UDEV_RULES_SRC}"
    exit 1
fi

cp "${UDEV_RULES_SRC}" "${UDEV_RULES_DEST}"
chmod 644 "${UDEV_RULES_DEST}"
udevadm control --reload-rules
udevadm trigger
echo "✓ Udev rules installed and reloaded"

# Step 2: Check/add user to plugdev group
echo ""
echo "[2/4] Checking user group membership..."
if [ -n "${SUDO_USER}" ]; then
    USERNAME="${SUDO_USER}"
else
    USERNAME="${USER}"
fi

if groups "${USERNAME}" | grep -q "\bplugdev\b"; then
    echo "✓ User ${USERNAME} is already in plugdev group"
else
    echo "Adding user ${USERNAME} to plugdev group..."
    usermod -aG plugdev "${USERNAME}"
    echo "✓ User ${USERNAME} added to plugdev group"
    echo "  NOTE: You may need to log out and log back in for group changes to take effect"
fi

# Step 3: Install SuiteSparse (required for stub generation and XVSDK dependencies)
echo ""
echo "[3/4] Installing SuiteSparse libraries..."
if dpkg -l | grep -q "libsuitesparse-dev"; then
    echo "SuiteSparse already installed"
else
    apt-get install -y libsuitesparse-dev
    echo "✓ SuiteSparse libraries installed"
fi

# Step 4: Install XVSDK driver
echo ""
echo "[4/4] Installing XVSDK driver..."
if [ ! -f "${DEB_FILE}" ]; then
    echo "ERROR: XVSDK .deb file not found at ${DEB_FILE}"
    exit 1
fi

# Check if already installed
if dpkg -l | grep -q "xvsdk"; then
    echo "XVSDK appears to be already installed. Reinstalling..."
    dpkg -i "${DEB_FILE}" || apt-get -f install -y
else
    dpkg -i "${DEB_FILE}" || apt-get -f install -y
fi

# Verify installation
if [ -f "/usr/lib/libxvsdk.so" ] && [ -d "/usr/include/xvsdk" ]; then
    echo "✓ XVSDK driver installed successfully"
    echo "  Library: /usr/lib/libxvsdk.so"
    echo "  Headers: /usr/include/xvsdk"
else
    echo "WARNING: XVSDK installation may have failed. Expected files not found."
    exit 1
fi

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Installed packages:"
echo "  - udev rules for device access"
echo "  - SuiteSparse libraries (for stub generation)"
echo "  - XVSDK driver"
echo ""
echo "Next steps:"
echo "1. If you were added to the plugdev group, log out and log back in"
echo "2. Connect your XR-50 device via USB"
echo "3. Run 'pixi run install' to build the Python package"
echo "4. Run 'pixi run demo_pose_imu' to test the device"

