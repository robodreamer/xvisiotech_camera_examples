#!/bin/bash

# Allow container to connect to X server
xhost +local:docker

# Find Xvisio device (for logging)
echo "Checking for Xvisio devices..."
DEVICE_LINES=$(lsusb | grep -i "Xvisio\|040e:f003")
if [ -z "$DEVICE_LINES" ]; then
    echo "No XR50/DS80 tracking camera found via lsusb."
else
    echo "Found XR50/DS80 tracking camera:"
    echo "$DEVICE_LINES"
fi

# Check for ttyUSB devices
echo "Checking for ttyUSB devices..."
USB_DEVICES=$(ls /dev/ttyUSB* 2>/dev/null)
if [ -z "$USB_DEVICES" ]; then
    echo "No Seer Controller device found."
else
    echo "Found Seer Controller device:"
    echo "$USB_DEVICES"

    # Change permissions for ttyUSB devices to 777
    echo "Changing permissions for ttyUSB devices to 777..."
    for device in $USB_DEVICES; do
        sudo chmod 777 "$device"
        echo "Changed permissions for $device to 777"
    done

    if [ -n "$DEVICE_LINES" ]; then
        echo "Note: a Seer Controller device is connected in addition to your XR50/DS80 tracking camera."
    fi
fi

# Final device status check
if [ -z "$DEVICE_LINES" ] && [ -z "$USB_DEVICES" ]; then
    echo "No XR50/DS80 tracking camera or Seer Controller device found. Please connect the device(s) and try again."
    echo "Continuing anyway..."
elif [ -z "$DEVICE_LINES" ]; then
    echo "Found Seer Controller device but no XR50/DS80 tracking camera. Please connect the tracking camera."
    echo "Continuing..."
elif [ -z "$USB_DEVICES" ]; then
    echo "Found XR50/DS80 tracking camera but no Seer Controller device. Controller functionality will not be available."
    echo "Continuing..."
else
    echo "XR50/DS80 tracking camera and Seer Controller device detected and configured successfully."
fi

RULES_FILE="/etc/udev/rules.d/50-myusb.rules"
TEMP_FILE=$(mktemp)

# Overwrite rules file with rules for both normal and DFU modes
echo "# Udev rules for Xvisio device" > "$TEMP_FILE"
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="040e", ATTRS{idProduct}=="f408", GROUP="users", MODE="0666"' >> "$TEMP_FILE"
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="040e", ATTRS{idProduct}=="f003", GROUP="users", MODE="0666"' >> "$TEMP_FILE"
# Add rule for ttyUSB devices
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0777"' >> "$TEMP_FILE"
echo "Added USB rules for XR50/DS80 tracking camera in normal and DFU modes"
echo "Added rule for Seer Controller device with permissions 777"

# Update rules file
sudo mv "$TEMP_FILE" "$RULES_FILE"
sudo chmod 644 "$RULES_FILE"
sudo udevadm control --reload
sudo udevadm trigger
echo "Udev rules overwritten and reloaded."
sleep 2  # Wait for device to settle

# Build the Docker image
docker build \
    --file .devcontainer/Dockerfile \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    -t xvisio-cam-dev .

# Run the container with full /dev access
docker run --privileged -it --rm \
    --volume=/dev:/dev \
    --mount type=bind,source=$(pwd),target=/workspaces/xvisio-cam -w /workspaces/xvisio-cam \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    xvisio-cam-dev \
    /bin/bash