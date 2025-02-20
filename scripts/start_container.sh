#!/bin/bash

# Allow container to connect to X server
xhost +local:docker

# Find Xvisio device (for logging)
DEVICE_LINES=$(lsusb | grep -i "Xvisio\|040e:f003")
if [ -z "$DEVICE_LINES" ]; then
    echo "No Xvisio devices found. Please connect the device(s) and try again."
    exit 1
fi

RULES_FILE="/etc/udev/rules.d/50-myusb.rules"
TEMP_FILE=$(mktemp)

# Overwrite rules file with rules for both normal and DFU modes
echo "# Udev rules for Xvisio device" > "$TEMP_FILE"
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="040e", ATTRS{idProduct}=="f408", GROUP="users", MODE="0666"' >> "$TEMP_FILE"
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="040e", ATTRS{idProduct}=="f003", GROUP="users", MODE="0666"' >> "$TEMP_FILE"
echo "Added USB rules for Xvisio device in normal and DFU modes"

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