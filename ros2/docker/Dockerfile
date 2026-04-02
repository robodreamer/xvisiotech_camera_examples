# .devcontainer/Dockerfile

# Start from ROS2 Humble base image (Ubuntu 22.04-based)
ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base-jammy

# Avoid interactive tzdata, etc.
ENV DEBIAN_FRONTEND=noninteractive

# Create a non-root user (ros) with sudo access
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y sudo \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG users,plugdev $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER root

# Install dependencies needed for the driver + common dev tools
RUN apt-get update && apt-get install -y \
    udev tree g++ cmake cmake-curses-gui pkg-config autoconf libtool libudev-dev libjpeg-dev zlib1g-dev \
    libopencv-dev rapidjson-dev libeigen3-dev libboost-thread-dev libboost-filesystem-dev \
    libboost-system-dev libboost-program-options-dev libboost-date-time-dev liboctomap-dev \
    git wget curl nano usbutils \
    libgtk-3-dev x11-apps \
    ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-rviz2 \
    libsuitesparse-dev \
    dfu-util \
    ros-${ROS_DISTRO}-plotjuggler-ros

# (Optional) If you have other tools or dependencies, add them here.

# Just to be safe, ensure /etc/udev/rules.d/ is present
RUN mkdir -p /etc/udev/rules.d

RUN echo "DEBUG: Listing everything in current build context" && ls -R .

# Copy USB udev rules (assuming you have 99-xvisio.rules in your repo)
# Check if the file exists first, then copy it.
COPY ubuntu-drivers/99-xvisio.rules /etc/udev/rules.d/99-xvisio.rules
# done, no need for udevadm control --reload-rules

# Install the XVSDK_jammy_amd64.deb driver with the specified version date:
ARG DRIVER_DATE=20250227
COPY ubuntu-drivers/XVSDK_jammy_amd64_${DRIVER_DATE}.deb /tmp/
RUN dpkg -i /tmp/XVSDK_jammy_amd64_${DRIVER_DATE}.deb || apt-get -f install -y

# Source the ROS2 setup in your shell
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

# Switch to non-root user
USER ${USERNAME}

# Create a colcon workspace directory structure
RUN mkdir -p /home/${USERNAME}/ros2_ws/src

# If you’d like to pre-populate the workspace, do it here or let the user copy
# their code at runtime:
WORKDIR /home/${USERNAME}/ros2_ws

# If 'xvsdk-ros2' is in the same folder as the Dockerfile or visible in build context:
COPY xvsdk-ros2 /home/${USERNAME}/ros2_ws/src/xvsdk-ros2

ENV XDG_RUNTIME_DIR=/tmp/xdg_runtime_ros
RUN mkdir -p /tmp/xdg_runtime_ros

# Done
