#!/bin/bash

cd ~/ros2_ws/ && \
colcon build  --packages-select \
	xv_ros2_msgs xv_sdk_ros2 --cmake-args \
	-DXVSDK_INCLUDE_DIRS="/usr/include/xvsdk" \
	-DXVSDK_LIBRARIES="/usr/lib/libxvsdk.so" && \
echo "source /home/ros/ros2_ws/install/setup.bash" >> ~/.bashrc && \
source ~/.bashrc && cd /workspaces/xvisio-cam/
