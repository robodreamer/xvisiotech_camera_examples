#!/bin/bash

cd ~/ros2_ws/ && \
colcon build  --packages-select xv_ros2_msgs xv_sdk_ros2 && \
echo "source /home/ros/ros2_ws/install/setup.bash" >> ~/.bashrc && \
source ~/.bashrc && \
cd /workspaces/xvisio-cam/
