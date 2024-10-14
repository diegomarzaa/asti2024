#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/foxy/setup.bash

# Check if the workspace has been built
if [ ! -f "/root/asti2024_ws/install/setup.bash" ]; then
    echo "Compilando workspace ya que es la primera vez que entras..."
    cd /root/asti2024_ws
    colcon build --packages-select custom_interfaces
    source /root/asti2024_ws/install/setup.bash
    colcon build --packages-select dynamixel_sdk
    source /root/asti2024_ws/install/setup.bash
    colcon build --symlink-install --packages-select bringup
    source /root/asti2024_ws/install/setup.bash
    colcon build --symlink-install
    source /root/asti2024_ws/install/setup.bash
else
    echo "Workspace ya compilado, sourcing setup.bash..."
    source /root/asti2024_ws/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"