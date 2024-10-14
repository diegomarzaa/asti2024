#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/foxy/setup.bash

# Check if the workspace has been built
if [ ! -f "/root/retos_ws/install/setup.bash" ]; then
    echo "Building workspace for the first time..."
    cd /root/retos_ws
    colcon build --packages-select custom_interfaces
    source install/setup.bash
    colcon build --packages-select dynamixel_sdk
    source install/setup.bash
    colcon build --symlink-install --packages-select bringup
    source install/setup.bash
    colcon build --symlink-install
else
    echo "Workspace already built, sourcing setup.bash..."
    source /root/retos_ws/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"