#!/bin/bash

# Build the workspace using colcon
echo "Building the workspace..."
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install


# Source the local setup file after building
source "install/local_setup.bash"
source sllidar_ros2/scripts/create_udev_rules.sh

echo "Waiting for running bringup launch file"

ros2 launch middleware middleware.launch.py