#!/bin/bash

# Define your workspace directory
WORKSPACE_DIR="$(pwd)"

# Check if the workspace directory is correct
if [ ! -d "$WORKSPACE_DIR/src" ]; then
  echo "Error: '$WORKSPACE_DIR/src' directory does not exist!"
  exit 1
fi

# Initialize MAKE_MAP variable
MAKE_MAP=${MAKE_MAP:-false}

# Build the workspace using colcon
echo "Sourcing ROS setup..."
source /opt/ros/humble/setup.bash

echo "Installing dependencies..."
rosdep install -i --from-path src --rosdistro humble -y

echo "Building the workspace..."
colcon build --symlink-install || { echo "colcon build failed"; exit 1; }

# Source the local setup file after building
source "install/local_setup.bash"

echo "Waiting for running bringup launch file..."

ros2 launch robot_bringup robot_bringup.launch.py &

# TODO ADD launch file of localization

if [ "$MAKE_MAP" == true ]; then
  echo "Launching map creation..."
  ros2 launch navigation slam.launch.py
else
  echo "Launching navigation with map..."
  ros2 launch navigation navigation.launch.py &
  ros2 launch lidar_localization lidar_localization.launch.py
fi