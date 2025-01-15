#!/bin/bash

# Enable exit on error
set -e

# Define your workspace directory
WORKSPACE_DIR="$(pwd)"

RUN_RVIZ=$1

# Check if the workspace directory is correct
if [ ! -d "$WORKSPACE_DIR/src" ]; then
  echo "Error: '$WORKSPACE_DIR/src' directory does not exist!"
  exit 1
fi

# Build the workspace using colcon
echo "Building the workspace..."
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build #--symlink-install

# Check if colcon build was successful before sourcing
if [ $? -eq 0 ]; then
  # Source the local setup file after building
  source "install/local_setup.bash"
else
  echo "Error: colcon build failed!"
  exit 1
fi

echo "Waiting for running launch file"

ros2 launch description construct.launch.py &

ros2 launch navigation slam.launch.py
