#!/bin/bash

# Define your workspace directory
WORKSPACE_DIR="$(pwd)"

# Check if the workspace directory is correct
if [ ! -d "$WORKSPACE_DIR/src" ]; then
  echo "Error: '$WORKSPACE_DIR/src' directory does not exist!"
  exit 1
fi

# Build the workspace using colcon
echo "Building the workspace..."
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build


# Source the local setup file after building
source "install/local_setup.bash"

echo "Waiting for running launch file"

#Run example node
ros2 launch cpp_pubsub test_launch.launch.py

