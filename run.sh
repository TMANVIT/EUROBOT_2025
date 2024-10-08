#!/bin/bash

# Exit on any error
set -eu

# Define your workspace directory
WORKSPACE_DIR="$(pwd)"

# Check if the workspace directory is correct
if [ ! -d "$WORKSPACE_DIR/src" ]; then
  echo "Error: '$WORKSPACE_DIR/src' directory does not exist!"
  exit 1
fi

# Build the workspace using colcon
echo "Building the workspace..."
colcon build --merge-install

# Source the local setup file after building
source "$WORKSPACE_DIR/install/local_setup.bash"

echo "Waiting for running launch file"


