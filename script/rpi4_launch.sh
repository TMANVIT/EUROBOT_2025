#!/bin/bash

# Exit on error and unset variables
set -eu

# Source ROS 2 environment
if ! source /opt/ros/humble/setup.bash; then
    echo "Failed to source ROS 2 environment. Is ROS 2 installed?"
    exit 1
fi

# Build the SLLidar ROS 2 driver
echo "Building SLLidar ROS 2 driver..."
if ! cd drivers/sllidar_ros2; then
    echo "Failed to change directory to drivers/sllidar_ros2. Does the directory exist?"
    exit 1
fi

if ! colcon build --symlink-install; then
    echo "Failed to build SLLidar ROS 2 driver."
    exit 1
fi

# Set permissions for the USB device
USB_DEVICE="/dev/ttyUSB0"
if [ ! -e "$USB_DEVICE" ]; then
    echo "Device $USB_DEVICE not found. Is the LiDAR connected?"
    exit 1
fi

echo "Setting permissions for $USB_DEVICE..."
if ! sudo chmod 666 "$USB_DEVICE"; then
    echo "Failed to set permissions for $USB_DEVICE."
    exit 1
fi

# Launch the SLLidar ROS 2 node
echo "Launching SLLidar ROS 2 node..."
ros2 launch sllidar_ros2 sllidar_s3_launch.py frame_id:=lidar &
SLLIDAR_PID=$!

# Ensure the launch was successful
if ! ps -p $SLLIDAR_PID > /dev/null; then
    echo "Failed to launch SLLidar ROS 2 node."
    exit 1
fi

# Run the Micro-ROS Agent
MICRO_ROS_DEVICE="/dev/ttyACM0"
if [ ! -e "$MICRO_ROS_DEVICE" ]; then
    echo "Device $MICRO_ROS_DEVICE not found. Is the microcontroller connected?"
    exit 1
fi

echo "Starting Micro-ROS Agent..."
ros2 run micro_ros_agent micro_ros_agent serial --dev "$MICRO_ROS_DEVICE" &
MICRO_ROS_PID=$!

# Ensure the Micro-ROS Agent was started successfully
if ! ps -p $MICRO_ROS_PID > /dev/null; then
    echo "Failed to start Micro-ROS Agent."
    exit 1
fi

# Cleanup function to stop background processes on script exit
cleanup() {
    echo "Stopping background processes..."
    kill -SIGTERM $SLLIDAR_PID $MICRO_ROS_PID
    wait $SLLIDAR_PID $MICRO_ROS_PID
    echo "Script exited."
}

# Trap script exit to run cleanup
trap cleanup EXIT

# Keep the script running to keep background processes alive
echo "Script is running. Press Ctrl+C to exit."
while true; do
    sleep 1
done