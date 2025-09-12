#!/bin/bash

# NAR Tractor Control System - Workspace Setup Script
# This script sets up the development environment for the NAR project

set -e  # Exit on error

echo "=== NAR Workspace Setup ==="

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

echo "Workspace directory: $WORKSPACE_DIR"

# Check if we're in a ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 not sourced. Please run:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo "ROS 2 $ROS_DISTRO detected"

# Install dependencies if rosdep is available
if command -v rosdep &> /dev/null; then
    echo "Installing ROS 2 dependencies..."
    cd "$WORKSPACE_DIR"
    rosdep update || true
    rosdep install --from-paths src --ignore-src -r -y
else
    echo "Warning: rosdep not found. You may need to install dependencies manually."
fi

# Build the workspace
echo "Building the workspace..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "=== Build Successful! ==="
    echo ""
    echo "To use this workspace, run:"
    echo "  cd $WORKSPACE_DIR"
    echo "  source install/setup.bash"
    echo ""
    echo "Then you can run the nodes:"
    echo "  ros2 run vehicle_dynamics vehicle_dynamics_node"
    echo "  ros2 run pid_controller pid_controller_node"  
    echo "  ros2 run fake_lidar fake_lidar_node"
    echo "  ros2 run setpoint_publisher setpoint_publisher_node"
    echo ""
    echo "For more information, see README.md"
else
    echo ""
    echo "=== Build Failed ==="
    echo "Please check the error messages above and fix any issues."
    exit 1
fi