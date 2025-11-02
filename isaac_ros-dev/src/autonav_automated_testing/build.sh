#!/bin/bash
# Quick setup script for building the automated testing package

echo "Building autonav_automated_testing package..."

cd /autonav/isaac_ros-dev

# Build the package
colcon build --packages-select autonav_automated_testing

# Source the workspace
source install/setup.bash

echo "Build complete!"
echo ""
echo "To run tests:"
echo "  1. Launch RQT GUI: ros2 run rqt_gui rqt_gui"
echo "  2. Or launch directly: ros2 launch autonav_automated_testing t001_GPS_Cal.launch.py"
echo ""
echo "Log files will be saved to: /autonav/logs/"
