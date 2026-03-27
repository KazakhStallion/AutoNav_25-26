#!/bin/bash
# Send a Nav2 goal pose from the command line.
# Usage: ./send_goal.sh <x> <y> [yaw_degrees]
#   yaw_degrees defaults to 0 (facing +x)

if [ $# -lt 2 ]; then
  echo "Usage: $0 <x> <y> [yaw_degrees]"
  exit 1
fi

X=$1
Y=$2
YAW_DEG=${3:-0}

# Convert yaw (degrees) to quaternion z,w
YAW_RAD=$(python3 -c "import math; print(math.radians($YAW_DEG))")
QZ=$(python3 -c "import math; print(math.sin(math.radians($YAW_DEG)/2))")
QW=$(python3 -c "import math; print(math.cos(math.radians($YAW_DEG)/2))")

echo "Sending goal: x=$X, y=$Y, yaw=${YAW_DEG}° (qz=$QZ, qw=$QW)"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"pose:
  header:
    frame_id: 'map'
  pose:
    position:
      x: $X
      y: $Y
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: $QZ
      w: $QW"
