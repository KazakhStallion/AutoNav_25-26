#!/bin/bash
# Send a Nav2 goal pose from the command line.
# Usage: ./send_goal.sh [-r] <x> <y> [yaw_degrees]
#   -r            relative mode: x/y/yaw are relative to the robot's current pose
#   yaw_degrees   defaults to 0 (facing +x)

RELATIVE=false
if [ "$1" = "-r" ]; then
  RELATIVE=true
  shift
fi

if [ $# -lt 2 ]; then
  echo "Usage: $0 [-r] <x> <y> [yaw_degrees]"
  echo "  -r  goal is relative to the robot's current pose"
  exit 1
fi

X=$1
Y=$2
YAW_DEG=${3:-0}

if $RELATIVE; then
  # Get current pose from tf2
  TF_OUTPUT=$(ros2 run tf2_ros tf2_echo map base_link --once 2>&1)

  GLOBAL=$(python3 -c "
import math, re, sys

output = '''$TF_OUTPUT'''

t = re.search(r'Translation: \[([^,]+),\s*([^,]+),\s*([^\]]+)\]', output)
q = re.search(r'Quaternion \[([^,]+),\s*([^,]+),\s*([^,]+),\s*([^\]]+)\]', output)

if not t or not q:
    print('ERROR: could not parse tf2_echo output', file=sys.stderr)
    sys.exit(1)

rx, ry = float(t.group(1)), float(t.group(2))
qz, qw = float(q.group(3)), float(q.group(4))
ryaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

dx, dy, dyaw_deg = $X, $Y, $YAW_DEG
dyaw = math.radians(dyaw_deg)

goal_x = rx + dx * math.cos(ryaw) - dy * math.sin(ryaw)
goal_y = ry + dx * math.sin(ryaw) + dy * math.cos(ryaw)
goal_yaw_deg = math.degrees(ryaw + dyaw)

print(f'{goal_x} {goal_y} {goal_yaw_deg}')
")

  if [ $? -ne 0 ]; then
    echo "Failed to get current pose. Is the robot localized?"
    exit 1
  fi

  read X Y YAW_DEG <<< "$GLOBAL"
  echo "Relative goal resolved to global: x=$X, y=$Y, yaw=${YAW_DEG}°"
fi

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
