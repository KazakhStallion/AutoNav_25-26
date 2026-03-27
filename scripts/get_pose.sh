#!/bin/bash
# Print the robot's current pose (x, y, yaw) in the map frame.
# Usage: ./get_pose.sh

OUTPUT=$(ros2 run tf2_ros tf2_echo map base_link --once 2>&1)

X=$(echo "$OUTPUT" | grep -oP 'Translation: \[\K[^,]+')
Y=$(echo "$OUTPUT" | grep -oP 'Translation: \[[^,]+, \K[^,]+')

# Extract quaternion components
QX=$(echo "$OUTPUT" | grep -oP 'Rotation: in Quaternion \[\K[^,]+')
QY=$(echo "$OUTPUT" | grep -oP 'Rotation: in Quaternion \[[^,]+, \K[^,]+')
QZ=$(echo "$OUTPUT" | grep -oP 'Rotation: in Quaternion \[[^,]+, [^,]+, \K[^,]+')
QW=$(echo "$OUTPUT" | grep -oP 'Rotation: in Quaternion \[[^,]+, [^,]+, [^,]+, \K[^]]+')

# Convert quaternion to yaw in degrees
YAW_DEG=$(python3 -c "
import math
qz, qw = $QZ, $QW
yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
print(f'{math.degrees(yaw):.1f}')
")

echo ""
echo "  Robot Pose (map frame)"
echo "  ----------------------"
echo "  x:   $X"
echo "  y:   $Y"
echo "  yaw: ${YAW_DEG}°"
echo ""
echo "  Send robot here with:"
echo "  ./send_goal.sh $X $Y $YAW_DEG"
echo ""
