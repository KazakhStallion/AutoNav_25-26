#!/bin/bash
# Print the robot's current pose (x, y, yaw) in the map frame.
# Usage: ./get_pose.sh

OUTPUT=$(ros2 run tf2_ros tf2_echo map base_link --once 2>&1)

python3 -c "
import math, re

output = '''$OUTPUT'''

t = re.search(r'Translation: \[([^,]+),\s*([^,]+),\s*([^\]]+)\]', output)
q = re.search(r'Quaternion \[([^,]+),\s*([^,]+),\s*([^,]+),\s*([^\]]+)\]', output)

if not t or not q:
    print('  Error: could not parse tf2_echo output')
    exit(1)

x, y = float(t.group(1)), float(t.group(2))
qz, qw = float(q.group(3)), float(q.group(4))
yaw = math.degrees(math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz))

print()
print('  Robot Pose (map frame)')
print('  ----------------------')
print(f'  x:   {x:.3f}')
print(f'  y:   {y:.3f}')
print(f'  yaw: {yaw:.1f}°')
print()
print(f'  Send robot here with:')
print(f'  ./send_goal.sh {x:.3f} {y:.3f} {yaw:.1f}')
print()
"
