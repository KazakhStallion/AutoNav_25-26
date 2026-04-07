#!/bin/bash
# Send a Nav2 goal pose from a GPS waypoint.
# Usage: ./send_GPS_waypoint.sh <latitude> <longitude> [yaw_degrees]
#
# Converts GPS coordinates to map frame using robot_localization's
# fromLL service (provided by navsat_transform_node), then sends
# a NavigateToPose action goal.
#
# Requires: navsat_transform_node running (from dual_ekf_navsat launch)
#
# Accepts decimal degrees OR DMS (degrees/minutes/seconds) format:
#   Decimal:  37.23112  -80.42459
#   DMS:      '37°13'\''49.7"N'  '80°25'\''29.2"W'
#   DMS alt:  37d13m49.7sN  80d25m29.2sW
#
# Examples:
#   ./send_GPS_waypoint.sh 37.23112 -80.42459                        # decimal
#   ./send_GPS_waypoint.sh "37°13'49.7\"N" "80°25'29.2\"W"          # DMS
#   ./send_GPS_waypoint.sh 37d13m49.7sN 80d25m29.2sW                # DMS alt
#   ./send_GPS_waypoint.sh 37.23112 -80.42459 90                    # with yaw

if [ $# -lt 2 ]; then
  echo "Usage: $0 <latitude> <longitude> [yaw_degrees]"
  echo "  latitude    Decimal degrees (37.2311) or DMS (\"37°13'49.7\\\"N\" or 37d13m49.7sN)"
  echo "  longitude   Decimal degrees (-80.4248) or DMS (\"80°25'29.2\\\"W\" or 80d25m29.2sW)"
  echo "  yaw_degrees Orientation at goal in degrees (default: face toward goal)"
  exit 1
fi

# Parse coordinates — converts DMS to decimal if needed
parse_coord() {
  python3 -c "
import re, sys
raw = '$1'

# Try parsing as DMS: 37°13'49.7\"N, 37d13m49.7sN, etc.
m = re.match(r'(-?)(\d+)[°d]\s*(\d+)[\'m]\s*([\d.]+)[\"s]?\s*([NSEWnsew])?', raw)
if m:
    sign = m.group(1)
    deg = float(m.group(2))
    minutes = float(m.group(3))
    sec = float(m.group(4))
    direction = (m.group(5) or '').upper()
    decimal = deg + minutes / 60.0 + sec / 3600.0
    if direction in ('S', 'W') or sign == '-':
        decimal = -abs(decimal)
    print(f'{decimal:.8f}')
else:
    # Assume already decimal
    print(f'{float(raw):.8f}')
"
}

LAT=$(parse_coord "$1")
LON=$(parse_coord "$2")
YAW_DEG_OVERRIDE=${3:-""}

# --- Step 1: Convert GPS to map frame via fromLL service ---
echo "Converting GPS ($LAT, $LON) to map frame..."

FROMLL_OUTPUT=$(ros2 service call /fromLL robot_localization/srv/FromLL \
  "{ll_point: {latitude: $LAT, longitude: $LON, altitude: 0.0}}" 2>&1)

if [ $? -ne 0 ]; then
  echo "ERROR: fromLL service call failed. Is navsat_transform_node running?"
  echo "  Launch it with: ros2 launch slam dual_ekf_navsat.launch.py"
  echo "Raw output: $FROMLL_OUTPUT"
  exit 1
fi

# Parse the map_point x,y from the service response
MAP_COORDS=$(python3 -c "
import re, sys

output = '''$FROMLL_OUTPUT'''

# robot_localization response format: ...Point(x=1.23, y=4.56, z=0.0)
# or YAML style: x: 1.23
xm = re.search(r'x[=:]\s*([-\d.eE+]+)', output)
ym = re.search(r'y[=:]\s*([-\d.eE+]+)', output)

if not xm or not ym:
    print('ERROR: Could not parse fromLL response', file=sys.stderr)
    print('Response was: ' + output, file=sys.stderr)
    sys.exit(1)

print(f'{xm.group(1)} {ym.group(1)}')
")

if [ $? -ne 0 ]; then
  echo "Failed to parse fromLL response."
  exit 1
fi

read X Y <<< "$MAP_COORDS"
echo "Map frame goal: x=$X, y=$Y"

# --- Step 2: Determine goal orientation ---
if [ -n "$YAW_DEG_OVERRIDE" ]; then
  YAW_DEG=$YAW_DEG_OVERRIDE
  echo "Using specified yaw: ${YAW_DEG}°"
else
  # Auto-compute yaw to face from current position toward goal
  TF_OUTPUT=$(ros2 run tf2_ros tf2_echo map base_link --once 2>&1)

  YAW_DEG=$(python3 -c "
import math, re, sys

output = '''$TF_OUTPUT'''
t = re.search(r'Translation: \[([^,]+),\s*([^,]+),\s*([^\]]+)\]', output)

if not t:
    # Can't get current pose — default to 0
    print('0')
    sys.exit(0)

rx, ry = float(t.group(1)), float(t.group(2))
goal_x, goal_y = $X, $Y

dx = goal_x - rx
dy = goal_y - ry

if abs(dx) < 0.01 and abs(dy) < 0.01:
    print('0')
else:
    print(f'{math.degrees(math.atan2(dy, dx)):.2f}')
")
  echo "Auto-computed yaw toward goal: ${YAW_DEG}°"
fi

# --- Step 3: Convert yaw to quaternion and send goal ---
QZ=$(python3 -c "import math; print(math.sin(math.radians($YAW_DEG)/2))")
QW=$(python3 -c "import math; print(math.cos(math.radians($YAW_DEG)/2))")

echo "Sending NavigateToPose: x=$X, y=$Y, yaw=${YAW_DEG}° (qz=$QZ, qw=$QW)"

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
