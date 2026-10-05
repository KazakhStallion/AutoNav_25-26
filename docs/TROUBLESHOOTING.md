# TROUBLESHOOTING

> 💡 **AI search tip:** This file is dense and grep-friendly by design — it's also a great paste-into-AI document. Ask your AI assistant along with the symptom you're seeing, and it can fuzzy-match against the entries below far better than Cmd-F. Every section ends with a `**Keywords**:` line listing symptoms, file paths, parameter names, error strings, and hardware terms — those are deliberate fuzzy-match anchors. AI assistants should give those high weight when ranking matches.

This doc has two halves:

1. **[Quick triage](#quick-triage)** — evergreen first-look reference. The symptoms a new member will hit first, with the first thing to check. Stays short and focused.
2. **[Full fix log](#full-fix-log)** — every bug fix mined from git history, sorted oldest → newest. Big, comprehensive. When the quick triage doesn't match, search this log by symptom or commit hash.

A flat **[Keyword index](#keyword-index)** at the very bottom maps topical keywords to the entries that mention them.

---

# Quick triage

## "I just started the GUI and a button is stuck yellow forever"

| First check | Why |
|---|---|
| Click the button to open its terminal viewer | The script's stdout is right there. Look for `[GUI_READY] <Label>` — most scripts emit it after a 0.5 s pacing delay; SLAM intentionally waits longer. |
| If you see the script text but no sentinel | The script crashed before printing it. Look up at the actual error in the terminal. |
| If you see no terminal output at all | **Connect to Container** likely wasn't clicked first. Without it, `docker exec` can't fire — see [2026-04-22 DDS fix](#2026-04-22--dds-udp-discovery-forced) and the GUI section in `PACKAGES.md`. |

**Keywords**: gui, hud, button, stuck-yellow, dot, status, queue, sentinel, gui-ready, run-script, terminal-viewer, connect-to-container, docker-exec, 5s-pacing

## "No `/odom`"

| First check | Why |
|---|---|
| `ls /dev/serial/by-id/ \| grep -i roboteq` | Hard-coded `/dev/ttyACM0` paths break across reboots — we now use the stable symlink. See [2025-12-03 USB enumeration fix](#2025-12-03--usb-serial-device-enumeration-unstable). |
| `ros2 topic hz /encoders` | If `/encoders` isn't publishing, the control node never read encoders → no `/odom`. |
| `ros2 topic info /encoders \| grep "Publisher count"` | Should be **1**. If 2, two control nodes are running — see [2025-11-16 duplicate publisher guard](#2025-11-16--duplicate-controlnode-instances-corrupt-encoder-publisher). |

**Keywords**: odom, /odom, encoder, /encoders, control, roboteq, motor-controller, /dev/ttyACM0, /dev/serial/by-id, usb-roboteq, publisher, duplicate, control-node, wheel-odometry, missing-topic

## "No `/gps_fix`"

| First check | Why |
|---|---|
| `ls /dev/serial/by-id/ \| grep -i cypress` | u-blox ZED-F9P is on a Cypress USB-serial chip; we use the by-id path now, not `/dev/ttyUSB0`. See [2025-12-03 GPS path fix](#2025-12-03--gps-serial-port-path-volatility). |
| Baud rate is **38400**, not 9600 | The 2026 u-blox migration changed format from NovAtel to u-blox NMEA. See [2026-04-17 NovAtel to u-blox parser](#2026-04-17--novatel-to-u-blox-parser-mismatch). |

**Keywords**: gps, /gps_fix, ublox, u-blox, zed-f9p, cypress, novatel, nmea, gga, baud, 38400, /dev/ttyUSB0, /dev/serial/by-id, usb-cypress, no-fix, navsatfix, gps_handler, missing-topic

## "No `/scan_fullframe`"

| First check | Why |
|---|---|
| `ip addr show eno1` shows `192.168.0.2` | Both the container entrypoint AND `run-lidar.sh` configure this. If missing, toggle the Lidar button off/on — see [2026-03-26 network automation](#2026-03-26--eno1-network-configuration-automation). |
| Last-resort: `sudo ip addr flush dev eno1 && sudo ip addr add 192.168.0.2/24 dev eno1 && sudo ip link set eno1 up` | Manual override if the script can't get sudo. |
| LiDAR power and Ethernet cable | The driver waits indefinitely for UDP packets; there's no error. |

**Keywords**: lidar, sick, /scan_fullframe, scan, eno1, network, ip-addr, 192.168.0.2, 192.168.0.1, udp, multiscan, sick_scan_xd, run-lidar.sh, ethernet, configure-lidar.sh, missing-topic, no-data, laser-scan, udp-receiver

## "RViz on my laptop shows nothing"

| First check | Why |
|---|---|
| `echo $ROS_DOMAIN_ID` matches the container | Default is `0`. Both ends must agree. |
| `echo $RMW_IMPLEMENTATION` is `rmw_fastrtps_cpp` | We force FastDDS UDP. See [2026-04-22 DDS fix](#2026-04-22--dds-udp-discovery-forced). |
| `FASTRTPS_DEFAULT_PROFILES_FILE` points at `env/docker/fastdds_udp.xml` | Forces UDP transport — shared-memory fails across machines. |
| `ROS_LOCALHOST_ONLY=0` | Default is 0 but worth confirming. |

**Keywords**: rviz, dds, fastdds, fastrtps, rmw, ros_domain_id, rmw_implementation, fastdds_udp.xml, ros_localhost_only, remote-rviz, no-topics, discovery, shared-memory, udp-transport, rmw_fastrtps_cpp, network, laptop

## "colcon build fails: `'object_tracking_parameters' has no member`"

The ZED wrapper submodule has drifted past v5.2.0 onto SDK 5.2 ABI, but the Jetson runs SDK 5.1.2. Fix:

```bash
cd isaac_ros-dev/src/zed-ros2-wrapper
git checkout v5.2.0
cd ../../..
git submodule update --init isaac_ros-dev/src/zed-ros2-wrapper   # never --remote
```

See [2026-05-04 wrapper pin saga](#2026-05-04--zed-wrapper-pin-saga-3-commits) and [`docs/SENSORS.md` (ZED section)](./SENSORS.md#zed-2i-camera).

**Keywords**: zed, zed-ros2-wrapper, submodule, pin, sdk, sdk-5.1, sdk-5.2, object_tracking_parameters, customobjectdetectionproperties, colcon, build-error, v5.2.0, 506e047, git-submodule, --remote, .gitmodules, abi-mismatch, stereolabs

## "Costmap looks tilted, inverted, or off-center in RViz"

| First check | Why |
|---|---|
| Line detector has `target_frame: "map"` in `line_detector.yaml` | Without this, points stay in the camera frame and the local costmap tilts. See [2026-03-20 local costmap tilt](#2026-03-20--local-costmap-tilt). |
| ZED launch args include `publish_tf:=false` and `publish_map_tf:=false` | If the ZED publishes its own TFs, they conflict with the URDF + slam_toolbox. |
| URDF roll/pitch/yaw on lidar/camera/GPS joints | We had to rotate frames in May 2026 — see [2026-05-06 rotation fixes](#2026-05-06--sensor-frame-rotations-2-commits). |

**Keywords**: costmap, tilt, rviz, frame, tf, target_frame, line_detector.yaml, publish_tf, publish_map_tf, urdf, rpy, roll-pitch-yaw, base_link, map, lidar_footprint, zed_camera_link, ghost-trace, line-layer

## "Robot drifts right while driving straight"

The left encoder over-samples physically. There's a calibration factor (~1.016) in the wheel-odom code. See [2026-04-29 left encoder over-counting](#2026-04-29--left-encoder-over-counts-causing-rightward-drift).

**Keywords**: drift, right-drift, encoder, left-encoder, oversample, calibration, scale-factor, 1.016, 1.016335, wheel-odom, wheel_odom_pub.cpp, left_encoder_scale_, straight-line

## "Stale line obstacles linger in costmap, blocking the path"

For the active lidar-line profile, this is currently intentional during tape testing: `local_costmap.lidar_line_layer.observation_persistence_ms: -1` and `global_costmap.lidar_line_memory_mirror_layer.allow_decrease: false` keep detected tape until an operator clears costmaps. This prevents the robot from pausing near tape, forgetting it, and driving over it.

If stale cells are from a previous run, clear both local/global costmaps or publish the mirror clear topic used by the test runner:

```bash
ros2 topic pub --once /local_mirror_layer/clear std_msgs/msg/Empty "{}"
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

Camera-line staleness is a separate legacy path: `line_hold_timeout_ms` in `line_detector.yaml` affects camera detections, but the camera `line_layer` is disabled in the active lidar-line Nav2 profile.

**Keywords**: line-layer, lidar-line, costmap, stale, observation_persistence_ms, allow_decrease, local_mirror_layer, clear-costmap, ghost-obstacle, blocked-path, nav2, planner, line-detection, tape, /lidar_line_points, /lidar_line_costmap, autonav_detection

## "Nav2 fails to start: MPPI controller plugin not found"

The active branch uses `nav2_mppi_controller::MPPIController`. The runtime image must include `ros-humble-nav2-mppi-controller`.

Check inside the container:

```bash
docker exec koopa-kingdom bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix nav2_mppi_controller'
```

If that fails, rebuild only the dev image:

```bash
cd ~/AutoNav_25-26
./env/docker/build_koopa-dev.sh
```

Do not rebuild the slow base image just for this package; the dependency belongs in `env/docker/dockerfiles/Dockerfile`.

**Keywords**: nav2, mppi, nav2_mppi_controller, plugin-not-found, controller-server, FollowPath, docker, dev-image, build_koopa-dev, ros-humble-nav2-mppi-controller

## "E-stop doesn't kill the motors"

`/dev/ttyTHS1` must be mounted into the container — older runs used the wrong device. See [2025-05-28 e-stop serial init](#2025-05-28--e-stop-serial-initialization-missing-device-mount).

**Keywords**: estop, e-stop, kill-switch, /dev/ttyTHS1, ttyTHS1, button-b, b-button, serial, uart, motors, control.cpp, estop_port, mount, --device, container, kill-motors

## "Behavior tree fires in manual mode"

Verify `/autonomous_mode` is being published by the control node. The BT now gates on it. See [2026-04-29 BT gating](#2026-04-29--behavior-tree-triggering-when-not-autonomous).

**Keywords**: behavior-tree, bt, /autonomous_mode, autonomous_mode, manual-mode, gradient_escape, goal_bender, recovery, nav2, bt_nav.xml, control-node, mode-switch, custom_behavior_tree_plugins

## "Power PCB silent — no `/electrical/*` readings"

**Current state (as of 2026-05-25):** the Power PCB is physically disconnected — I²C is broken on the Jetson and a recent fix attempt left the Jetson unbootable. Until I²C is restored and the PCB is reattached, no `/electrical/voltage`, `/electrical/current`, `/electrical/power`, or `/electrical/soc` topic will publish. The GUI's Power PCB launch button will start `electrical_publisher_node`, but the node will fail to initialize the INA226 over I²C and emit a retry log instead of readings.

Triage:
1. Check `/dev/i2c-1` exists (`ls /dev/i2c-*` inside the container). If missing, the bus is down at the kernel/device-tree level — see the [2026-04-17 Jetson incident](#2026-04-17--jetson-orin-nano-bricked-by-manual-device-tree-overlay-incident) and the brick-recovery row above.
2. If `/dev/i2c-1` exists but readings still silent, check the node's terminal in the GUI — INA226 init will retry on a timer and log `[INA226] init failed, retrying...`.
3. Don't add new code dependencies on `/electrical/*` until the PCB is back. Treat power readings as currently unavailable.

**Keywords**: power-pcb, electrical, ina226, i2c, /dev/i2c-1, readings-missing, silent, autonav_electrical_publisher, /electrical/voltage, /electrical/current, /electrical/power, /electrical/soc, jetson-i2c, broken, disconnected, 2026-05

## "`docker exec` says user not found right after start"

Race between container init and shell attach. The script now waits for `getent passwd` to return — see [2026-04-06 container user race](#2026-04-06--container-user-initialization-race).

**Keywords**: docker, container, user-not-found, koopa-kingdom, getent, attach_shell, run-container.sh, race, init, wait_for_container_user, admin, entrypoint, docker-exec

## "Jetson won't boot — gets hot, fan stops after ~10 s, no SSH, no USB enumerate"

This is the **brick** failure mode. The Jetson is not actually dead, but it's stuck in an incomplete boot sequence and needs forced recovery. **Pull power immediately if it's hot** — letting it stay hot is how you get real damage on top of a software brick.

| Step | What to do |
|---|---|
| 1 | **Power off.** Pull the barrel jack — don't wait. |
| 2 | **Force recovery mode** by jumpering pins **9 and 10** on the button header (FC REC + GND). NVIDIA's official guide: <https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/howto.html> |
| 3 | **Power on with the jumper in place.** USB enumeration on the host should return. |
| 4 | **Reflash** with the Linux laptop's `sdkmanager` (JetPack 36.4.0), or `flash.sh` from the SDK. |

The almost-certain cause is a bad **device tree overlay** — see the [full incident write-up on SharePoint](https://virginiatech.sharepoint.com/:w:/r/sites/IDC2024-2025/AutoNav/Shared%20Documents/AutoNav%202025%20-%202026/Software/Important%20References/April%2017%202026%20Jetson%20incident%20report.docx?d=w89d25418776c4543ab058dc7d734e0f5&csf=1&web=1&e=3tfO3t) and the log entry [2026-04-17 Jetson Orin Nano bricked by manual device tree overlay](#2026-04-17--jetson-orin-nano-bricked-by-manual-device-tree-overlay-incident). **Never edit `/boot/extlinux/extlinux.conf` or hand-write `.dtbo` files unless you have a recovery plan.** Use `jetson-io.py` for anything header-pin related.

**Keywords**: jetson, brick, bricked, no-boot, no-ssh, no-usb, hot, fan-stops, recovery-mode, force-recovery, fc-rec, button-header, pin-9, pin-10, sdkmanager, jetpack, flash.sh, dtbo, device-tree, overlay, extlinux.conf, jetson-io.py, dp_aux_ch3_i2c, kernel-panic, reflash, orin-nano

## When the quick triage doesn't help

1. Open the device's terminal in the GUI — every script's stdout/stderr is captured live.
2. `ros2 topic list | grep <thing>` — confirm the topic is named what you think.
3. `ros2 topic hz <topic>` — confirm it's actually publishing.
4. `ros2 run tf2_tools view_frames` — generates `frames<numbers>.pdf`; check the chain.
5. Search the log below by symptom (`grep -i 'symptom keyword' docs/TROUBLESHOOTING.md`) or commit hash.
6. `git log --oneline --all --grep='<keyword>'` — your bug may have happened before. The log below is curated; the full git history is broader.

---

# Full fix log

Every bug fix mined from git history (15 parallel research agents), sorted by date. Format per entry: symptom → commit + date → cause → fix → triage tip → keywords. Use Cmd-F / `/` to jump.

---

## 2025-04-04 — `eth0` → `eno1` network interface for SICK LiDAR

- **Commit**: `333f5462` — 2025-04-04 — *"bowser at koopa kingdom is back"*
- **Cause**: Container's `configure-LiDAR.sh` used `ifconfig eth0`, but Jetson's host network only exposes `eno1`.
- **Fix**: Switched to `ip link set eno1 up` and `ip addr add 192.168.0.2/24 dev eno1`.
- **Triage tip**: `ip addr show eno1` should list `192.168.0.2`. If the address went missing, toggle the Lidar button to re-run the config.
- **Keywords**: lidar, sick, network, eno1, eth0, ifconfig, ip-addr, ip-link, 192.168.0.2, configure-lidar.sh, container, entrypoint, interface

## 2025-04-04 — Garbled entrypoint output (`tput`/`ifconfig` syntax)

- **Commit**: `193e7c79` — 2025-04-04 — *"she's working good"*
- **Cause**: Bare `tput 5` (missing `setaf` subcommand) and wrong `ifconfig set eth0` syntax in startup script.
- **Fix**: Replaced with `tput setaf 5` and `ip addr add` / `ip link set`.
- **Triage tip**: If container startup logs print color-control gibberish, it's likely an old `tput` invocation.
- **Keywords**: tput, setaf, ifconfig, ip-addr, ip-link, eth0, eno1, entrypoint, startup-logs, color-codes, container, garbled-output

## 2025-04-16 — SLAM not localizing because TF tree was incomplete

- **Commit**: `86eb7bd8` — 2025-04-16 — *"slam wired to global ekf"*
- **Cause**: SLAM was emitting odometry without a valid `map → odom → base_link` chain, blocking Nav2 / EKF fusion.
- **Fix**: Wired SLAM into the global EKF and ensured `core_bringup.launch.py` publishes the full TF tree from URDF.
- **Triage tip**: `ros2 run tf2_tools view_frames` should show every link from `map` down to each sensor.
- **Keywords**: slam, ekf, tf, tf-tree, map, odom, base_link, core_bringup.launch.py, urdf, robot_state_publisher, view_frames, no-localization, missing-frame, slam_toolbox

## 2025-04-26 — Encoder serial timeouts starved motor commands

- **Commit**: `6a1bdf6a` — 2025-04-26 — *"Trying to fix publishing delay issue"*
- **Cause**: `readString()` calls used 1000 ms / 100 ms timeouts on encoder reads, blocking the motor command loop.
- **Fix**: Reduced all encoder serial timeouts to 20 ms; corrected buffer sizes.
- **Triage tip**: If motor input lags during straight driving, check timestamps on motor writes — gaps of 100 ms+ point at this.
- **Keywords**: encoder, serial, timeout, readString, roboteq, motor, control, latency, blocking, motor-controller.cpp, publish-delay

## 2025-05-05 — Wheel odom only published `String`, no TF broadcast

- **Commit**: `4a1bd68a` — 2025-05-05 — *"odom handler changes"*
- **Cause**: Original code published only a String message; downstream (Nav2, slam_toolbox) expected `nav_msgs/Odometry` and a TF broadcast.
- **Fix**: Switched publisher to `nav_msgs/Odometry`; added `tf2_ros::TransformBroadcaster` for `odom → base_link`.
- **Triage tip**: `ros2 topic echo /odom --once` should show position + quaternion + twist.
- **Keywords**: odom, /odom, wheel-odom, nav_msgs/Odometry, std_msgs/String, TransformBroadcaster, tf2_ros, odom-base_link, wheel_odom_pub.cpp, odom_handler, message-type

## 2025-05-07 — `/wheel_odom` topic name didn't match Nav2 expectations

- **Commit**: `ad6fbef6` — 2025-05-07 — *"who the heck is world"*
- **Cause**: We published to `/wheel_odom`; convention (and Nav2 / slam_toolbox defaults) is `/odom`.
- **Fix**: Renamed topic to `/odom`; remapped LiDAR `/scan` → `/scan_fullframe` for the same kind of consistency.
- **Triage tip**: `ros2 topic list | grep odom` should show `/odom`, not `/wheel_odom`.
- **Keywords**: odom, /odom, /wheel_odom, topic-rename, /scan, /scan_fullframe, remap, nav2, slam_toolbox, naming-convention

## 2025-05-28 — E-stop serial initialization missing device mount

- **Commit**: `c4f7978b` — 2025-05-28 — *"estop stuff ready to test"*
- **Cause**: E-stop UART on Jetson is `/dev/ttyTHS1`, but it wasn't mounted into the container; param defaulted to non-existent `/dev/THS0`.
- **Fix**: Added `estop_port` param defaulting to `/dev/ttyTHS1`, registered an init handler, and added `--device /dev/ttyTHS1` to the docker run script.
- **Triage tip**: If pressing **B** doesn't kill motors, run `ls -la /dev/ttyTHS1` inside the container — it must be present.
- **Keywords**: estop, e-stop, /dev/ttyTHS1, ttyTHS1, /dev/THS0, button-b, kill-motors, --device, container-mount, run-container.sh, uart, serial, estop_port

## 2025-05-28 — Hardcoded waypoint file paths

- **Commit**: `532ff0ba` — 2025-05-28 — *"gps changes yesterday"*
- **Cause**: Python scripts hardcoded `/home/vtuser/...` paths that didn't exist on the deploy Jetson.
- **Fix**: Updated to `/home/vtcro/AutoNav/...` to match the actual deploy structure.
- **Triage tip**: Waypoint commander reports "file not found" → check `stored_waypoints.txt` path in `gps_waypoint_handler/setup.py` console_scripts.
- **Superseded (2026)**: `waypoint_commander` and `stored_waypoints.txt` no longer exist. GPS goals arrive over the `/navigate_to_waypoint` action (`autonav_interfaces`), not a file. Entry kept for historical context.
- **Keywords**: gps, waypoint, gps_waypoint_handler, stored_waypoints.txt, hardcoded-path, /home/vtuser, /home/vtcro, file-not-found, deploy-path, console_scripts, setup.py

## 2025-11-12 — Stale TF used for line projection

- **Commit**: `50b1873e` — 2025-11-12 — *"Fix bug: taking old transform data without checking the timestamps"*
- **Cause**: Line detection accepted any `map ← camera_depth_frame` transform; stale TFs after a SLAM restart projected points into wrong (or NaN) map coords.
- **Fix**: Added 1 s staleness check, depth bounds (10 cm – 20 m), NaN/Inf guards.
- **Triage tip**: Watch for `"Transform from X to map is stale"` warnings; if you see them, your SLAM node is unhealthy.
- **Keywords**: line-detection, line_detector, tf, transform, stale, timestamp, slam-restart, nan, inf, depth, camera_depth_frame, map-frame, lookup-transform

## 2025-11-15 — Encoder serial parser misaligned by command echoes

- **Commit**: `5c75ceff` — 2025-11-15 — *"Changing how the encoder serial is parsed to hopefully remove the encoder reading error."*
- **Cause**: The Roboteq controller echoes `?C`, `!G`, `!C` in the response stream; char-by-char parsing aligned to garbage.
- **Fix**: Switched to regex extraction (`R"((-?\d+))"`) plus echo-detection that triggers a second `readString()`.
- **Triage tip**: Encoder values jumping or sticking → enable INFO logs and check raw buffers for `?C` / `!G`.
- **Keywords**: encoder, serial, parser, roboteq, echo, command-echo, regex, ?C, !G, !C, readString, motor-controller, parse-error, junk-data

## 2025-11-15 — Reverted to simpler reader (kept echo detection)

- **Commit**: `265ec882` — 2025-11-15 — *"Reverting to the simpler reader but still checking for echoed encoder readings on serial."*
- **Cause**: Regex was overkill once echo detection caught the failure mode.
- **Fix**: Reverted to char-by-char extraction after `=` sign, kept echo detection.
- **Triage tip**: If parsing breaks again on a different RoboteQ firmware, this is the first place to look.
- **Keywords**: encoder, serial, parser, revert, simpler-reader, echo-detection, roboteq, motor-controller, char-parsing

## 2025-11-16 — Duplicate ControlNode instances corrupt encoder publisher

- **Commit**: `1ab6dd27` — 2025-11-16 — *"Adding logic to control to ensure that multiple can't be brought up and corrupt data."*
- **Fix**: Hard guard in `configure()` — if `/encoders` already has a publisher, log FATAL and `rclcpp::shutdown()`.
- **Triage tip**: `ros2 topic info /encoders` showing >1 publisher means a second control node started somewhere.
- **Keywords**: control, control-node, /encoders, duplicate, publisher, get_publishers_info_by_topic, race-condition, configure, shutdown, multiple-instances

## 2025-11-16 — Encoder publisher init guard refactor

- **Commit**: `fcf4ddcd` — 2025-11-16 — *"Away with this timer stuff, create a hard guard around the encoder publishing."*
- **Cause**: Earlier fix used a 500 ms guard timer that two simultaneous nodes could race past.
- **Fix**: Moved duplicate check to the configure-service callback; added a hard guard right before creating the publisher and timer.
- **Triage tip**: Kept for future maintainers — startup-race fixes go in service callbacks, not background timers.
- **Keywords**: control, encoder, publisher, guard, configure-service, race-condition, timer, init, refactor, control-node

## 2025-12-03 — GPS serial port path volatility

- **Commit**: `326c8325` — 2025-12-03 — *"Fixing GPS path"*
- **Cause**: Hard-coded `/dev/ttyUSB0` shifted on USB re-enumeration.
- **Fix**: Switched to `/dev/serial/by-id/usb-Prolific_Technology...` (and later `/usb-Cypress_Semiconductor...` after the u-blox swap).
- **Triage tip**: `ls /dev/serial/by-id/` shows the persistent symlink the driver expects.
- **Keywords**: gps, /dev/ttyUSB0, /dev/serial/by-id, usb-prolific, usb-cypress, serial-path, enumeration, persistent-symlink, gps_publisher.cpp, port-volatile

## 2025-12-03 — USB serial device enumeration unstable

- **Commit**: `614df146` — 2025-12-03 — *"Fix/usb order (#7)"*
- **Cause**: Hard-coded `/dev/ttyACM0` / `/dev/ttyACM2` flipped after reboots; container `--device` mounts also missed devices.
- **Fix**: Switched defaults to `/dev/serial/by-id/usb-RoboteQ_*` and `/dev/serial/by-id/usb-Arduino_*`. `run-container.sh` now passes `/dev/ttyACM*`, `/dev/ttyUSB*`, `/dev/video*`, `/dev/ttyTHS1` through.
- **Triage tip**: `dmesg | tail` after replug — confirm what device path the kernel assigned, then check `/dev/serial/by-id/`.
- **Keywords**: usb, /dev/ttyACM0, /dev/ttyACM2, /dev/ttyUSB, /dev/serial/by-id, roboteq, arduino, run-container.sh, --device, dmesg, enumeration, persistent-path

## 2026-02-02 — TF broadcast guarded behind compile flag

- **Commit**: `9fd9be3b` — 2026-02-02 — *"defined PUBLISH_TRANSFORM at build time"*
- **Cause**: `#ifdef PUBLISH_TRANSFORM` blocks were silently compiled out.
- **Fix**: `target_compile_definitions(... PRIVATE PUBLISH_TRANSFORM)` in CMakeLists.txt.
- **Triage tip**: If TF goes silent for a service that just rebuilt, check for stray `#ifdef` guards on your TF broadcaster.
- **Keywords**: tf, transform, PUBLISH_TRANSFORM, ifdef, target_compile_definitions, CMakeLists.txt, wheel_odom_pub.cpp, build-flag, broadcaster, silent-fail

## 2026-03-04 — SICK driver crash didn't auto-restart

- **Commit**: `294d5cf2` — 2026-03-04 — *"Fix/lidar driver (#12)"*
- **Cause**: Original launch had `respawn="false"`.
- **Fix**: Added `required="true"` so ROS 2 immediately restarts the driver if it exits.
- **Triage tip**: HUD shows green but `ros2 node list` is missing `multiScan` → check launch logs for the death cause.
- **Keywords**: sick, lidar, driver, crash, respawn, required, sick_multiscan.launch, multiScan, auto-restart, exit, launch-file

## 2026-03-20 — Local costmap tilt

- **Commit**: `ef23f983` — 2026-03-20 — *"Fixing local costmap tilt"*
- **Cause**: Line detector published in the camera frame; ZED was also publishing its own TFs, conflicting with URDF + slam_toolbox.
- **Fix**: Set `target_frame: "map"` in `line_detector.yaml`; ZED launched with `publish_tf:=false`, `publish_map_tf:=false`.
- **Triage tip**: Costmap tilted in RViz → check `target_frame` and the ZED publish-TF args.
- **Keywords**: costmap, tilt, local-costmap, line-detector, target_frame, line_detector.yaml, zed, publish_tf, publish_map_tf, urdf, slam_toolbox, frame-mismatch

## 2026-03-21 — Costmap data type mismatch (width/height)

- **Commit**: `09f286c8` — 2026-03-21 — *"Data type mismatch fixed for height and width fo the costmap"*
- **Cause**: YAML was loading width/height as the wrong numeric type, leaving them at 0.
- **Fix**: Aligned the YAML types with what the costmap plugin expects.
- **Triage tip**: If `local_costmap` or `global_costmap` initializes with zero dimensions, this is the first place to look.
- **Keywords**: costmap, width, height, data-type, yaml, nav2_paramsv2.yaml, local_costmap, global_costmap, type-mismatch, zero-dimension, init-failure

## 2026-03-21 — Costmap window size + ghost traces

- **Commit**: `0eb436e9` — 2026-03-21 — *"Adjusted local and global costmap window size and removed ghost traces"*
- **Cause**: Line layer wasn't resetting between updates; old detections persisted as ghosts.
- **Fix**: `isClearable()` returns true; `matchSize()` on init; `resetMaps()` before applying detections.
- **Triage tip**: Old line marks linger after the line moved → confirm `LineLayer::reset()` is called.
- **Keywords**: costmap, line-layer, ghost-trace, ghost-obstacle, isClearable, matchSize, resetMaps, reset, line_layer.cpp, persistent-obstacle

## 2026-03-24 — Double TF broadcasts from `tf_publish_rate`

- **Commit**: `cf8b6b90` — 2026-03-24 — *"Getting rid of some extra parameters that might be messing up the lidar."*
- **Cause**: Redundant `tf_publish_rate` overrides in `sensors.launch.py` conflicted with URDF.
- **Fix**: Removed the overrides; we now pass `tf_publish_rate:=0` and rely on URDF for `base_link → lidar_footprint`.
- **Triage tip**: TF tree showing duplicate edges → check launch args for `tf_publish_rate`.
- **Keywords**: tf, tf_publish_rate, sensors.launch.py, urdf, base_link, lidar_footprint, double-broadcast, duplicate-tf, sick, lidar

## 2026-03-24 — `max_laserscan_range` LaserScan constraints

- **Commit**: `039845a1` — 2026-03-24 — *"Added demo_day.launch.py and fixed the laserscan contraints"*
- **Cause**: SICK driver was publishing `Inf` ranges that propagated into Nav2's ObstacleLayer.
- **Fix**: Added `max_laserscan_range:=10.0` to `sick_multiscan.launch`.
- **Triage tip**: `ros2 topic echo /scan_fullframe | head` — no `inf`/`NaN` should appear.
- **Keywords**: lidar, sick, laserscan, max_laserscan_range, inf, nan, range, ObstacleLayer, nav2, sick_multiscan.launch, demo_day.launch.py, /scan_fullframe

## 2026-03-24 — Demo-day startup race needed staggered delays

- **Commit**: `bb2f4716` — 2026-03-24 — *"Added delays between node bringups for full bringup launch script"*
- **Cause**: All nodes launching at once raced on TF and topic readiness.
- **Fix**: Wrapped sensors / SLAM / line detection / Nav2 in `TimerAction` with staggered 3 / 8 / 14 / 18 s offsets.
- **Triage tip**: TF lookup errors in demo_day → bump the delay arguments.
- **Keywords**: demo_day.launch.py, TimerAction, startup-race, race-condition, tf-lookup, delay, staggered, bringup, full-stack

## 2026-03-24 — Demo-day eth setup automated

- **Commit**: `78d3db99` — 2026-03-24 — *"Having the demo_day.launch.py fix the ethernet"*
- **Cause**: `eno1` had to be manually configured before launching.
- **Fix**: `ExecuteProcess` action in `demo_day.launch.py` runs the IP commands before sensors.
- **Triage tip**: SICK works under `run-lidar.sh` but not `demo_day.launch.py` → check that the network ExecuteProcess fired.
- **Keywords**: demo_day.launch.py, eno1, network, ip-addr, ExecuteProcess, run-lidar.sh, sick, lidar, network-setup, automation

## 2026-03-24 — Hardcoded BT XML paths

- **Commit**: `4917d9ca` — 2026-03-24 — *"Fixed wrong path to bt2.xml"*
- **Cause**: Absolute machine-specific paths in launch files / YAML.
- **Fix**: `nav.launch.py` now resolves `bt_nav.xml` via `ament_index_python.get_package_share_directory`.
- **Triage tip**: `bt_navigator: file not found` → confirm install dir contains `slam/share/slam/behavior_trees/`.
- **Keywords**: behavior-tree, bt, bt_nav.xml, bt2.xml, nav.launch.py, hardcoded-path, ament_index_python, get_package_share_directory, file-not-found, bt_navigator

## 2026-03-24 — Progress checker reacted too slowly

- **Commit**: `0b734f36` — 2026-03-24 — *"If stuck react quicker"*
- **Cause**: 5 s movement allowance + 0.15 m radius = robot took 5 s to detect being stuck.
- **Fix**: 3 s allowance, 0.10 m radius; tightened angular limits to reduce oscillation.
- **Triage tip**: Robot stalls noticeably before recovering → tune the progress-checker thresholds.
- **Keywords**: progress-checker, stuck, movement_time_allowance, required_movement_radius, max_vel_theta, acc_lim_theta, nav2, oscillation, recovery-delay

## 2026-03-24 — Line detection flickered on RGB/depth desync

- **Commit**: `a455f638` — 2026-03-24 — *"Increased backing out distance to 0.3m and increased holding the last line set to 750ms"*
- **Cause**: Brief sensor desyncs flushed the line layer, breaking Nav2 around obstacles.
- **Fix**: Added `line_hold_timeout_ms: 750` to keep the last valid set across short outages; gave `LinePoints` a header.
- **Triage tip**: Line obstacles flicker in costmap → raise `line_hold_timeout_ms`.
- **Keywords**: line-detection, line-layer, flicker, line_hold_timeout_ms, rgb-depth-sync, max_rgb_depth_delta_ms, LinePoints, header, sensor-desync, line_detector.yaml

## 2026-03-26 — Line layer stale obstacles block Nav2

- **Commit**: `aee664f4` — 2026-03-26 — *"Fix stale line-layer obstacles blocking Nav2 path execution"*
- **Cause**: Layer never cleared when the detector went silent.
- **Fix**: `publishHeldOrEmpty()` checks elapsed time vs `line_hold_timeout_ms`; emits empty `LinePoints` to clear.
- **Triage tip**: Path blocked by invisible obstacles → check `line_hold_timeout_ms` is finite, and that the detector is publishing fresh stamps.
- **Keywords**: line-layer, stale-obstacle, blocked-path, nav2, planner, line_hold_timeout_ms, publishHeldOrEmpty, /line_points, costmap, ghost-obstacle, layer-clear

## 2026-03-26 — eno1 network configuration automation

- **Commit**: `8f854997` — 2026-03-26 — *"Configure network settings for lidar script"*
- **Fix**: Network setup now happens automatically in `run-lidar.sh` on every Lidar button press, not as a manual prereq.
- **Triage tip**: If you ever need to do it by hand: `sudo ip addr flush dev eno1 && sudo ip addr add 192.168.0.2/24 dev eno1 && sudo ip link set eno1 up`.
- **Keywords**: eno1, network, run-lidar.sh, ip-addr, ip-link, 192.168.0.2, sudo, lidar, sick, automation, network-setup, configure-lidar

## 2026-03-28 — INA226 init failure was fatal, no retry

- **Commit**: `d08ca50c` — 2026-03-28 — *"Electrical publisher bugfix"*
- **Cause**: A transient I²C failure during boot killed the node permanently.
- **Fix**: Refactored to `try_init_ina226()` running in a periodic timer; logs `WARN_THROTTLE` and retries every 5 s; verifies the calibration register readback.
- **Triage tip**: Wait for "calibrated and ready" in the logs before trusting voltage/current readings.
- **Keywords**: ina226, i2c, electrical, autonav_electrical_publisher, init-failure, retry, try_init_ina226, calibration, register, 0x05, 0x0800, voltage, current, power-pcb

## 2026-04-04 — Encoder publisher null crash blocked controller

- **Commit**: `c46a15d3` — 2026-04-04 — *"bugfix in control for t000 where it would block controller input"*
- **Cause**: When duplicate guard skipped publisher creation, the encoder timer still tried to publish — null deref.
- **Fix**: Always create the timer; null-check before `publish()`.
- **Triage tip**: Controller input goes silent right after init → look for null-pointer dereferences in the backtrace.
- **Keywords**: control, encoder, publisher, null-deref, segfault, encoder_timer, t000, joystick, controller-input, control.cpp, encodersPub

## 2026-04-06 — Container user initialization race

- **Commit**: `4be1ba4d` — 2026-04-06 — *"Wait for user on container launch"*
- **Cause**: `attach_shell` exec'd before the entrypoint finished creating the user account.
- **Fix**: `wait_for_container_user()` polls `getent passwd` for up to 30 s.
- **Triage tip**: "could not find user" right after `docker run` → wait or check `docker logs koopa-kingdom`.
- **Keywords**: container, docker, user-not-found, getent, attach_shell, wait_for_container_user, run-container.sh, init-race, koopa-kingdom, admin, entrypoint

## 2026-04-06 — ZED SDK CMake/include path errors

- **Commit**: `92e802d0` — 2026-04-06 — *"Fix zed sdk error"*
- **Cause**: Stale `:${...}` in CMake env vars; missing symlink for SDK 5 CMake module.
- **Fix**: Cleaned env vars; added `ln -sfn /usr/local/zed/lib /usr/local/zed/lib/cmake/ZED/lib`.
- **Triage tip**: CMake "ZED not found" → confirm that symlink exists.
- **Keywords**: zed, cmake, sdk, sdk-5, CMAKE_PREFIX_PATH, CPATH, CPLUS_INCLUDE_PATH, /usr/local/zed, symlink, find_package, build-error, zed-not-found

## 2026-04-07 — ZED calibration files couldn't persist

- **Commits**: `52fd9663` / `13deb88a` — 2026-04-07 — *"Directories for camera config" / "SDK paths too now exist"*
- **Cause**: SDK 5.x writes `SN<serial>.conf` into the read-only image path → "CALIBRATION FILE NOT AVAILABLE".
- **Fix**: Created host dirs `~/zed/{settings,resources}` and bind-mounted them into both legacy and SDK 5.x paths inside the container.
- **Triage tip**: ZED logs "CALIBRATION FILE NOT AVAILABLE" → confirm the host dirs exist and are mounted.
- **Keywords**: zed, calibration, SN, .conf, settings, resources, ~/zed, bind-mount, sdk-5, /usr/local/zed/settings, CALIBRATION-FILE-NOT-AVAILABLE, run-container.sh

## 2026-04-07 — Removed redundant `BESTVELA` GPS logging

- **Commit**: `9e1fe23e` — 2026-04-07 — *"Do not need BESTVELA"*
- **Fix**: Dropped velocity logging; we only use position-critical `BESTPOSA`.
- **Triage tip**: If GPS feels slow, you don't need to publish what you don't consume.
- **Keywords**: gps, novatel, BESTVELA, BESTPOSA, velocity, logging-overhead, gps_publisher.cpp, log-removal

## 2026-04-07 — GPS yaw jitter at low speeds

- **Commit**: `4662f6e6` — 2026-04-07 — *"Should not jump around at low speeds."*
- **Cause**: Heading calibrator updated yaw even when motion was below noise floor (~0.15 m/s).
- **Fix**: `min_speed_mps` threshold (default 0.3 m/s); skip updates below it.
- **Triage tip**: Heading oscillates while parked or creeping → raise `min_speed_mps`.
- **Keywords**: gps, heading, yaw, jitter, low-speed, min_speed_mps, calibration, noise-floor, gps_publisher.cpp, parked-jitter

## 2026-04-08 — X11 display forwarding hangs (MIT-SHM)

- **Commit**: `8ff710f4` — 2026-04-08 — *"Fix display forwarding"*
- **Fix**: `QT_X11_NO_MITSHM=1`; tightened xauth permissions; flushed stale auth entries.
- **Triage tip**: RViz hangs on window create → confirm `QT_X11_NO_MITSHM=1` is set in both the container and `attach_shell` env.
- **Keywords**: x11, display, rviz, qt, MIT-SHM, QT_X11_NO_MITSHM, xauth, .Xauthority, hang, run-container.sh, attach_shell, ssh-forwarding, glx

## 2026-04-15 — Caster + wheel dynamics for MuJoCo

- **Commit**: `8da4dccb` — 2026-04-15 — *"Make robot manueverable and fix caster rotation, swivel"*
- **Fix**: Hierarchical body structure with caster swivel + wheel roll joints, damping/armature, collision geometry, velocity controllers with force limits.
- **Triage tip**: Sim wheels slip or lock → check `damping`, `armature`, friction (~1.6).
- **Keywords**: mujoco, sim, simulation, caster, wheel, joint, damping, armature, friction, shogi.xml, dynamics, swivel, slip

## 2026-04-15 — Joystick axis mapping reversed

- **Commit**: `8bcff849` — 2026-04-15 — *"Correct left vs right wheel"*
- **Fix**: Swapped left/right stick → motor assignments in `xbox.cpp` and `control.cpp`.
- **Triage tip**: Push left stick forward, robot pivots wrong way → swap the assignments.
- **Keywords**: joystick, xbox, left-stick, right-stick, motor, control.cpp, xbox.cpp, axis-mapping, reversed, manual-mode, tank-drive

## 2026-04-15 — Wheel radius wrong after Bowser → Shogi swap

- **Commit**: `c066fc58` — 2026-04-15 — *"Changing the wheel radius from Bowser to Shogi"*
- **Cause**: Constant stuck at 0.205 m (Bowser) on Shogi (0.12946 m). All odom distances were proportionally wrong.
- **Fix**: Updated `wheel_radius_` in C++ and reference Python.
- **Triage tip**: Odom distance off by a fixed scale factor → measure the wheel and update.
- **Keywords**: odom, wheel-radius, wheel_radius_, 0.205, 0.12946, bowser, shogi, robot-swap, scale-factor, distance, wheel_odom_pub.cpp, calibration

## 2026-04-15 — X11 authorization wildcards

- **Commit**: `d8f2084e` — 2026-04-15 — *"Improve x auth copying"*
- **Fix**: `_xauth_names_for_display()` tries multiple lookup patterns (DISPLAY, hostname/unix:N, unix:N, localhost:N).
- **Triage tip**: "Authorization required" → check `run-container.sh` output for which Xauthority sources it tried.
- **Keywords**: x11, xauth, .Xauthority, authorization-required, _xauth_names_for_display, DISPLAY, run-container.sh, hostname, unix:N, localhost, ssh-forwarding

## 2026-04-15 — Jetson SSH X11 forwarding fell back to software GL

- **Commit**: `72fcc210` — 2026-04-15 — *"Adjust display"*
- **Fix**: Auto-detect aarch64 + `localhost:*` DISPLAY → switch to `:0`. Added `/tmp/.X11-unix` mount and `XDG_RUNTIME_DIR` passthrough.
- **Triage tip**: RViz on Jetson hangs/blank → check `DISPLAY` value; SSH-forwarded values force software GL.
- **Keywords**: x11, ssh, jetson, aarch64, software-gl, DISPLAY, localhost, /tmp/.X11-unix, XDG_RUNTIME_DIR, rviz, hardware-gl, glxinfo

## 2026-04-15 — Added `ObstacleFootprint` critic

- **Commit**: `e9c5bf90` — 2026-04-15 — *"Add ObstacleFootprint critic for controller"*
- **Fix**: Added the critic to DWB's list with scale 1.0; controller now evaluates obstacle distance footprint-aware.
- **Triage tip**: Robot clips obstacles in tight spaces → confirm `ObstacleFootprint` is in the critics list.
- **Keywords**: nav2, dwb, controller, ObstacleFootprint, critic, footprint, obstacle, nav2_paramsv2.yaml, BaseObstacle, scale, clipping

## 2026-04-15 — Footprint + inflation radii rebalanced

- **Commit**: `40d25a2e` — 2026-04-15 — *"Add footprint and adjust inflation radii"*
- **Cause**: `robot_radius` 0.41 m circle oversimplified the actual 0.52 × 0.82 m rectangle.
- **Fix**: Explicit footprint polygon; **local** inflation 0.3 m (DWB knows the shape), **global** 0.8 m (NavfnPlanner doesn't).
- **Triage tip**: Plans hug obstacles too tight or veer wildly → check inflation per costmap.
- **Keywords**: footprint, inflation, robot_radius, polygon, nav2_paramsv2.yaml, local-inflation, global-inflation, dwb, navfn, planner, 0.52, 0.82

## 2026-04-16 — ZED IMU 100 Hz limit override

- **Commit**: `82fe5a4c` — 2026-04-16 — *"Override the 100 Hz limit and support de-duplication in the DAQ"*
- **Fix**: `zed_override.yaml` sets `sensors_pub_rate: 380.0`; DAQ added new-data flag tracking.
- **Triage tip**: IMU fusion feels coarse → confirm the override file is being passed to the launch.
- **Keywords**: zed, imu, sensors_pub_rate, 100hz, 380hz, zed_override.yaml, ros_params_override_path, ekf, fusion, daq, deduplication

## 2026-04-17 — INA226 I²C bus path collision with onboard INA3221

- **Commit**: `802223a9` — 2026-04-17 — *"Unbinding the INA3221 from the Jetson so we can use the external INA226 one"*
- **Cause**: Jetson's onboard INA3221 kernel driver claimed addr `0x40` on bus 1, blocking the external PCB.
- **Fix**: Updated bus path to `/dev/i2c-1`; added `unbind-ina3221.sh` to deregister the kernel driver from `0x40` on bus 1.
- **Triage tip**: Voltage reads zero → check `cat /sys/bus/i2c/drivers/ina3221/unbind` and the entrypoint logs for the unbind line.
- **Keywords**: ina226, ina3221, i2c, /dev/i2c-1, /dev/i2c-7, 0x40, unbind, kernel-driver, electrical, autonav_electrical_publisher, unbind-ina3221.sh, bus-collision, address-conflict, voltage-zero

## 2026-04-17 — Jetson Orin Nano bricked by manual device tree overlay (incident)

- **Incident time**: 2026-04-17 14:00, AMP lab. Branch `test/t000-dev`, PCB rev 3 (purple), MacBook Pro host.
- **Symptoms**: Jetson got very hot during a failed boot, fan stopped spinning after ~10 s, USB enumeration never appeared, SSH timed out, ping stalled.
- **Cause**: While trying to re-enable I²C on header pins 3/5 (`i2c8` → controller `i2c@31e0000` = `dp_aux_ch3_i2c`), the user hand-wrote a `.dtbo` device-tree overlay flipping that controller's `status` from `disabled` → `okay` and appended it to `OVERLAYS=` in `/boot/extlinux/extlinux.conf`. The controller is a **DisplayPort AUX channel** that NVIDIA disabled deliberately; enabling it without its parent DP controller triggered a kernel panic (or UEFI boot failure) on next boot. The `compatible` string also only listed one board variant instead of NVIDIA's full set.
- **Fix (recovery, NOT prevention)**: Forced recovery mode by jumpering pins **9 and 10** on the button header (FC REC + GND), then reflashed JetPack 36.4.0 via `sdkmanager` from the AMP-lab Linux laptop. NVIDIA recovery guide: <https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/howto.html>.
- **Triage tip**: Hot Jetson + stopped fan + no USB enum + no SSH = forced recovery mode + reflash. Don't waste time on `screen /dev/tty.debug-console` or `ssh -o ConnectTimeout=10`; if the boot is genuinely incomplete, those won't help. **Power off first.**
- **Lessons** (full action-by-action incident report on [SharePoint](https://virginiatech.sharepoint.com/:w:/r/sites/IDC2024-2025/AutoNav/Shared%20Documents/AutoNav%202025%20-%202026/Software/Important%20References/April%2017%202026%20Jetson%20incident%20report.docx?d=w89d25418776c4543ab058dc7d734e0f5&csf=1&web=1&e=3tfO3t)):
  1. Disabled controllers in the device tree are disabled for a reason.
  2. Prefer **runtime** GPIO/I²C reconfig over **boot-time** device-tree changes — runtime is reversible, boot-time is not.
  3. The official tools (`jetson-io.py`, `config-by-function.py`, `config-by-pin.py`) are safe; hand-written `.dtbo` files plus manual `extlinux.conf` edits are not.
  4. Always have a recovery plan before any boot-time change. Know where the FC REC pin is.
  5. The actual fix for the original I²C problem turned out to be **moving the wires from pins 3/5 to pins 27/28** (bus 1, already enabled) and unbinding the kernel `INA3221` driver from address `0x40`. That fix is captured in commit `802223a9` and the entry above; the device-tree-overlay attempt was a parallel experiment that wasn't necessary.
- **Keywords**: jetson, brick, bricked, orin-nano, no-boot, no-ssh, no-usb-enum, hot, fan-stops, recovery-mode, force-recovery, fc-rec, button-header, pin-9, pin-10, sdkmanager, jetpack, jetpack-36.4.0, flash.sh, dtbo, .dtbo, device-tree, overlay, /boot/extlinux/extlinux.conf, OVERLAYS, jetson-io.py, config-by-function.py, config-by-pin.py, dp_aux_ch3_i2c, i2c@31e0000, i2c8, kernel-panic, uefi, reflash, incident, displayport-aux, nvidia, header-pins, pins-3-5, pins-27-28, AMP-lab

## 2026-04-17 — `zed_override.yaml` introduced (IMU rate boost)

- **Commit**: `8863842a` — 2026-04-17 — *"Added yaml overwrite to increase IMU publish rate"*
- **Fix**: Override loads after the wrapper defaults so we win the merge.
- **Triage tip**: See the full ZED launch breakdown in [`docs/SENSORS.md`](./SENSORS.md#zed-2i-camera) and `PACKAGES.md`.
- **Keywords**: zed, zed_override.yaml, imu, sensors_pub_rate, override, ros_params_override_path, run-zed.sh, common_stereo.yaml, zed2i.yaml

## 2026-04-17 — NovAtel to u-blox parser mismatch

- **Commit**: `7beb4b98` — 2026-04-17 — *"Updated the gps_publisher to u-blox format"*
- **Cause**: Parser expected NovAtel binary but received u-blox NMEA GGA/RMC.
- **Fix**: Rewrote the parser around NMEA GGA; baud → 38400; serial path → `/dev/serial/by-id/usb-Cypress_Semiconductor...`.
- **Triage tip**: GPS coords look gibberish → confirm the receiver is in NMEA mode and the baud matches.
- **Keywords**: gps, novatel, u-blox, ublox, zed-f9p, nmea, gga, rmc, baud, 38400, gps_publisher.cpp, /dev/serial/by-id, usb-cypress, parser, BESTPOSA, garbled-coords

## 2026-04-22 — Costmap geometry mismatch with SLAM

- **Commit**: `b4354d31` — 2026-04-22 — *"Fixing Costmap"*
- **Fix**: Removed unused `line_memory_resolution_m`; tuned obstacle layer to subscribe to the right topics; cleaned up bounds.
- **Triage tip**: Costmap origin doesn't follow the robot → check static_layer status and SLAM map origin.
- **Keywords**: costmap, slam, line_memory_resolution_m, obstacle-layer, static-layer, nav2_paramsv2.yaml, origin, geometry, line-detector

## 2026-04-22 — DDS UDP discovery forced

- **Commit**: `40b2c4c5` — 2026-04-22 — *"DDS fix"*
- **Cause**: Shared-memory transport failed across machines; discovery silently broke.
- **Fix**: `fastdds_udp.xml` profile forces UDPv4; `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` + `FASTRTPS_DEFAULT_PROFILES_FILE` everywhere.
- **Triage tip**: Remote RViz can't see Jetson topics → confirm both env vars match on both ends.
- **Keywords**: dds, fastdds, fastrtps, rmw, RMW_IMPLEMENTATION, FASTRTPS_DEFAULT_PROFILES_FILE, FASTDDS_DEFAULT_PROFILES_FILE, fastdds_udp.xml, udpv4, shared-memory, discovery, rviz-remote, rmw_fastrtps_cpp

## 2026-04-22 — Headless container migration (X11 → DDS exposure)

- **Commit**: `e63c968a` — 2026-04-22 — *"REplaced X11 forwarding with ROS graph exposure via DDS"*
- **Fix**: `AUTONAV_CONTAINER_GUI=0` (default) skips X11 setup; `REMOTE_RVIZ.md` documents the laptop-side setup.
- **Triage tip**: Don't run RViz inside the container anymore; run it locally on your Linux machine.
- **Keywords**: x11, dds, headless, AUTONAV_CONTAINER_GUI, run-container.sh, REMOTE_RVIZ.md, rviz, laptop, ros-graph, no-forwarding, container-gui

## 2026-04-22 — Line detection thresholds relaxed

- **Commit**: `89b8da3f` — 2026-04-22 — *"RElaxing the CUDA thresholds and strict continuity checking"*
- **Fix**: `SIGMA_THRESHOLD` 5 → 8; `kMinLineComponentSpanPx` replaced by `kMinLineAspectRatio = 4.0`; min area 40 → 20 px.
- **Triage tip**: Short tape segments disappear → check the aspect-ratio filter in `cuda.cu` / `detection.cpp`.
- **Keywords**: line-detection, cuda, SIGMA_THRESHOLD, sigma_threshold, kMinLineComponentSpanPx, kMinLineAspectRatio, aspect-ratio, cuda.cu, detection.cpp, threshold, tape, short-segment

## 2026-04-22 — Smaller window size for distant tape

- **Commit**: `55e48ae3` — 2026-04-22 — *"Changed the half_window_size to 5x5 and increased the threshold"*
- **Fix**: `HALF_WINDOW_SIZE` 3 → 2 (5×5 window); brightness threshold 220 → 230.
- **Triage tip**: Detection drops at distance → smaller window adapts to perspective shrinkage.
- **Keywords**: line-detection, cuda, HALF_WINDOW_SIZE, half_window_size, brightness_threshold, 220, 230, kernel-window, distant-tape, perspective, line_detector.yaml

## 2026-04-22 — NumPy 2.x ABI break in OpenCV

- **Commit**: `d3467780` — 2026-04-22 — *"Fixing the video recorder and numpy version in the dev layer"*
- **Cause**: pip pulled numpy 2.x; system OpenCV/cv_bridge built against numpy 1.x → segfaults.
- **Fix**: Pinned numpy `<2` in dev Dockerfile; added `SensorDataQoS()` for camera/lidar in test automator.
- **Triage tip**: Video recorder segfaults → `pip list | grep numpy` should show 1.x.
- **Keywords**: numpy, numpy-2, abi, opencv, cv_bridge, dockerfile, dev-layer, segfault, pip, version-pin, video-recorder, SensorDataQoS

## 2026-04-22 — Rolling global costmap

- **Commit**: `226d2b0d` — 2026-04-22 — *"Rolling fixed-size window around the robot"*
- **Fix**: `rolling_window: true`, 20 × 20 m, removed `static_layer`.
- **Triage tip**: Planner ignores distant obstacles → confirm rolling_window is true and StaticLayer is disabled.
- **Keywords**: costmap, rolling_window, static_layer, global-costmap, nav2_paramsv2.yaml, distant-obstacle, planner, window-size, 20x20

## 2026-04-24 — Live mode flashing all dots

- **Commit**: `a22e6258` — 2026-04-24 — *"Devices in live mode fix"*
- **Fix**: `_live_sensors` set narrowed to Camera/Lidar/GPS/Encoders/Power PCB; non-sensor dots stay off.
- **Triage tip**: Non-sensor dots flashing in Live mode → upgrade.
- **Keywords**: gui, hud, live-mode, _live_sensors, dots, flashing, hud_node.py, sensor-list, status-dot

## 2026-04-24 — Playback odom scatter rebuild every frame

- **Commit**: `16df680e` — 2026-04-24 — *"Fixing the install sh for the gui and the lag in playback mode."*
- **Fix**: Switched to a line plot with in-place updates; throttled redraws; fixed `install.sh`.
- **Triage tip**: Playback lags → confirm the plot is using set_data, not re-creating the artist.
- **Keywords**: gui, hud, playback, scatter-plot, line-plot, set_data, matplotlib, lag, install.sh, hud_node.py, performance

## 2026-04-24 — QoS mismatch on live sensor subscriptions

- **Commit**: `90cfedea` — 2026-04-24 — *"QoS mismatch fixing."*
- **Cause**: HUD subscribed reliable + depth=1; publishers used BEST_EFFORT + VOLATILE.
- **Fix**: Defined `_SENSOR_QOS` (BEST_EFFORT, VOLATILE, KEEP_LAST, depth=1) and applied to all 7 sensor subs.
- **Triage tip**: "Awaiting live data" forever → check middleware logs for QoS negotiation errors.
- **Keywords**: gui, hud, qos, qos-mismatch, _SENSOR_QOS, BEST_EFFORT, VOLATILE, KEEP_LAST, qos_profile_sensor_data, hud_node.py, awaiting-live-data, subscription

## 2026-04-24 — Terminal widget unresponsive (matplotlib warnings + 500 ms refresh)

- **Commit**: `5b3016d9` — 2026-04-24 — *"Fixed terminal spamming, device info in the screen to the right of the gui, and more live mode debugging."*
- **Fix**: Suppressed matplotlib warnings; aspect mode `'box'`; visual highlight on selected device; debug subscription logs at startup.
- **Triage tip**: GUI freezes during high-output runs → check the terminal-refresh timer.
- **Keywords**: gui, hud, terminal, matplotlib, warnings, aspect-mode, freeze, terminal-spam, hud_node.py, refresh-timer, performance

## 2026-04-24 — Device button highlight desynced with selection

- **Commit**: `7dd1b41a` — 2026-04-24 — *"Bugfixing with the buttons and also live mode."*
- **Fix**: Sync selection highlight with `_selected_process`; added 5 s callback counters for live debugging.
- **Triage tip**: The highlighted button doesn't match the terminal you're looking at → restart GUI.
- **Keywords**: gui, hud, button, _selected_process, highlight, terminal-viewer, callback-counter, hud_node.py, selection-sync

## 2026-04-25 — Obstacle layer switched LaserScan → PointCloud2

- **Commit**: `6bd524ca` — 2026-04-25 — *"Switch obstacle layer to use pointcloud, rather than laserscan"*
- **Cause**: 2D scan misses height-filtered obstacles; 3D cloud separates ground from obstacles.
- **Fix**: Source = `/cloud_all_fields_fullframe`; `min_obstacle_height: 0.4`.
- **Triage tip**: Robot hits obstacles at certain heights → tune `min/max_obstacle_height`.
- **Keywords**: obstacle-layer, laserscan, pointcloud, /cloud_all_fields_fullframe, /scan_fullframe, min_obstacle_height, max_obstacle_height, nav2_paramsv2.yaml, costmap, height-filter, 3d-obstacle

## 2026-04-28 — GPS crash on invalid `stoi`

- **Commit**: `5ae4b93` — 2026-04-28 — *"GPS connection fails when invalid STOI comes in."*
- **Fix**: Try/catch around fix-quality parsing; auto-reconnect after 10 consecutive failures (~5 s).
- **Triage tip**: Watch for "GPS connection lost — attempting reconnect" in logs.
- **Keywords**: gps, stoi, std::stoi, exception, crash, parse-error, fix-quality, auto-reconnect, gps_publisher.cpp, nmea, corrupted-data

## 2026-04-28 — Status dots resetting on mode switch

- **Commit**: `abf17b1b` — 2026-04-28 — *"Dots not staying green bugfix."*
- **Fix**: Mode switch only turns off dots whose process isn't actually running.
- **Triage tip**: (fix is in code; not user-visible anymore)
- **Keywords**: gui, hud, status-dot, mode-switch, dots, _process_objects, hud_node.py, green-dot, dots-resetting

## 2026-04-28 — NaN/Inf lidar rays drawn as white lines

- **Commit**: `38e346ed` — 2026-04-28 — *"NaN lines should not draw."*
- **Fix**: Skip NaN/Inf instead of drawing 10 m white rays.
- **Triage tip**: White rays cluttering the lidar view → upgrade.
- **Keywords**: gui, hud, lidar-view, nan, inf, white-rays, hud_node.py, range-overflow, visualization

## 2026-04-28 — Playback laggy from terminal refresh on 2 Hz polling

- **Commit**: `123d20d4` — 2026-04-28 — *"Performance improvements related to terminal output logging causing lag."*
- **Fix**: Process poll → 1 Hz; redraws throttled to ~4 Hz; per-process buffer 200 → 500 lines.
- **Triage tip**: GUI sluggish under high stdout → check polling interval.
- **Keywords**: gui, hud, performance, lag, terminal, polling, _MAX_BUF_LINES, redraw-throttle, hud_node.py, 1hz, 4hz, buffer-size

## 2026-04-28 — Terminal buffer dropping early lines

- **Commit**: `ec3a5cf6` — 2026-04-28 — *"Raise the terminal memory buffer."*
- **Fix**: Removed aggressive cap; raised `_MAX_BUF_LINES` from 200 to 500.
- **Triage tip**: Early launch output missing → check buffer size.
- **Keywords**: gui, hud, terminal, buffer, _MAX_BUF_LINES, dropped-lines, 200, 500, hud_node.py, log-truncation

## 2026-04-28 — More HUD performance work

- **Commit**: `43f47b5d` — 2026-04-28 — *"More performance updates"*
- **Fix**: Batched terminal updates; conditional visibility checks for off-screen elements.
- **Keywords**: gui, hud, performance, batch-update, visibility-check, hud_node.py, off-screen

## 2026-04-28 — Process polling cadence tuned

- **Commit**: `d8600ef3` — 2026-04-28 — *"Performance improvements"*
- **Fix**: Tuned `_process_poll_timer` interval; balance responsiveness vs CPU.
- **Keywords**: gui, hud, performance, _process_poll_timer, polling-rate, hud_node.py, cpu-usage

## 2026-04-28 — HUD throttle reverted (odom plot was hiding spikes)

- **Commit**: `38dab145` — 2026-04-28 — *"Reverting a couple changes because Odom is jumping around."*
- **Fix**: Removed dirty-flag throttles so odom jumps are actually visible in the plot.
- **Triage tip**: If odom plot looks jerky after this commit, the upstream is genuinely jumping — `ros2 topic hz /odom`.
- **Keywords**: gui, hud, odom, jumping, throttle-revert, _odom_dirty, hud_node.py, plot-redraw, visibility

## 2026-04-29 — Behavior tree triggering when not autonomous

- **Commit**: `8496e123` — 2026-04-29 — *"Experimental setup for the robot behavior and fixes for the robot getting into the behavior tree when not wanted."*
- **Fix**: Control node publishes `/autonomous_mode`; BT and recovery behaviors gate on it. Map padder pre-filters transient obstacles.
- **Triage tip**: BT recovery firing in manual mode → confirm `/autonomous_mode` is publishing.
- **Keywords**: behavior-tree, bt, /autonomous_mode, autonomous_mode, manual-mode, recovery, gradient_escape, goal_bender, map_padder, control.cpp, bt_nav.xml

## 2026-04-29 — Left encoder over-counts, causing rightward drift

- **Commit**: `6485b9f8` — 2026-04-29 — *"Encoders bend to the right a ton because the left encoder oversamples."*
- **Cause**: Left encoder physically over-samples (~2.7%).
- **Fix**: `left_encoder_scale_ ≈ 1.016335` divides into the left displacement.
- **Triage tip**: Robot drifts steadily right → check the calibration factor.
- **Keywords**: encoder, left-encoder, oversample, drift, right-drift, left_encoder_scale_, 1.016335, 1.016, calibration, wheel_odom_pub.cpp, straight-line, scale-factor

## 2026-04-30 — Behavior tree gives up too easily

- **Commit**: `fdcd7f40` — 2026-04-30 — *"More tweaks to keep the robot from giving up"*
- **Fix**: Planner tolerance 0.5 → 2.0 m; recovery retries → 999; server timeout 20 → 60 s; added Spin to recovery.
- **Triage tip**: Robot abandons goals after a few attempts → bump retries / tolerance.
- **Keywords**: nav2, planner, tolerance, behavior-tree, bt_nav.xml, retries, number_of_retries, default_server_timeout, spin, recovery, nav2_paramsv2.yaml, goal-abandoned

## 2026-04-30 — BT plugin export macro missing

- **Commit**: `a5772dec` — 2026-04-30 — *"Bugfix for the BT plugin."*
- **Cause**: `goal_bender` library missing `BT_PLUGIN_EXPORT` define.
- **Fix**: `target_compile_definitions(autonav_goal_bender_bt_node PRIVATE BT_PLUGIN_EXPORT)`.
- **Triage tip**: "unknown node type" in `bt_navigator` → check the export define.
- **Keywords**: behavior-tree, bt, goal_bender, BT_PLUGIN_EXPORT, target_compile_definitions, CMakeLists.txt, custom_behavior_tree_plugins, unknown-node-type, plugin-load, bt_navigator

## 2026-04-30 — Goal behind robot stalls Nav2

- **Commit**: `4619a8dd` — 2026-04-30 — *"Reorganizing and adding a new behavior tree for if the goal is placed behind."*
- **Fix**: New `GoalBender` BT plugin inserts an intermediate waypoint when the goal is > 1.57 rad behind the robot.
- **Triage tip**: Robot just sits when goal is rear-quadrant → confirm GoalBender is registered in `bt_nav.xml`.
- **Keywords**: behavior-tree, bt, GoalBender, goal_bender, rear-goal, 1.57-rad, intermediate-waypoint, bt_nav.xml, custom_behavior_tree_plugins, stalled, nav2

## 2026-04-30 — Gradient-escape cost threshold type mismatch

- **Commit**: `6e098b8d` — 2026-04-30 — *"Bugfixing, NAV2 with the custom behavior, gradient escape had a type issue."*
- **Cause**: `cost_threshold` declared as int (127), used as double.
- **Fix**: Declared `127.0` everywhere; also fixed `default_bt_xml_filename` wiring in `run-nav2.sh`.
- **Triage tip**: GradientEscape silently doesn't run → check param type and the BT XML path.
- **Keywords**: behavior-tree, bt, gradient_escape, GradientEscape, cost_threshold, 127.0, type-mismatch, int-double, default_bt_xml_filename, run-nav2.sh, nav2_paramsv2.yaml, custom_behavior_tree_plugins

## 2026-04-30 — Map padder dynamic resolution

- **Commit**: `97c8323b` — 2026-04-30 — *"Fixing the resolution and using dynamic resolution for the map padder."*
- **Fix**: Distance-adaptive tile sizing (1 m near, 3 m far past 15 m); default output_resolution 0.10 m.
- **Triage tip**: Costmap memory huge → tune `near_radius_m` / `far_tile_size_m`.
- **Keywords**: map_padder, dynamic-resolution, tile_size_m, near_radius_m, far_tile_size_m, output_resolution, /map_padded, costmap-memory, distance-adaptive

## 2026-04-30 — Map padder seed-and-flood optimization

- **Commits**: `00c3bf74`, `c9f66d1f`, `fc886c68`, `5b7f77a7`, `6561aa11` — 2026-04-30 — *(several "padder" tweaks)*
- **Fix**: Replaced bounding-box approach with seed-and-flood from SLAM tiles + robot + goal + plan; everything else lethal. Pure single-resolution variant won out after the dual-res experiment.
- **Triage tip**: Padded map looks weird → check `tile_size_m`, `output_resolution`, and which seed sources are wired (goal_topic, plan_topic).
- **Keywords**: map_padder, seed-and-flood, /map, /map_padded, /goal_pose, /plan, tile_size_m, output_resolution, lethal, bounding-box, optimization

## 2026-04-30 — `min_speed_theta` too low, no in-place rotation

- **Commit**: `e7b38f56` — 2026-04-30 — *"Allowing rotation"*
- **Fix**: `min_speed_theta: 0.15 rad/s` so the controller actually rotates instead of drifting sideways.
- **Triage tip**: Robot drifts sideways at goal instead of turning → raise `min_speed_theta`.
- **Keywords**: nav2, controller, dwb, min_speed_theta, 0.15, rotation, in-place-turn, sideways-drift, nav2_paramsv2.yaml, goal-alignment

## 2026-04-30 — Dijkstra planner tried (then reverted)

- **Commit**: `91f72d74` — 2026-04-30 — *"Using the original Dijkstra planner."*
- **Cause**: Tested `use_astar: false` to fix planner oscillations.
- **Note**: Reverted on **2026-05-04** by `270b3d5a`.
- **Triage tip**: Planner timeouts → A* is back on; if it spreads too widely, tighten `tolerance` first.
- **Keywords**: nav2, planner, dijkstra, a-star, astar, use_astar, NavfnPlanner, nav2_paramsv2.yaml, planner-oscillation, revert

## 2026-05-01 — Polling rate mismatch (electrical / DAQ)

- **Commit**: `38a56a98` — 2026-05-01 — *"Leveling out rates in data collection"*
- **Fix**: Unified to 30 Hz across electrical publisher, DAQ launcher, and launch params.
- **Triage tip**: Electrical-data jitter or sample gaps → match `publish_rate` everywhere.
- **Keywords**: electrical, daq, polling-rate, 30hz, 10hz, publish_rate, autonav_electrical_publisher, autonav_automated_testing, sample-gaps, jitter, loop_rate_hz_

## 2026-05-04 — ZED wrapper pin saga (3 commits)

- **`a8f7f708`** — 2026-05-04 — *"Updated the zed wrapper"* — accidental bump to wrapper master (SDK 5.2 ABI). Build broke on the 5.1.x Jetson.
- **`51cd0917`** — 2026-05-04 — *"Pin zed-ros2-wrapper to v5.2.0 for SDK 5.1.x compatibility"* — re-pinned to the last v5.2.0 tag (SHA `506e047`).
- **`fb78812c`** — 2026-05-04 — *"Apply zed-ros2-wrapper v5.2.0 pin (mirrors fix/zed-ros2-wrapper-tag)"* — re-applied the pin and added 66 lines of `docs/zed.md` policy.
- **`dec0cfd7`** — 2026-05-04 — *"Document zed-ros2-wrapper submodule pin in docs/zed.md"* — formal documentation.
- **Triage tip**: `git submodule status isaac_ros-dev/src/zed-ros2-wrapper` should show **no** leading `+` or `-`. **Never** run `git submodule update --remote`.
- **Keywords**: zed, zed-ros2-wrapper, submodule, pin, v5.2.0, 506e047, sdk-5.1, sdk-5.2, abi-mismatch, .gitmodules, --remote, object_tracking_parameters, git-submodule, stereolabs, build-error, colcon

## 2026-05-04 — Reverted Dijkstra planner

- **Commit**: `270b3d5a` — 2026-05-04 — *"Revert 'Using the original Dijkstra planner.'"*
- **Fix**: Back to A* with tightened tolerance.
- **Triage tip**: Path-quality regression in this window → check planner config.
- **Keywords**: nav2, planner, dijkstra, a-star, astar, use_astar, revert, tolerance, nav2_paramsv2.yaml

## 2026-05-05 — `-Wpedantic` tripping NVCC stub files

- **Commit**: `e7e9f83f` — 2026-05-05 — *"Scope -Wpedantic to CXX only so NVCC's stub files don't trip it"*
- **Fix**: Generator-expression-scoped warning flags (`$<COMPILE_LANGUAGE:CXX>`).
- **Triage tip**: CUDA build flooded with "style of line directive is a GCC extension" → language-scope your warning flags.
- **Keywords**: cuda, nvcc, -Wpedantic, warnings, COMPILE_LANGUAGE, CXX, CMakeLists.txt, autonav_detection, cudafe1.cpp, stub-file, build-flag, generator-expression

## 2026-05-05 — Eigen Vector3f uninitialized in DBSCAN cell accumulator

- **Commit**: `1599bcf1` — 2026-05-05 — *"Raising the limit to help with PCA and fixing a bug with the grade detection"*
- **Cause**: Default-constructed `Eigen::Vector3f` left uninitialized; first point summed with garbage.
- **Fix**: Replaced `std::pair` with a struct that explicitly zero-inits.
- **Triage tip**: Obstacle clusters land at totally wrong locations → check Eigen accumulator init.
- **Keywords**: pca, grade-detector, dbscan, eigen, Vector3f, uninitialized, std::pair, CellAccum, pca_pipeline.cpp, autonav_detection, cluster-position, garbage-data

## 2026-05-05 — PCA z-gap split tripped by ring artifacts

- **Commit**: `aec4cd7d` — 2026-05-05 — *"Trying to fix the grade detection"*
- **Cause**: Ring-gap artifacts on tilted surfaces look like wall splits.
- **Fix**: Wall-height validation — reject splits where the upper cluster span < `wall_min_height` (0.5 m).
- **Triage tip**: Tilted ramps mis-classified as walls → check the wall-height test.
- **Keywords**: pca, grade-detector, ring-gap, wall, wall_min_height, ramp, z-gap-split, lidar-rings, tilted-surface, pca_pipeline.cpp, grade_detector.yaml

## 2026-05-05 — Planarity threshold too strict for real LiDAR noise

- **Commit**: `cdeea84d` — 2026-05-05 — *"Planarity threashold change"*
- **Fix**: `pca_planarity_max` 0.005 → 0.02. Sim-tuned values rejected real ramp cells.
- **Triage tip**: Ramps look "swiss-cheese" → planarity ratio.
- **Keywords**: pca, grade-detector, planarity, pca_planarity_max, eigenvalue, sim-vs-real, ramp, swiss-cheese, grade_detector.yaml, threshold

## 2026-05-05 — Cluster-size minimum dropping small ramps

- **Commit**: `24db50a0` — 2026-05-05 — *"Cluster size was making it hard to detect small ramps with steep grades"*
- **Fix**: Lowered `min_cluster_size` thresholds in `grade_detector.yaml`.
- **Triage tip**: Sim sees the ramp, hardware doesn't → DBSCAN cluster sizes.
- **Keywords**: pca, grade-detector, dbscan, min_cluster_size, cluster, small-ramp, sim-vs-real, grade_detector.yaml, ramp-detection

## 2026-05-05 — PCA detected the robot itself

- **Commit**: `21a2d8e5` — 2026-05-05 — *"Performance increasing edits and not detecting the robot for PCA"*
- **Fix**: `front_arc_only: true` (default) — drop points behind base_link.
- **Triage tip**: False obstacles behind the robot → confirm `front_arc_only` is on.
- **Keywords**: pca, grade-detector, front_arc_only, base_link, robot-self-detection, false-obstacle, dbscan, performance, grade_detector.yaml

## 2026-05-05 — Removed simulator-only surface-normal estimation

- **Commit**: `32754dff` — 2026-05-05 — *"No computing the lidar normal using a circle of PCA points, this was an artifact of simulation, we are not simulating."*
- **Cause**: The 15-sample disk-PCA found chassis clutter on real hardware (spurious ~38° tilt).
- **Fix**: Algorithm now relies on the static URDF rotation for ground normal — no per-frame discovery.
- **Triage tip**: Ramps reported at random angles → confirm the URDF lidar rotation is correct.
- **Keywords**: pca, grade-detector, surface-normal, surface_normal_samples, lidar-tilt, urdf, lidar_footprint, sim-artifact, disk-pca, chassis-clutter

## 2026-05-05 — 20 Hz steady-rate publishing scheduler for grade detector

- **Commit**: `117e9766` — 2026-05-05 — *"Publishing scheduler to ensure 20Hz rates"*
- **Fix**: Wall timer publishes from the latest cached result every 50 ms; stamps `now()`.
- **Triage tip**: Costmap update jitter → confirm `publish_rate_hz > 0` is set.
- **Keywords**: pca, grade-detector, publish_rate_hz, 20hz, scheduler, wall-timer, costmap-jitter, /scan_pca_filtered_points, pca_node.cpp

## 2026-05-05 — DBSCAN O(n²) replaced with grid-indexed neighbors

- **Commit**: `080034d1` — 2026-05-05 — *"Performance changes"*
- **Fix**: Grid-indexed DBSCAN; complexity O(n·w²), w ≈ 3 cells.
- **Triage tip**: Grade detector misses its 60 ms budget → profile for DBSCAN dominance.
- **Keywords**: pca, grade-detector, dbscan, performance, complexity, O(n²), grid-index, neighbors, 60ms-budget, pca_pipeline.cpp

## 2026-05-05 — Performance debugging instrumentation

- **Commit**: `a2c45cdf` — 2026-05-05 — *"Performance debugging"*
- **Fix**: Added timing probes; inlined hot-path checks.
- **Keywords**: pca, grade-detector, performance, profiling, timing-probe, computeTime, pca_node.cpp, latency-tracking

## 2026-05-05 — PCA vectorization + allocation pooling

- **Commit**: `49f48dd6` — 2026-05-05 — *"Performance improvements"*
- **Fix**: -O3 / vectorization for grade_detector; pre-sized Eigen pools to avoid reallocation.
- **Keywords**: pca, grade-detector, -O3, vectorization, eigen, allocation-pool, performance, CMakeLists.txt

## 2026-05-06 — Cross-stack startup race fixed

- **Commit**: `d20a1357` — 2026-05-06 — *"Fixing race issues with things starting without waiting and map not being published."*
- **Cause**: Devices started before topics existed; SLAM latched a "no data" state and never recovered.
- **Fix**: Introduced `wait_for_topic.py` + `gui_ready.sh`; each `run-*.sh` blocks on first message before emitting `[GUI_READY]`. SLAM gated on `/scan_fullframe` + `/map_padded`.
- **Triage tip**: Process running but topic never publishes → check `[GUI_READY]` did/didn't fire.
- **Keywords**: race-condition, startup-race, wait_for_topic.py, gui_ready.sh, [GUI_READY], slam_toolbox, no-data-latch, /scan_fullframe, /map_padded, run-script, sentinel, queue

## 2026-05-06 — `bringup` missing launch dependencies

- **Commit**: `1852fd84` — 2026-05-06 — *"Add launch, launch ros, and ament_index_python dependencies"*
- **Fix**: Added the three `<exec_depend>` lines to `package.xml`.
- **Triage tip**: Launch fails to import → check `package.xml` exec_depend list.
- **Keywords**: bringup, package.xml, exec_depend, launch, launch_ros, ament_index_python, import-error, missing-dependency, rosdep

## 2026-05-06 — Sensor frame rotations (2 commits)

- **`3fdcb16d`** — 2026-05-06 — *"Rotate lidar"* — corrected lidar joint RPY.
- **`90789bf8`** — 2026-05-06 — *"Rotate gps and caster"* — corrected GPS and caster swivel joint RPY.
- **Triage tip**: Point clouds appear rotated relative to TF → check `shogi.urdf` joint origins.
- **Keywords**: urdf, shogi.urdf, rpy, roll-pitch-yaw, lidar-rotation, gps-rotation, caster, frame-alignment, lidar_footprint, base_link, joint-origin

## 2026-05-06 — Replaced topic-wait with fixed 5 s pacing

- **Commit**: `1ebdae23` — 2026-05-06 — *"Replace topic-wait gating with fixed 5s queue pacing"*
- **Cause**: `wait_for_topic.py` deadlocks froze the queue when GPS or ZED were slow.
- **Fix**: Each `run-*.sh` and `slam.launch.py` does `sleep 5 && echo "[GUI_READY] <Label>"` so the queue advances on a fixed cadence.
- **Triage tip**: Queue stuck on a single device → check that `sleep 5` exists in the script.
- **Keywords**: 5s-pacing, sleep-5, [GUI_READY], wait_for_topic.py, queue-stuck, deadlock, run-zed.sh, run-lidar.sh, run-gps.sh, slam.launch.py, gui_ready_emit, hud_node.py

## 2026-05-06 — `run-detect.sh` was stalling the queue

- **Commit**: `f5940a96` — 2026-05-06 — *"Fixing detect sh stalling."*
- **Cause**: `run-detect.sh` was a bare `ros2 launch ...` with no `[GUI_READY]` sentinel; HUD waited 60 s for it.
- **Fix**: Wrapped with the same 5 s pattern + `echo "[GUI_READY] DETECT"`. Also fixed the stale `"LINE DETECT"` key in `_ready_timeouts` → `"DETECT"`.
- **Triage tip**: DETECT button stuck for ~60 s → confirm `run-detect.sh` has the `[GUI_READY]` line.
- **Keywords**: detect, DETECT, run-detect.sh, [GUI_READY], stalling, _ready_timeouts, LINE-DETECT, autonav_detection, sentinel, hud_node.py, button-stuck

## 2026-05-07 — Wheel joints declared `fixed`, not `continuous`

- **Commit**: `30ef2e18` — 2026-05-07 — *"fix(urdf): make wheel joints continuous in shogi model"*
- **Fix**: Changed Right/Left Wheel joints from `type="fixed"` to `type="continuous"`; added `<axis xyz="1 0 0" />`.
- **Triage tip**: Sim robot won't move → check shogi URDF joint types.
- **Keywords**: urdf, shogi.urdf, joint, type-fixed, type-continuous, wheel-joint, axis, sim, robot-immobile, mujoco, gazebo

## 2026-05-07 — `docs/sick.md` modernized

- **Commit**: `f3fdd450` — 2026-05-07 — *"Modernizing the SICK Lidar MD."*
- **Fix**: Documented the correct `udp_receiver_ip` arg name (old README had `udp_receiver_id`, which silently does nothing).
- **Triage tip**: Driver launches fine but UDP validation fails → check the arg name.
- **Keywords**: sick, lidar, udp_receiver_ip, udp_receiver_id, sick.md, sick_multiscan.launch, run-lidar.sh, doc, typo, silent-fail

## 2026-05-07 — `docs/zed.md` modernized

- **Commit**: `be226dc9` — 2026-05-07 — *"Modernizing the ZED Camera MD file"*
- **Fix**: Restructured with submodule-pin policy front and center.
- **Keywords**: zed, zed.md, doc, submodule-pin, v5.2.0, 506e047, modernize

## 2026-05-11 — SICK IMU yaw / covariance handling (historical; mechanism since superseded)

- **Original commit**: `7acefa48` — 2026-05-11 — *"Fixing IMU transform"*
- **Original cause (historical)**: The SICK MultiScan is mounted with a π-roll (180° around X), defined in the URDF as the fixed `base_link → lidar_footprint` joint. The driver publishes its onboard IMU directly in `lidar_footprint`, so the Z angular velocity (yaw rate) and Y angular velocity (pitch rate are negated relative to `base_link`. The original `ekf_local.yaml` fused `/sick_scansegment_xd/imu` directly — feeding the EKF an inverted yaw rate. Wheel-derived yaw fought IMU-derived yaw at every tick; `odom → base_link` drifted in yaw; slam_toolbox kept scan-match-correcting `map → odom` against a moving target, producing map-snap and stall behavior.
- **Original fix (now removed)**: A `imu_frame_transformer.py` node republished the IMU in `base_link` on `/sick_scansegment_xd/imu_base_link`.
- **Current mechanism (superseded)**: The frame-transformer was later replaced by the `imu_cov_inflator` package (Python, lives at `isaac_ros-dev/src/imu_cov_inflator/`). It republishes the SICK IMU as `/sick_scansegment_xd/imu_inflated` with inflated covariance so the EKF treats the rotated readings as a high-uncertainty input rather than relying on a pre-rotation. `ekf_local.yaml` consumes `imu0: /sick_scansegment_xd/imu_inflated` and disables the orientation/translation components it can't trust from the rotated mount (see the `imu0_config` row in that file). The net behavioral fix is the same — yaw is no longer fought between sensors — but the mechanism is covariance-based, not frame-rotation-based.
- **Triage tip**: If yaw tracking misbehaves, first confirm `imu_cov_inflator` is running and publishing on `/sick_scansegment_xd/imu_inflated` (`ros2 topic hz`). Then confirm `ekf_local.yaml`'s `imu0` is pointed at the inflated topic, **not** the raw one. The `imu_frame_transformer.py` no longer exists — do not re-add it without coordinating, since the inflator-based design assumes the frame mismatch is handled by trust-weighting rather than transformation.
- **Keywords**: imu, sick-imu, yaw, yaw-inverted, imu_cov_inflator, imu_inflated, ekf_local, ekf_local.yaml, lidar_footprint, base_link, sick_scansegment_xd, upside-down, lidar-mount, urdf, π-roll, covariance-inflation, superseded

## 2026-10-05 — Robot won't drive: SBS wait mode, crossed hall rows, reversed motor direction

- **Symptoms**: Roboteq red LED, `?FF=64` (MOSFET failure) latched at every power-up and returned after `%RESET`; same on both controllers. Later: wheels locked/stalled at 7–16 A with hall state not changing; after that fix, autonomous drove backward and Nav2 steering oscillated.
- **Cause 1**: The Roboteq Safety Brake Switches (SBS1xxyy) sat in Wait mode with motor phases shorted, so the Roboteq's boot MOSFET self-test failed. Their safety inputs (X2 pins 2/6) need 18–60 V from the 24 V terminal.
- **Cause 2**: Hall signal wires were in the wrong rows of the shared 10-pin hall connector, so each motor was commutated from the other motor's sensors. Fixed by swapping the signal groups between rows, then running Roborun+ Motor Sensor and Tuning Setup on both motors and saving (`%EESAV`).
- **Cause 3**: After that setup, a positive `!G 1` / negative `!G 2` drove the robot backward. Signs flipped in `MotorController::move()`; hall counts follow physical rotation, so odometry was unaffected.
- **Still required**: the Roboteq boots before the SBS units release, so it latches `FF=64` on every power-up — `%RESET 321654987` once the SBS CH1/CH2 LEDs are blue. Permanent fix (not done): SBS X2 pins 3/7 → Roboteq STO1/STO2 (pins 15/16) and remove the STO jumper.
- **Triage tip**: `?FF=64` with phases unplugged → controller. `FF=0` unplugged but 64 connected → SBS/wiring. `!STT` reports the MOSFET self-test (`5` = failed). Turn each wheel by hand and watch `?C` to see which channel its halls report on. If the Roboteq motor setup is ever re-run, re-check that sticks-up drives forward.
- **Keywords**: roboteq, FF=64, mosfet, sbs, safety-brake-switch, wait-mode, sto, hall, hall-connector, crossed-halls, motor-setup, roborun, HSAT=128, reversed, backward, motor-direction, motor_controller.cpp, eesav

---

# Keyword index

A flat alphabetical map of common search terms to entries that mention them. Use `Cmd-F` on this section to find which entries discuss a topic. AI assistants can use this as a fast index when fuzzy-matching a user's question to relevant log entries.

**Hardware / devices**
- `arduino` → 2025-12-03 USB enumeration, 2025-05-28 e-stop
- `gps`, `ublox`, `zed-f9p`, `cypress` → 2025-12-03 GPS path, 2026-04-07 BESTVELA, 2026-04-07 yaw jitter, 2026-04-17 NovAtel→u-blox, 2026-04-28 stoi crash
- `gradient_escape`, `goal_bender` → 2026-04-30 BT plugin export, 2026-04-30 goal behind, 2026-04-30 cost threshold
- `ina226`, `ina3221`, `i2c`, `power-pcb` → 2026-03-28 INA226 init retry, 2026-04-17 INA226 unbind, 2026-04-17 Jetson incident, 2026-05-01 polling rate
- `jetson`, `orin-nano` → 2026-04-17 incident, 2026-04-15 SSH X11 / software GL
- `lidar`, `sick`, `multiscan`, `eno1`, `192.168.0.x` → 2025-04-04 eth0→eno1, 2026-03-04 driver respawn, 2026-03-24 max_laserscan_range, 2026-03-24 demo-day eth, 2026-03-26 network automation, 2026-05-07 sick.md
- `motor`, `roboteq`, `encoder` → 2025-04-26 timeouts, 2025-11-15 parser, 2025-11-16 dup guard, 2025-12-03 USB order, 2026-04-04 null deref, 2026-04-15 axis swap, 2026-04-15 wheel radius, 2026-04-29 left over-count
- `xbox`, `joystick`, `estop`, `e-stop`, `ttyTHS1`, `b-button` → 2025-05-28 e-stop init, 2026-04-15 axis swap
- `zed`, `zed-ros2-wrapper`, `submodule`, `v5.2.0`, `506e047`, `sdk-5.1`, `sdk-5.2` → 2026-04-06 SDK CMake, 2026-04-07 calibration files, 2026-04-16 IMU rate, 2026-04-17 zed_override, 2026-05-04 wrapper saga, 2026-05-07 zed.md

**Topics / messages**
- `/autonomous_mode` → 2026-04-29 BT gating
- `/cloud_all_fields_fullframe` → 2026-04-25 obstacle layer
- `/encoders` → 2025-11-16 dup guard, 2026-04-04 null deref
- `/gps_fix` → 2026-04-17 NovAtel→u-blox, 2026-04-28 stoi crash
- `/line_points`, `/lidar_line_points`, `LinePoints`, `line-layer` → 2025-11-12 stale TF, 2026-03-21 ghost traces, 2026-03-24 line-hold, 2026-03-26 line-layer staleness, quick triage lidar-line stale cells
- `/map`, `/map_padded` → 2026-04-30 map padder dyn-res, 2026-04-30 seed-and-flood, 2026-05-06 startup race
- `/odom`, `/wheel_odom` → 2025-05-05 odom string, 2025-05-07 odom rename, 2026-04-15 wheel radius, 2026-04-28 odom jumps, 2026-04-29 left encoder
- `/scan`, `/scan_fullframe`, `/scan_pca_filtered_points` → 2025-05-07 odom rename, 2026-03-04 driver respawn, 2026-03-24 max_laserscan_range, 2026-04-25 obstacle layer, 2026-05-05 PCA scheduler, 2026-05-06 startup race
- `/sick_scansegment_xd/imu`, `/sick_scansegment_xd/imu_base_link` → 2026-05-11 IMU yaw inverted

**Subsystems / concepts**
- `behavior-tree`, `bt`, `bt_nav.xml` → 2026-03-24 hardcoded BT, 2026-04-29 BT gating, 2026-04-30 retries, 2026-04-30 BT plugin, 2026-04-30 goal-bender, 2026-04-30 gradient-escape
- `costmap`, `local-costmap`, `global-costmap`, `inflation`, `footprint`, `static-layer`, `rolling_window` → 2026-03-20 tilt, 2026-03-21 dimensions, 2026-03-21 ghost traces, 2026-04-15 footprint+inflation, 2026-04-22 rolling, 2026-04-22 geometry, 2026-04-25 PointCloud2 obstacles
- `cuda`, `nvcc`, `-Wpedantic`, `kernel`, `line-detection` → 2026-04-22 thresholds, 2026-04-22 window size, 2026-05-05 -Wpedantic
- `dds`, `fastdds`, `rmw`, `qos`, `discovery` → 2026-04-22 DDS fix, 2026-04-22 headless, 2026-04-24 QoS mismatch
- `docker`, `container`, `koopa-kingdom`, `entrypoint`, `dev-image`, `mppi` → 2026-04-06 user race, 2025-12-03 USB order, 2026-04-22 headless, 2026-04-22 NumPy, 2026-04-17 INA226 unbind, quick triage MPPI dependency
- `ekf`, `slam`, `slam_toolbox`, `tf`, `frame`, `urdf` → 2025-04-16 SLAM TF, 2025-11-12 stale TF, 2026-02-02 PUBLISH_TRANSFORM, 2026-03-24 double TF, 2026-05-06 frame rotations, 2026-05-07 wheel-joint continuous, 2026-05-11 IMU yaw inverted
- `imu`, `imu-frame`, `imu_frame_transformer`, `yaw-inverted`, `upside-down`, `π-roll`, `lidar_footprint` → 2026-04-16 ZED IMU rate, 2026-05-11 SICK IMU yaw inverted
- `gui`, `hud`, `hud_node.py`, `[GUI_READY]`, `dot`, `live-mode` → 2026-04-24 live-mode, 2026-04-24 playback, 2026-04-24 QoS, 2026-04-24 terminal, 2026-04-24 buttons, 2026-04-28 (5 entries), 2026-05-06 startup race, 2026-05-06 5s pacing, 2026-05-06 run-detect
- `map_padder` → 2026-04-30 dynamic-res, 2026-04-30 seed-and-flood
- `nav2`, `planner`, `dwb`, `mppi`, `smac`, `lattice`, `dijkstra`, `astar`, `tolerance` → 2026-04-15 footprint, 2026-04-15 critic, 2026-04-22 rolling, 2026-04-30 retries, 2026-04-30 dijkstra, 2026-05-04 dijkstra revert, quick triage MPPI dependency
- `pca`, `grade-detector`, `dbscan`, `eigen`, `planarity` → 2026-05-05 (8 entries)
- `x11`, `display`, `rviz`, `xauth`, `mit-shm` → 2026-04-08 MIT-SHM, 2026-04-15 xauth, 2026-04-15 SSH→GL, 2026-04-22 headless

**Symptoms / errors / phrases**
- `bricked`, `no-boot`, `no-ssh`, `force-recovery`, `fc-rec` → 2026-04-17 incident
- `colcon`, `build-error`, `object_tracking_parameters` → 2026-05-04 wrapper saga
- `costmap-tilted`, `costmap-inverted` → 2026-03-20 local costmap tilt, 2026-05-06 frame rotations
- `crash`, `null-deref`, `segfault` → 2026-04-04 null deref, 2026-04-22 numpy, 2026-04-28 stoi
- `discovery-broken`, `awaiting-live-data` → 2026-04-22 DDS, 2026-04-24 QoS mismatch
- `drift`, `right-drift`, `straight-line-drift` → 2026-04-29 left encoder
- `flicker`, `ghost-obstacle`, `stale` → 2026-03-21 ghost traces, 2026-03-24 line-hold, 2026-03-26 line-layer staleness
- `lag`, `slow`, `performance` → 2026-04-28 (multiple), 2026-05-05 (PCA performance entries)
- `not-publishing`, `topic-missing` → 2025-05-05 odom string, 2026-05-06 startup race
- `queue-stuck`, `button-yellow`, `dot-stuck` → 2026-05-06 5s pacing, 2026-05-06 run-detect, 2026-05-06 startup race
- `racing`, `race-condition`, `init-race` → 2025-11-16 dup guard, 2026-04-06 container user, 2026-05-06 startup race
- `tilt`, `costmap-tilt` → 2026-03-20 local costmap tilt
- `unknown-node-type` → 2026-04-30 BT plugin export

**Files / paths**
- `/boot/extlinux/extlinux.conf`, `.dtbo`, `device-tree` → 2026-04-17 incident
- `/dev/i2c-1`, `/dev/i2c-7`, `/sys/bus/i2c/drivers/ina3221/unbind` → 2026-04-17 INA226 unbind, 2026-04-17 incident
- `/dev/serial/by-id`, `/dev/ttyACM*`, `/dev/ttyUSB*`, `/dev/ttyTHS1` → 2025-05-28 e-stop, 2025-12-03 GPS path, 2025-12-03 USB order
- `.gitmodules` → 2026-05-04 wrapper saga
- `bt_nav.xml`, `bt2.xml` → 2026-03-24 BT path, 2026-04-30 retries, 2026-04-30 goal bender
- `core_bringup.launch.py`, `pre_slam.launch.py`, `sensors.launch.py`, `demo_day.launch.py`, `bringup.launch.py` → 2025-04-16 SLAM, 2026-03-24 staggered delays, 2026-03-24 demo-day eth, 2026-05-06 missing deps
- `ekf_local.yaml`, `slam.yaml`, `nav2_paramsv2.yaml` → many entries
- `env/docker/run-container.sh`, `entrypoint.sh`, `entrypoint_additions/*.sh`, `fastdds_udp.xml` → 2025-12-03 USB, 2026-04-06 user race, 2026-04-08 X11, 2026-04-15 X11 (multi), 2026-04-17 INA226 unbind, 2026-04-22 DDS, 2026-04-22 headless
- `grade_detector.yaml`, `line_detector.yaml`, `zed_override.yaml` → many detection / ZED entries
- `hud_node.py`, `run_gui.sh`, `run-gui.sh` → all 2026-04-24 / 2026-04-28 / 2026-05-06 GUI entries
- April 17 2026 Jetson incident report (SharePoint), `docs/zed.md`, `docs/sick.md`, `PACKAGES.md` → cross-references throughout
- `wheel_odom_pub.cpp`, `control.cpp`, `motor_controller.cpp`, `xbox.cpp` → control & odom entries
- `slam.launch.py`, `nav.launch.py`, `nav.launch.py`, `dual_ekf_navsat.launch.py` → 2026-03-24 BT path, 2026-04-30 (multiple), 2026-05-06 5s pacing, 2026-05-06 startup race

---

*Mined by 15 parallel research agents from the full git history (563 commits as of 2026-05-07). Reviewed by 10 parallel verification agents. Add new entries chronologically as you fix new issues — keep the format consistent (heading, commit/cause/fix/triage-tip, keywords list) so this stays grep-friendly and AI-searchable.*
