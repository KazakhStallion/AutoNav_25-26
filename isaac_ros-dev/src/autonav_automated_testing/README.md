# AutoNav Automated Testing System

An RQT-based plugin system for running automated tests on the AutoNav robot with comprehensive data logging.

## Overview

This package provides:
- **RQT Plugin**: GUI interface for launching automated tests
- **Data Publisher Node**: Collects sensor and navigation data during tests
- **Test Launch Files**: Pre-configured launch files for each test scenario
- **Automater Scripts**: Python scripts that manage test execution and data logging

## Architecture

### Components

1. **RQT Plugin** (`autonav_automated_testing_plugin.cpp`)
   - Provides a GUI within `rqt_gui` to launch tests
   - Displays test information and status
   - Can be extended to show real-time test progress

2. **Data Publisher Node** (`data_publisher.cpp`)
   - Subscribes to test-specific topics (GPS, odometry, cmd_vel, etc.)
   - Listens to `/data/toggle_collect` to start/stop collection
   - Publishes formatted data to `/data/dump`
   - Monitors `/estop` for emergency stops

3. **Test Launch Files** (`launch/t00X_*.launch.py`)
   - Starts all necessary nodes for a specific test
   - Configures data collection topics
   - Launches the automater script
   - Includes core bringup (sensors, control, navigation)

4. **Automater Scripts** (`scripts/automatic_testing/t00X_automater.py`)
   - Orchestrates test execution
   - Manages test timing and flow
   - Collects data from `/data/dump`
   - Generates CSV log files with results

## Test Workflow

```
User clicks test button in RQT
         ↓
Launch file starts:
  - Core robot nodes (bringup, sensors, control)
  - Test-specific nodes (GPS handler, waypoint navigator)
  - Data publisher node
  - Automater script
         ↓
Automater script:
  1. Creates log file
  2. Publishes TRUE to /data/toggle_collect
  3. Triggers test action (e.g., autonomous mode)
  4. Collects data from /data/dump
  5. Publishes FALSE to /data/toggle_collect
  6. Saves CSV log file
```

## Available Tests

### T001: GPS Calibration Test
**Purpose**: Validate GPS accuracy and waypoint navigation

**Launch**: `ros2 launch autonav_automated_testing t001_GPS_Cal.launch.py`

**Topics Monitored**:
- `/gps/fix` - GPS position data
- `/odom` - Odometry data
- `/tf` - Transform tree
- `/encoders` - Wheel encoder readings
- `/cmd_vel` - Commanded velocities

**Test Flow**:
1. Robot remains stationary to get GPS fix
2. GPS waypoints are loaded
3. Autonomous navigation to waypoints begins
4. Data is collected throughout journey
5. Test completes when waypoints are reached or timeout occurs

**Log Output**: `t001_YYYYMMDD_HHMMSS.csv`

### T002: Line Compliance Test
*(Template - to be implemented)*

### T003: Forward Movement Test
*(Template - to be implemented)*

## Usage

### From RQT GUI

1. Start the RQT GUI with the plugin:
   ```bash
   ros2 run rqt_gui rqt_gui
   ```

2. Load the plugin:
   - Plugins → AutoNav → Automated Testing

3. Click the test button you want to run

4. Monitor the terminal for test progress

5. Find log files in `/autonav/logs/`

### From Command Line

```bash
# Launch a specific test
ros2 launch autonav_automated_testing t001_GPS_Cal.launch.py

# View collected data
cat /autonav/logs/t001_*.csv
```

## Configuration

### Adding Topics to Monitor

Edit `config/testing_data_collection_setter.yaml`:

```yaml
data_publisher:
  ros__parameters:
    t001: ["/gps/fix", "/odom", "/tf", "/encoders", "/cmd_vel"]
    t002: ["/camera/image", "/line_detection/lines"]
    # Add more tests...
```

### Creating a New Test

### Quick Start
Use the provided template to create new tests quickly and consistently.

1. **Create automater script from template**:
   ```bash
   cd /autonav/isaac_ros-dev/../scripts/automatic_testing/
   cp t00X_automater_template.py t00X_automater.py
   ```
   
   Edit `t00X_automater.py`:
   - Update `test_id` and `test_name` in `__init__()`
   - Implement your test logic in `test_actions()`
   - **DO NOT MODIFY** `start_test()`, `stop_test()`, or `test_manager()` - these are generic

2. **Create launch file**: `launch/t00X_TestName.launch.py`
   - Copy `t001_GPS_Cal.launch.py` as a template
   - Modify nodes and parameters for your test
   - Update the automater script path

3. **Update topics**: Add topics to `config/testing_data_collection_setter.yaml`
   ```yaml
   data_publisher:
     ros__parameters:
       t00X: ["/topic1", "/topic2", "/topic3"]
   ```

4. **Add to plugin**: Edit `src/autonav_automated_testing_plugin.cpp`
   - Add test to `populateTests()` function

5. **Rebuild**:
   ```bash
   cd /autonav/isaac_ros-dev
   colcon build --packages-select autonav_automated_testing
   source install/setup.bash
   ```

### Automater Script Architecture

All automater scripts follow a consistent pattern:

- **`start_test()`**: Generic function that enables data collection
  - ✅ Reusable for all tests
  - ❌ DO NOT MODIFY

- **`test_actions()`**: Test-specific function containing your unique test logic
  - ✅ MODIFY THIS for each test
  - Contains waypoint navigation, sensor validation, movement patterns, etc.

- **`stop_test()`**: Generic function that disables data collection and saves logs
  - ✅ Reusable for all tests
  - ❌ DO NOT MODIFY

This architecture ensures consistency across all tests while allowing flexibility for test-specific behavior.

## Data Format

Log files are CSV format with metadata:

```csv
# Test ID:,t001
# Test Name:,GPS_Calibration
# Start Time:,2025-11-01T14:30:00
,
timestamp,gps_lat,gps_lon,gps_alt,odom_x,odom_y,odom_theta,cmd_vel_linear,cmd_vel_angular,encoder_data
1698852600.123,37.2311284,-80.4245910,650.5,0.0,0.0,0.0,0.0,0.0,0,0
1698852600.223,37.2311285,-80.4245911,650.5,0.1,0.0,0.0,0.5,0.0,5,5
...
# End Time:,2025-11-01T14:35:00
# Total Data Points:,3000
# E-Stop Triggered:,False
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/data/toggle_collect` | `std_msgs/Bool` | Enable/disable data collection |
| `/data/dump` | `std_msgs/String` | CSV-formatted data stream |
| `/estop` | `std_msgs/String` | Emergency stop status |
| `/autonomous_mode` | `std_msgs/Bool` | Trigger autonomous navigation |

## Building

```bash
cd /autonav/isaac_ros-dev
colcon build --packages-select autonav_automated_testing
source install/setup.bash
```

## Dependencies

- ROS2 Humble
- `rqt_gui` and `rqt_gui_cpp`
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`
- `gps_handler` package
- `gps_waypoint_handler` package
- `bringup` package

## Troubleshooting

**Plugin doesn't appear in RQT**
```bash
# Rebuild and source
colcon build --packages-select autonav_automated_testing
source install/setup.bash
# Clear RQT cache
rm -rf ~/.config/ros.org/rqt_gui.ini
```

**No data being collected**
- Check that `/data/toggle_collect` is being published to
- Verify topics exist: `ros2 topic list`
- Check data_publisher node is running: `ros2 node list`

**Test doesn't start**
- Ensure all dependent nodes are running
- Check launch file for errors
- Verify automater script has execute permissions

## License

Apache-2.0
