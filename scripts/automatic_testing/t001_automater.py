
#!/usr/bin/env python3
"""
t001_automater.py - GPS Calibration Test Automater

This script manages the automated GPS calibration test:
1. Creates a log file with test metadata
2. Enables data collection via /data/toggle_collect topic
3. Executes GPS waypoint square pattern test (10ft x 10ft)
4. Collects data from /data/dump topic during the test
5. Stops data collection when complete
6. Saves all data to a CSV log file

ARCHITECTURE:
- start_test(): Generic function that enables data collection (reusable for all tests)
- test_actions(): Test-specific function containing T001 GPS square pattern navigation
- stop_test(): Generic function that disables data collection and saves logs (reusable for all tests)

TEST PATTERN (T001):
Creates a 10ft x 10ft square pattern:
  WP0 (0,0) → WP1 (10,0) → WP2 (10,-10) → WP3 (0,-10) → WP0 (0,0)
  5-second pause at each waypoint except final return to start.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import csv
import os
import time
from datetime import datetime
from pathlib import Path


class T001Automater(Node):
    def __init__(self):
        super().__init__('t001_automater')
        
        # Test configuration
        self.test_id = 't001'
        self.test_name = 'GPS_Calibration'
        self.test_started = False
        self.test_complete = False
        self.estop_triggered = False
        self.test_actions_started = False
        
        # Data storage
        self.collected_data = []
        
        # Waypoint tracking
        self.waypoints = {}
        self.current_waypoint_index = 0
        self.waypoint_reached = False
        self.pause_end_time = None
        
        # Create log directory and file
        self.log_dir = Path('/autonav/logs')
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = self.log_dir / f'{self.test_id}_{timestamp}.csv'
        
        self.get_logger().info(f'Test log file: {self.log_file}')
        
        # Publishers
        self.toggle_pub = self.create_publisher(Bool, '/data/toggle_collect', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.data_sub = self.create_subscription(
            String, '/data/dump', self.data_callback, 10)
        self.estop_sub = self.create_subscription(
            String, '/estop', self.estop_callback, 10)
        
        # Timer to manage test flow
        self.timer = self.create_timer(1.0, self.test_manager)
        self.elapsed_time = 0
        
        # Initialize log file with header
        self.init_log_file()
        
        self.get_logger().info('T001 Automater initialized')

    def init_log_file(self):
        """Create log file with metadata and CSV header"""
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            # Metadata
            writer.writerow(['# Test ID:', self.test_id])
            writer.writerow(['# Test Name:', self.test_name])
            writer.writerow(['# Start Time:', datetime.now().isoformat()])
            writer.writerow([])  # Empty line
            # CSV Header
            writer.writerow([
                'timestamp', 'gps_lat', 'gps_lon', 'gps_alt',
                'odom_x', 'odom_y', 'odom_theta',
                'cmd_vel_linear', 'cmd_vel_angular',
                'encoder_data'
            ])

    def test_manager(self):
        """Manage test execution flow"""
        self.elapsed_time += 1
        
        if self.estop_triggered:
            self.get_logger().error('E-Stop triggered! Terminating test.')
            self.stop_test()
            return
        
        # Start test at t=2s (give nodes time to initialize)
        if self.elapsed_time == 2 and not self.test_started:
            self.start_test()
        
        # Start test-specific actions after test is started
        if self.test_started and not self.test_actions_started and self.elapsed_time == 5:
            self.test_actions_started = True
            self.test_actions()
        
        # Check for test completion conditions
        if self.elapsed_time > 600:  # 10 minute timeout
            self.get_logger().info('Test duration limit reached')
            self.stop_test()

    def start_test(self):
        """Start the GPS calibration test"""
        self.get_logger().info('Starting GPS Calibration Test')
        self.test_started = True
        
        # Enable data collection
        toggle_msg = Bool()
        toggle_msg.data = True
        self.toggle_pub.publish(toggle_msg)
        self.get_logger().info('Data collection enabled')
    
    def test_actions(self):
        """
        Test-specific actions for T001 GPS Calibration
        
        Creates a 10ft x 10ft square waypoint pattern and navigates through it:
        - Waypoint 0: (0, 0) - Starting position
        - Waypoint 1: (10, 0) - 10 feet forward
        - Waypoint 2: (10, -10) - 10 feet right from waypoint 1
        - Waypoint 3: (0, -10) - 10 feet back
        - Return to Waypoint 0: (0, 0) - Complete the square
        
        Robot pauses for 5 seconds at each waypoint except the final return to start.
        """
        self.get_logger().info('='*60)
        self.get_logger().info('Starting T001 GPS Calibration Test Actions')
        self.get_logger().info('Test Pattern: 10ft x 10ft square')
        self.get_logger().info('='*60)
        
        # Convert feet to meters (1 foot = 0.3048 meters)
        FEET_TO_METERS = 0.3048
        
        # Define waypoints in local coordinates (x, y in feet, converted to meters)
        # These are relative to the robot's starting position
        waypoint_coordinates = {
            0: (0.0, 0.0),           # Starting position
            1: (10.0, 0.0),          # 10 feet forward (North)
            2: (10.0, -10.0),        # 10 feet right (East)
            3: (0.0, -10.0),         # 10 feet back (South)
        }
        
        # Create PoseStamped messages for each waypoint
        for wp_id, (x_feet, y_feet) in waypoint_coordinates.items():
            # Convert feet to meters
            x_meters = x_feet * FEET_TO_METERS
            y_meters = y_feet * FEET_TO_METERS
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x_meters
            pose.pose.position.y = y_meters
            pose.pose.position.z = 0.0
            
            # Set orientation (facing forward)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            self.waypoints[wp_id] = pose
            self.get_logger().info(f'Waypoint {wp_id}: ({x_feet:.1f}ft, {y_feet:.1f}ft) = ({x_meters:.2f}m, {y_meters:.2f}m)')
        
        # Start navigation sequence
        self.navigate_waypoint_sequence()
    
    def navigate_waypoint_sequence(self):
        """
        Navigate through waypoints in sequence with pauses
        
        Sequence:
        1. Go to Waypoint 1, pause 5 seconds
        2. Go to Waypoint 2, pause 5 seconds
        3. Go to Waypoint 3, pause 5 seconds
        4. Return to Waypoint 0 (start), stop test
        """
        # Waypoint sequence: 1 -> 2 -> 3 -> 0
        sequence = [1, 2, 3, 0]
        pause_duration = 5  # seconds
        
        for i, wp_id in enumerate(sequence):
            if self.estop_triggered:
                self.get_logger().warn('E-Stop triggered during navigation!')
                break
            
            # Navigate to waypoint
            self.get_logger().info(f'--- Navigating to Waypoint {wp_id} ---')
            self.send_waypoint_goal(wp_id)
            
            # Wait for navigation to complete (simplified - in real implementation, 
            # you would subscribe to navigation status)
            # For now, estimate travel time based on distance and speed
            wait_time = self.estimate_travel_time(wp_id)
            self.get_logger().info(f'Estimated travel time: {wait_time} seconds')
            time.sleep(wait_time)
            
            # Check if we reached the final waypoint (return to start)
            if wp_id == 0:
                self.get_logger().info('Returned to starting position - Test Complete!')
                self.stop_test()
                break
            
            # Pause at waypoint (except at final position)
            self.get_logger().info(f'Reached Waypoint {wp_id} - Pausing for {pause_duration} seconds')
            time.sleep(pause_duration)
        
        self.get_logger().info('='*60)
        self.get_logger().info('GPS Calibration Test Actions Complete')
        self.get_logger().info('='*60)
    
    def send_waypoint_goal(self, waypoint_id):
        """Send a waypoint goal to the navigation system"""
        if waypoint_id not in self.waypoints:
            self.get_logger().error(f'Waypoint {waypoint_id} not found!')
            return
        
        pose = self.waypoints[waypoint_id]
        self.waypoint_pub.publish(pose)
        self.get_logger().info(f'Published goal: Waypoint {waypoint_id}')
    
    def estimate_travel_time(self, target_waypoint_id):
        """
        Estimate travel time to reach target waypoint
        
        Simple estimation based on distance and assumed robot speed
        Assumes average speed of ~0.5 m/s
        """
        if self.current_waypoint_index not in self.waypoints or target_waypoint_id not in self.waypoints:
            return 30  # Default 30 seconds if we can't estimate
        
        current_pose = self.waypoints[self.current_waypoint_index]
        target_pose = self.waypoints[target_waypoint_id]
        
        # Calculate distance
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        distance = (dx**2 + dy**2)**0.5
        
        # Estimate time (assuming 0.5 m/s average speed + 10 second buffer)
        avg_speed = 0.5  # m/s
        travel_time = (distance / avg_speed) + 10
        
        self.current_waypoint_index = target_waypoint_id
        
        return int(travel_time)

    def stop_test(self):
        """Stop the test and save data"""
        if self.test_complete:
            return
        
        self.get_logger().info('Stopping GPS Calibration Test')
        self.test_complete = True
        
        # Disable data collection
        toggle_msg = Bool()
        toggle_msg.data = False
        self.toggle_pub.publish(toggle_msg)
        self.get_logger().info('Data collection disabled')
        
        # Save collected data
        self.save_data()
        
        # Shutdown
        self.get_logger().info('Test complete. Log saved to: {}'.format(self.log_file))
        rclpy.shutdown()

    def data_callback(self, msg: String):
        """Collect data from /data/dump topic"""
        if self.test_started and not self.test_complete:
            self.collected_data.append(msg.data)

    def estop_callback(self, msg: String):
        """Handle emergency stop"""
        if msg.data == "STOP":
            self.get_logger().warn('E-Stop detected!')
            self.estop_triggered = True

    def save_data(self):
        """Save collected data to CSV file"""
        self.get_logger().info(f'Saving {len(self.collected_data)} data points')
        
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            for data_point in self.collected_data:
                # Data comes as CSV string, split and write
                values = data_point.split(',')
                writer.writerow(values)
        
        # Add end time metadata
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([])
            writer.writerow(['# End Time:', datetime.now().isoformat()])
            writer.writerow(['# Total Data Points:', len(self.collected_data)])
            writer.writerow(['# E-Stop Triggered:', self.estop_triggered])


def main(args=None):
    rclpy.init(args=args)
    
    try:
        automater = T001Automater()
        rclpy.spin(automater)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()