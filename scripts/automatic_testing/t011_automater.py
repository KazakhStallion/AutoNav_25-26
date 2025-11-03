#!/usr/bin/env python3
"""
t011_automater.py - Incline Performance Test Automater

This script manages the automated test:
1. Creates a log file with test metadata
2. Enables data collection via /data/toggle_collect topic
3. Executes the test
4. Collects data from /data/dump topic during the test
5. Stops data collection when complete
6. Saves all data to a CSV log file

ARCHITECTURE:
- start_test(): Generic function that enables data collection (reusable for all tests)
- test_actions(): Test-specific function containing triggers for test actions
- stop_test(): Generic function that disables data collection and saves logs (reusable for all tests)

TEST Briefing (T011):
- 
- 
- 
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

class T011Automater(Node):
    def __init__(self):
        super().__init__('t011_automater')
        
        # Test configuration
        self.test_id = 't011'
        self.test_name = 'Incline_Performance'
        self.test_started = False
        self.test_complete = False
        self.estop_triggered = False
        self.test_actions_started = False
        
        # Data storage
        self.collected_data = []

        # ===== Specific Variables related to executing the test_actions() ===== #

        # ====================================================================== #
        
        # Create log directory and file !!!(Change this later to store on Jetson)!!!
        # WHEN CHANGED, CHANGE FOR ALL AUTOMATER FILES
        self.log_dir = Path('/autonav/logs')
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = self.log_dir / f'{self.test_id}_{timestamp}.csv'
        
        self.get_logger().info(f'Test log file: {self.log_file}')
        
        # Publishers
        self.toggle_pub = self.create_publisher(Bool, '/data/toggle_collect', 10)

        # Subscribers
        self.data_sub = self.create_subscription(
            String, '/data/dump', self.data_callback, 10)
        self.estop_sub = self.create_subscription(
            String, '/estop', self.estop_callback, 10)
        
        # ===== Test Specific Publishers  ===== #
        # ===== Test Specific Publishers  ===== #

        # ===== Test Specific Subscribers ===== #
        # ===== Test Specific Subscribers ===== #
        
        # Timer to manage test flow and serves as watchdog
        self.timer = self.create_timer(1.0, self.test_manager)
        self.elapsed_time = 0
        self.timeout = 600 # When the test should terminate if left running too long [10 minutes]
        
        # Initialize log file with header
        self.init_log_file()
        
        self.get_logger().info(f'{self.test_id} Automater initialized')

    def init_log_file(self):
        """Create log file with metadata and CSV header"""
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            # Metadata
            writer.writerow(['# Test ID:', self.test_id])
            writer.writerow(['# Test Name:', self.test_name])
            writer.writerow(['# Start Time:', datetime.now().isoformat()])
            writer.writerow([])  # Empty line
            # CSV Header (Changes between tests)
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
        if self.elapsed_time > self.timeout:
            self.get_logger().info('Test duration limit reached')
            self.stop_test()

    def start_test(self):
        """Start the test"""
        self.get_logger().info(f'Starting {self.test_name} Test')
        self.test_started = True
        
        # Enable data collection
        toggle_msg = Bool()
        toggle_msg.data = True
        self.toggle_pub.publish(toggle_msg)
        self.get_logger().info('Data collection enabled')
    
    # ===== Everything within these lines are specific to the t011 test ===== #
    def test_actions(self):
        """
        Test-specific actions for T011 Incline Performance
        - 
        """
        pass
    # ======================================================================= #

    def stop_test(self):
        """Stop the test and save data"""
        if self.test_complete:
            return
        
        self.get_logger().info(f'Stopping {self.test_name} Test')
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
        automater = T011Automater()
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
