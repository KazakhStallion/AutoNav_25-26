#!/usr/bin/env python3
"""
base_automater.py - Base class for all automated test scripts

This provides common functionality for:
- Standardized CSV data format
- Data parsing and formatting
- Common ROS2 interfaces
- Shared utility functions

All test automater scripts should inherit from this base class.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import csv
import os
from datetime import datetime
from pathlib import Path

class BaseAutomater(Node):
    def __init__(self, node_name: str, test_id: str, test_name: str):
        super().__init__(node_name)
        
        # Test configuration
        self.test_id = test_id
        self.test_name = test_name
        self.test_started = False
        self.test_complete = False
        self.estop_triggered = False
        self.test_actions_started = False
        
        # Data storage
        self.collected_data = []
        
        # Create log directory and file
        self.log_dir = Path('/autonav/logs')
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = self.log_dir / f'{self.test_id}_{timestamp}.csv'
        
        self.get_logger().info(f'Test log file: {self.log_file}')
        
        # Common Publishers
        self.toggle_pub = self.create_publisher(Bool, '/data/toggle_collect', 10)

        # Common Subscribers
        self.data_sub = self.create_subscription(
            String, '/data/dump', self.data_callback, 10)
        self.estop_sub = self.create_subscription(
            String, '/estop', self.estop_callback, 10)
        
        # Timer to manage test flow and serves as watchdog
        self.timer = self.create_timer(1.0, self.test_manager)
        self.elapsed_time = 0
        self.timeout = 600  # 10 minutes default timeout
        
        # Initialize log file with header
        self.init_log_file()
        
        self.get_logger().info(f'{self.test_id} Automater initialized')

    def init_log_file(self):
        """Create log file with standardized CSV header only"""
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            # Standardized CSV Header for all tests (no metadata)
            writer.writerow(['ROS2_Clock', 'Topic_Name', 'Data_Keys', 'Data_Values'])

    def test_manager(self):
        """Base test manager - override in child classes for specific behavior"""
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
        """Start the test - common for all tests"""
        self.get_logger().info(f'Starting {self.test_name} Test')
        self.test_started = True
        
        # Enable data collection
        toggle_msg = Bool()
        toggle_msg.data = True
        self.toggle_pub.publish(toggle_msg)
        self.get_logger().info('Data collection enabled')

    def test_actions(self):
        """Test-specific actions - must be overridden in child classes"""
        raise NotImplementedError("test_actions() must be implemented in child class")

    def stop_test(self):
        """Stop the test and save data - common for all tests"""
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
        """Collect and parse data from /data/dump topic"""
        if self.test_started and not self.test_complete:
            # Parse the incoming data string
            parsed_data = self.parse_data_dump(msg.data)
            if parsed_data:
                self.collected_data.extend(parsed_data)

    def parse_data_dump(self, data_string: str):
        """
        Parse the data dump string and convert to standardized format
        Expected format: "timestamp,topic_name,data_type,data_values"
        Returns list of formatted rows for CSV
        """
        try:
            # Get current ROS time
            current_time = self.get_clock().now()
            ros_timestamp = current_time.nanoseconds
            
            # Split the incoming data
            parts = data_string.strip().split(',')
            if len(parts) < 3:
                return None
                
            topic_name = parts[0] if parts[0].startswith('/') else f"/{parts[0]}"
            data_type = parts[1]
            data_values = parts[2:]
            
            # Format based on topic type
            formatted_rows = []
            
            if topic_name == "/gps/fix":
                if len(data_values) >= 3:
                    formatted_rows.append([
                        ros_timestamp, topic_name, "latitude,longitude,altitude",
                        data_values[0], data_values[1], data_values[2]
                    ])
            elif topic_name == "/encoders":
                if len(data_values) >= 2:
                    formatted_rows.append([
                        ros_timestamp, topic_name, "encoder_left,encoder_right",
                        data_values[0], data_values[1]
                    ])
            elif topic_name == "/odom":
                if len(data_values) >= 6:
                    formatted_rows.append([
                        ros_timestamp, topic_name, "pos_x,pos_y,pos_z,orient_x,orient_y,orient_z",
                        data_values[0], data_values[1], data_values[2],
                        data_values[3], data_values[4], data_values[5]
                    ])
            elif topic_name == "/cmd_vel":
                if len(data_values) >= 2:
                    formatted_rows.append([
                        ros_timestamp, topic_name, "linear_x,angular_z",
                        data_values[0], data_values[1]
                    ])
            elif topic_name == "/imu/data":
                if len(data_values) >= 9:
                    formatted_rows.append([
                        ros_timestamp, topic_name, "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,orient_x,orient_y,orient_z",
                        data_values[0], data_values[1], data_values[2],
                        data_values[3], data_values[4], data_values[5],
                        data_values[6], data_values[7], data_values[8]
                    ])
            elif topic_name == "/scan":
                # LiDAR data - summarize range data
                if len(data_values) >= 3:
                    formatted_rows.append([
                        ros_timestamp, topic_name, "range_min,range_max,ranges_count",
                        data_values[0], data_values[1], data_values[2]
                    ])
            elif topic_name == "/line_detection/lines":
                # Line detection data
                if len(data_values) >= 1:
                    formatted_rows.append([
                        ros_timestamp, topic_name, "lines_detected",
                        data_values[0]
                    ])
            else:
                # Generic format for unknown topics
                keys = ",".join([f"value_{i}" for i in range(len(data_values))])
                row = [ros_timestamp, topic_name, keys] + data_values
                formatted_rows.append(row)
                
            return formatted_rows
            
        except Exception as e:
            self.get_logger().warn(f'Error parsing data dump: {e}')
            return None

    def estop_callback(self, msg: String):
        """Handle emergency stop"""
        if msg.data == "STOP":
            self.get_logger().warn('E-Stop detected!')
            self.estop_triggered = True

    def save_data(self):
        """Save collected data to CSV file in standardized format"""
        self.get_logger().info(f'Saving {len(self.collected_data)} data points')
        
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            for data_row in self.collected_data:
                # Data is already formatted as list from parse_data_dump
                writer.writerow(data_row)