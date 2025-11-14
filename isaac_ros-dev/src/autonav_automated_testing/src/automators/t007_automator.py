#!/usr/bin/env python3
"""
t002_automator.py - Line Compliance Test Automator (Standardized Version)

This script implements the line compliance test with the new standardized CSV format:
- ROS2_Clock: ROS2 timestamp when data was received
- Topic_Name: The ROS2 topic the data came from  
- Data_Keys: Comma-separated list of data field names
- Data_Values: The actual data values (split into multiple columns)

TEST: Robot follows white lines for 110 feet while monitoring sensors
"""

import rclpy
from base_automator import BaseAutomator
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import math
import csv

class T007Automator(BaseAutomator):
    def __init__(self):
        # Initialize base class with test-specific info
        super().__init__('t002_automater', 't007', 'Line_Comp')
        
        # ===== Test-Specific Variables ===== #
        # Distance tracking variables
        self.start_gps_position = None
        self.current_gps_position = None
        self.start_odom_position = None
        self.current_odom_position = None
        self.gps_distance_traveled = 0.0
        self.odom_distance_traveled = 0.0
        self.target_distance_ft = 110.0  # 110 feet target
        self.target_distance_m = self.target_distance_ft * 0.3048  # Convert to meters
        self.line_following_active = False
        # For Joy rising-edge detection (A button)
        self.A_BUTTON_INDEX = 0
        self.last_joy_buttons = None
        self.waiting_for_trigger = False  # Start as False until systems ready
        self.systems_ready = False  # Flag for pre-flight checks
        # =================================== #
        
        # ===== System Status Tracking ===== #
        self.encoder_online = False
        
        # Timeouts for sensor checks (seconds)
        self.sensor_check_start_time = self.get_clock().now()
        self.sensor_timeout = 30.0  # 30 seconds to get all sensors online
        # =================================== #
        
        # ===== Test Specific Publishers ===== #
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # ==================================== #

        # ===== Test Specific Subscribers ===== #
        from autonav_interfaces.msg import Encoders
        self.encoder_sub = self.create_subscription(
            Encoders, '/encoders', self.encoder_callback, 10)
        # ===================================== #

        # Need to be able to publish to Joy for A button to start test
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Listen to external /joy to start the test when A button is pressed
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        # Create a timer to check system status
        self.status_timer = self.create_timer(1.0, self.check_systems)
        
        self.get_logger().info('T007 Automator initialized - running system checks...')

    def check_systems(self):
        """Check if all required systems are online"""
        if self.systems_ready:
            return
        
        # Check elapsed time since start
        elapsed = (self.get_clock().now() - self.sensor_check_start_time).nanoseconds / 1e9
        
        # Print status every second
        self.get_logger().info('=== System Status Check ===')
        self.get_logger().info(f'Encoders:       {"ONLINE" if self.encoder_online else "OFFLINE"}')
        self.get_logger().info('===========================')
        
        # Check if all systems are ready
        if (self.encoder_online):
            self.systems_ready = True
            self.status_timer.cancel()  # Stop checking
            self.get_logger().info('')
            self.get_logger().info('!' * 50)
            self.get_logger().info('!!!ALL SYSTEMS READY!!!')
            self.get_logger().info('!' * 50)
            self.get_logger().info('')
            
            # Now start waiting for trigger
            self.waiting_for_trigger = True
            self.waiting_timer = self.create_timer(2.0, self.print_waiting_message)
            return
        
        # Check timeout
        if elapsed > self.sensor_timeout:
            self.get_logger().error('Sensor timeout! Not all systems came online.')
            self.get_logger().error('Missing systems - cannot proceed with test.')
            self.status_timer.cancel()
            # Don't shutdown, just stop checking - operator can troubleshoot

    def print_waiting_message(self):
        """Periodically print waiting message until test starts"""
        if self.waiting_for_trigger and not self.test_started and self.systems_ready:
            self.get_logger().info("Awaiting test start trigger - Press 'A' button on joystick")

    # ===== Test Specific Callbacks ===== #
    # Sensor callback functions to mark systems as online
    def encoder_callback(self, msg):
        """Encoder data received - mark as online"""
        if not self.encoder_online:
            self.encoder_online = True
            self.get_logger().info('Encoders came online')
    # =================================== #

    def test_manager(self):
        """Override base test manager to add distance checking"""
        # Call parent test manager for standard behavior
        super().test_manager()
        
        # Add test-specific completion condition: distance traveled
        if self.test_started and not self.test_complete:
            print("TODO")
            # Specific conditional to automatically detect when the test should end.

    def test_actions(self):
        self.get_logger().info('Starting speed calibration test')
        self.line_following_active = True
        self.waiting_for_trigger = False
        
        if hasattr(self, 'waiting_timer'):
            self.waiting_timer.cancel()
        
        # THEN enable data collection after a short delay
        def enable_data_collection():
            toggle_msg = Bool()
            toggle_msg.data = True
            self.toggle_pub.publish(toggle_msg)
            self.get_logger().info('Data collection enabled - robot is now moving')
        
        # Wait 1 second for control node to switch modes
        self.create_timer(1.0, enable_data_collection)

    def joy_callback(self, msg: Joy):
        """Start the test on an external Joy A button rising edge (buttons[0])"""
        try:
            # Mark joystick as online on first message
            if not self.joy_online:
                self.joy_online = True
                self.get_logger().info('Joystick came online')
            
            if not hasattr(msg, 'buttons') or len(msg.buttons) <= self.A_BUTTON_INDEX:
                # Can't detect A button, just store and exit
                self.last_joy_buttons = list(msg.buttons) if hasattr(msg, 'buttons') else None
                return

            curr_buttons = list(msg.buttons)

            # If we have a previous sample, check for rising edge on A button index
            if self.last_joy_buttons is not None:
                prev = self.last_joy_buttons
                prev_val = prev[self.A_BUTTON_INDEX] if len(prev) > self.A_BUTTON_INDEX else 0
                curr_val = curr_buttons[self.A_BUTTON_INDEX]
                if curr_val == 1 and prev_val == 0:
                    # Rising edge detected on A button
                    if not self.test_started and not self.test_complete and self.systems_ready:
                        self.get_logger().info('Joy A rising edge detected — starting test')
                        try:
                            self.start_test()
                        except Exception as e:
                            self.get_logger().warn(f'Failed to start test from Joy input: {e}')
                    elif not self.systems_ready:
                        self.get_logger().warn('Cannot start test - systems not ready yet!')

            # Save the latest buttons state for future edge detection
            self.last_joy_buttons = curr_buttons
        except Exception as e:
            self.get_logger().warn(f'Error in joy_callback: {e}')
    # ======================================================================= #

def main(args=None):
    rclpy.init(args=args)
    automator = None
    
    try:
        automator = T007Automator()
        rclpy.spin(automator)
    except KeyboardInterrupt:
        print('\n[INFO] Keyboard interrupt detected (Ctrl+C)')
        if automator is not None:
            print('[INFO] Saving collected data before shutdown...')
            try:
                automator.save_data()
                print(f'[INFO] Data saved to: {automator.log_file}')
            except Exception as e:
                print(f'[ERROR] Failed to save data: {e}')
    except Exception as e:
        print(f'[ERROR] Unexpected error: {e}')
        if automator is not None:
            try:
                automator.save_data()
                print(f'[INFO] Data saved to: {automator.log_file}')
            except:
                pass
    finally:
        if automator is not None:
            try:
                automator.destroy_node()
            except:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()