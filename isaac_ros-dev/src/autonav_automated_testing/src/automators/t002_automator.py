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

class T002Automator(BaseAutomator):
    def __init__(self):
        # Initialize base class with test-specific info
        super().__init__('t002_automater', 't002', 'Line_Comp')
        
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
        self.gps_online = True
        self.odom_online = False
        self.imu_online = False
        self.lidar_online = True
        self.encoder_online = False
        self.line_detection_online = False
        self.joy_online = False
        
        # Timeouts for sensor checks (seconds)
        self.sensor_check_start_time = self.get_clock().now()
        self.sensor_timeout = 60.0  # 30 seconds to get all sensors online
        # =================================== #
        
        # ===== Test Specific Publishers ===== #
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publish a one-time Joy toggle to enable autonomous mode in control node
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        # ==================================== #

        # ===== Test Specific Subscribers ===== #
        from sensor_msgs.msg import Imu, LaserScan
        from autonav_interfaces.msg import Encoders
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.encoder_sub = self.create_subscription(
            Encoders, '/encoders', self.encoder_callback, 10)
        # Listen to external /joy to start the test when A button is pressed
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        # Subscribe to cmd_vel to log movement commands directly
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # ===================================== #
        
        # Create a timer to check system status
        self.status_timer = self.create_timer(1.0, self.check_systems)
        
        self.get_logger().info('T002 Automator initialized - running system checks...')

    def check_systems(self):
        """Check if all required systems are online"""
        if self.systems_ready:
            return
        
        # Check elapsed time since start
        elapsed = (self.get_clock().now() - self.sensor_check_start_time).nanoseconds / 1e9
        
        # Print status every second
        self.get_logger().info('=== System Status Check ===')
        self.get_logger().info(f'GPS:            {"ONLINE" if self.gps_online else "OFFLINE"}')
        self.get_logger().info(f'Odometry:       {"ONLINE" if self.odom_online else "OFFLINE"}')
        self.get_logger().info(f'IMU:            {"ONLINE" if self.imu_online else "OFFLINE"}')
        self.get_logger().info(f'LiDAR:          {"ONLINE" if self.lidar_online else "OFFLINE"}')
        self.get_logger().info(f'Encoders:       {"ONLINE" if self.encoder_online else "OFFLINE"}')
        self.get_logger().info(f'Joystick:       {"ONLINE" if self.joy_online else "OFFLINE"}')
        self.get_logger().info(f'Elapsed: {elapsed:.1f}s / {self.sensor_timeout}s')
        self.get_logger().info('===========================')
        
        # Check if all systems are ready
        if (self.gps_online and self.odom_online and self.imu_online and 
            self.lidar_online and self.encoder_online and self.joy_online):
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

    # Sensor callback functions to mark systems as online
    def imu_callback(self, msg):
        """IMU data received - mark as online"""
        if not self.imu_online:
            self.imu_online = True
            self.get_logger().info('IMU came online')

    def lidar_callback(self, msg):
        """LiDAR data received - mark as online"""
        if not self.lidar_online:
            self.lidar_online = True
            self.get_logger().info('LiDAR came online')

    def encoder_callback(self, msg):
        """Encoder data received - mark as online"""
        if not self.encoder_online:
            self.encoder_online = True
            self.get_logger().info('Encoders came online')

    def test_manager(self):
        """Override base test manager to add distance checking"""
        # Call parent test manager for standard behavior
        super().test_manager()
        
        # Add test-specific completion condition: distance traveled
        if self.test_started and not self.test_complete:
            if (self.gps_distance_traveled >= self.target_distance_m or 
                self.odom_distance_traveled >= self.target_distance_m):
                self.get_logger().info(f'Target distance reached! GPS: {self.gps_distance_traveled:.2f}m, Odom: {self.odom_distance_traveled:.2f}m')
                self.stop_test()
                return

    def test_actions(self):
        self.get_logger().info('Starting line compliance test')
        self.line_following_active = True
        self.waiting_for_trigger = False
        
        if hasattr(self, 'waiting_timer'):
            self.waiting_timer.cancel()
        
        # Send X button to control FIRST
        self._send_x_button_to_control()
        
        # THEN enable data collection after a short delay
        def enable_data_collection():
            toggle_msg = Bool()
            toggle_msg.data = True
            self.toggle_pub.publish(toggle_msg)
            self.get_logger().info('Data collection enabled - robot is now moving')
        
        # Wait 1 second for control node to switch modes
        self.create_timer(1.0, enable_data_collection)
        
    def _send_x_button_to_control(self):
        """Send fake X button press to control node to trigger autonomous mode."""
        try:
            # X button is index 3 in control node
            X_BUTTON_INDEX = 3
            
            # Press X button
            press = Joy()
            press.buttons = [0]*8
            press.axes = [0.0]*4
            press.buttons[X_BUTTON_INDEX] = 1  # X button
            self.joy_pub.publish(press)
            self.get_logger().info('Sent fake X button press to control node')

            # Release after short delay to create rising edge
            def release_once():
                rel = Joy()
                rel.buttons = [0]*8
                rel.axes = [0.0]*4
                self.joy_pub.publish(rel)
                self.get_logger().info('Released X button - control node should be in autonomous mode')
                try:
                    release_timer.cancel()
                except Exception:
                    pass

            release_timer = self.create_timer(0.2, release_once)
            
        except Exception as e:
            self.get_logger().warn(f'Failed to send X button to control: {e}')

    def gps_callback(self, msg: NavSatFix):
        """Track GPS position for distance calculation"""
        # Mark GPS as online on first valid message
        if not self.gps_online and msg.status.status >= 0:
            self.gps_online = True
            self.get_logger().info('GPS came online')
        
        if msg.status.status >= 0:  # Valid GPS fix
            self.current_gps_position = msg
            
            if self.start_gps_position is None and self.test_started:
                self.start_gps_position = msg
                self.get_logger().info(f'GPS starting position set: {msg.latitude:.6f}, {msg.longitude:.6f}')
            elif self.start_gps_position is not None:
                # Calculate distance using Haversine formula
                self.gps_distance_traveled = self.calculate_gps_distance(
                    self.start_gps_position, self.current_gps_position)
                
                # Log progress every 10 meters
                if int(self.gps_distance_traveled) % 10 == 0 and int(self.gps_distance_traveled) > 0:
                    distance_ft = self.gps_distance_traveled / 0.3048
                    self.get_logger().info(f'GPS Distance: {self.gps_distance_traveled:.1f}m ({distance_ft:.1f}ft)')

    def odom_callback(self, msg: Odometry):
        """Track odometry position for distance calculation"""
        # Mark odometry as online on first message
        if not self.odom_online:
            self.odom_online = True
            self.get_logger().info('Odometry came online')
        
        self.current_odom_position = msg
        
        if self.start_odom_position is None and self.test_started:
            self.start_odom_position = msg
            self.get_logger().info('Odometry starting position set')
        elif self.start_odom_position is not None:
            # Calculate distance from odometry
            dx = msg.pose.pose.position.x - self.start_odom_position.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.start_odom_position.pose.pose.position.y
            self.odom_distance_traveled = math.sqrt(dx*dx + dy*dy)


    def calculate_gps_distance(self, start_pos: NavSatFix, current_pos: NavSatFix) -> float:
        """Calculate distance between two GPS positions using Haversine formula"""
        R = 6371000  # Earth's radius in meters
        
        lat1 = math.radians(start_pos.latitude)
        lat2 = math.radians(current_pos.latitude)
        dlat = math.radians(current_pos.latitude - start_pos.latitude)
        dlon = math.radians(current_pos.longitude - start_pos.longitude)
        
        a = (math.sin(dlat/2) * math.sin(dlat/2) + 
             math.cos(lat1) * math.cos(lat2) * 
             math.sin(dlon/2) * math.sin(dlon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c

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
        automator = T002Automator()
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