#!/usr/bin/env python3
"""
t007_automator.py - Speed Calibration Test Automator

This script implements a speed calibration test to generate empirical data for the
speedConverter() function in motor_controller.cpp. It incrementally increases motor 
CMD values from 0 to 1000 while recording actual robot speed calculated from encoder data.

IMPORTANT: This test requires the control node to subscribe to /motor/raw_cmd (Int32) topic
and directly apply the CMD value to motors WITHOUT going through speedConverter().

The control node should:
1. Subscribe to /motor/raw_cmd (std_msgs/Int32)
2. When received, call: motors.move(cmd_value, cmd_value) bypassing speedConverter
3. This allows us to collect raw CMD-to-speed data for calibration

CSV Output (Standard Format from base_automator):
- ROS2_Clock: ROS2 timestamp
- Topic_Name: /motor/linear_velocity or /motor/cmd_value
- Data_Keys: velocity_mph or cmd_value
- Value_0+: Corresponding data values

TEST FLOW:
1. Wait for velocity topic and joystick to come online
2. Press A button to start test
3. Slowly ramp CMD from 0 to 1000 (increment by 5 every 100ms)
4. Calculate speed from encoder changes  
5. Stop when speed reaches 6 MPH consistently (10 samples) or CMD reaches 1000
6. Save CSV data for analysis

USAGE:
After collecting data, use the CSV to fit coefficients A0, A1, A2 in:
  converted_speed_CMD = A0 * v * v + A1 * v + A2
Where v = desired_speed_mph * Kcal
"""

import rclpy
from base_automator import BaseAutomator
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32
import csv

class T007Automator(BaseAutomator):
    def __init__(self):
        # Initialize base class with test-specific info
        super().__init__('t007_automator', 't007', 'Speed_Calibration')
        
        # ===== Test-Specific Variables ===== #
        # Speed calibration variables
        self.current_cmd_value = 0  # Start at 0
        self.max_cmd_value = 1000  # Maximum motor command value
        self.cmd_increment = 5  # Increment by 5 per update (very smooth)
        self.target_speed_mph = 6.0  # Stop when we reach 6 MPH
        self.speed_stable_count = 0  # Count how many times we're at/above target speed
        self.speed_stable_threshold = 10  # Need 10 consecutive samples above 6 MPH
        
        # Track latest velocity data
        self.current_speed_mph = 0.0
        self.latest_velocity_received = False
        
        # Track latest CMD value from topic (not just what we publish)
        self.latest_cmd_from_topic = 0
        self.cmd_value_online = False
        
        # For Joy rising-edge detection (A button)
        self.A_BUTTON_INDEX = 0
        self.last_joy_buttons = None
        self.waiting_for_trigger = False
        self.systems_ready = False
        # =================================== #
        
        # ===== System Status Tracking ===== #
        self.joy_online = False
        self.velocity_online = False
        self.cmd_value_online = False
        
        # Timeouts for sensor checks (seconds)
        self.sensor_check_start_time = self.get_clock().now()
        self.sensor_timeout = 30.0  # 30 seconds to get all sensors online
        # =================================== #
        
        # ===== Test Specific Publishers ===== #
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_cmd_pub = self.create_publisher(Int32, '/motor/cmd_value', 10)
        # ==================================== #

        # ===== Test Specific Subscribers ===== #
        from std_msgs.msg import String
        
        self.velocity_sub = self.create_subscription(
            String, '/motor/linear_velocity', self.velocity_callback, 10)
        
        # Subscribe to /motor/cmd_value to get the actual published CMD values
        # This ensures we pair velocity with the CMD value at the same timestamp
        self.cmd_value_sub = self.create_subscription(
            Int32, '/motor/cmd_value', self.cmd_value_callback, 10)
        
        # Listen to external /joy to start the test when A button is pressed
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        # ===================================== #

        # Create a timer to check system status
        self.status_timer = self.create_timer(1.0, self.check_systems)
        
        # Create timer for slow CMD ramping (update every 100ms)
        self.cmd_ramp_timer = None
        
        self.get_logger().info('T007 Speed Calibration Automator initialized - running system checks...')

    def check_systems(self):
        """Check if all required systems are online"""
        if self.systems_ready:
            return
        
        # Check elapsed time since start
        elapsed = (self.get_clock().now() - self.sensor_check_start_time).nanoseconds / 1e9
        
        # Print status every second
        self.get_logger().info('=== System Status Check ===')
        self.get_logger().info(f'Velocity:       {"ONLINE" if self.velocity_online else "OFFLINE"}')
        self.get_logger().info(f'CMD Value:      {"ONLINE" if self.cmd_value_online else "OFFLINE"}')
        self.get_logger().info(f'Joystick:       {"ONLINE" if self.joy_online else "OFFLINE"}')
        self.get_logger().info(f'Elapsed: {elapsed:.1f}s / {self.sensor_timeout}s')
        self.get_logger().info('===========================')
        
        # Check if all systems are ready
        if self.velocity_online and self.cmd_value_online and self.joy_online:
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

    def print_waiting_message(self):
        """Periodically print waiting message until test starts"""
        if self.waiting_for_trigger and not self.test_started and self.systems_ready:
            self.get_logger().info("Awaiting test start trigger - Press 'A' button on joystick")

    # ===== Test Specific Callbacks ===== #
    def cmd_value_callback(self, msg):
        """CMD value received - store latest value"""
        if not self.cmd_value_online:
            self.cmd_value_online = True
            self.get_logger().info('Motor CMD value topic came online')
        
        # Store the latest CMD value that was actually published
        self.latest_cmd_from_topic = msg.data
    
    def velocity_callback(self, msg):
        """Velocity data received from control node"""
        if not self.velocity_online:
            self.velocity_online = True
            self.get_logger().info('Motor velocity topic came online')
        
        try:
            self.current_speed_mph = float(msg.data)
            self.latest_velocity_received = True
            
            # Append standardized rows for both velocity and CMD value during test
            # Use latest_cmd_from_topic to ensure we're pairing velocity with the
            # CMD value that was actually published (not just what we're about to publish)
            if self.test_started and not self.test_complete:
                # Use base_automator's append_standard_row for consistent CSV format
                self.append_standard_row('/motor/linear_velocity', [self.current_speed_mph])
                self.append_standard_row('/motor/cmd_value', [self.latest_cmd_from_topic])
        except ValueError:
            self.get_logger().warn(f'Invalid velocity data: {msg.data}')
    # =================================== #

    def test_manager(self):
        """Override base test manager to add speed checking"""
        # Call parent test manager for standard behavior
        super().test_manager()
        
        # Add test-specific completion condition: speed reached target
        if self.test_started and not self.test_complete:
            # Check if we've reached target speed consistently
            if self.current_speed_mph >= self.target_speed_mph:
                self.speed_stable_count += 1
                
                if self.speed_stable_count >= self.speed_stable_threshold:
                    self.get_logger().info(f'Target speed reached! Speed: {self.current_speed_mph:.2f} MPH at CMD: {self.current_cmd_value}')
                    self.stop_test()
                    return
            else:
                # Reset counter if speed drops below target
                self.speed_stable_count = 0
            
            # Also check if we've reached max CMD value
            if self.current_cmd_value >= self.max_cmd_value:
                self.get_logger().info(f'Maximum CMD value reached ({self.max_cmd_value}). Final speed: {self.current_speed_mph:.2f} MPH')
                self.stop_test()
                return

    def test_actions(self):
        """Start the speed calibration test"""
        self.get_logger().info('Starting speed calibration test')
        self.waiting_for_trigger = False
        
        if hasattr(self, 'waiting_timer'):
            self.waiting_timer.cancel()
        
        # Enable data collection immediately
        toggle_msg = Bool()
        toggle_msg.data = True
        self.toggle_pub.publish(toggle_msg)
        self.get_logger().info('Data collection enabled')
        
        # Start the CMD ramping timer (100ms updates for smooth acceleration)
        self.cmd_ramp_timer = self.create_timer(0.1, self.ramp_cmd_value)
        self.get_logger().info('CMD ramp started - robot will slowly accelerate')

    def ramp_cmd_value(self):
        """Slowly increment CMD value and send to motors"""
        if not self.test_started or self.test_complete:
            if self.cmd_ramp_timer:
                self.cmd_ramp_timer.cancel()
            return
        
        # Increment CMD value
        self.current_cmd_value += self.cmd_increment
        
        # Clamp to max value
        if self.current_cmd_value > self.max_cmd_value:
            self.current_cmd_value = self.max_cmd_value
        
        # Send raw CMD value to control node
        # The control node should subscribe to /motor/raw_cmd topic
        # and directly apply this value to motors without speed conversion
        cmd_msg = Int32()
        cmd_msg.data = self.current_cmd_value
        self.motor_cmd_pub.publish(cmd_msg)
        
        # Log progress every 50 CMD increments
        if self.current_cmd_value % 50 == 0:
            self.get_logger().info(f'CMD: {self.current_cmd_value}, Speed: {self.current_speed_mph:.2f} MPH')

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