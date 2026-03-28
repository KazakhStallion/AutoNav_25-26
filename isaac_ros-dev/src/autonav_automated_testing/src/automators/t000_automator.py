#!/usr/bin/env python3
"""
t000_automator.py - DAQ Mode Test Automator

Objective:
- Provide a minimal, operator-driven data acquisition test.
- Start/Stop data collection via joystick A button toggle.

Behavior:
- Do no driving automatically, collect data while operator drives robot.
"""

import rclpy
from base_automator import BaseAutomator
from sensor_msgs.msg import Joy, NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class T000Automator(BaseAutomator):
    def __init__(self):
        super().__init__('t000_automater', 't000', 'DAQ_MODE')

        # State
        self.odom_online = False
        self.joy_online = False
        self.gps_online = False
        self.imu_online = False
        self.systems_ready = False
        self.waiting_for_trigger = False
        self.A_BUTTON_INDEX = 0
        self.last_joy_buttons = None

        # Publishers
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps_fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)

        # Timers
        self.status_timer = self.create_timer(1.0, self.check_systems)
        self.sensor_check_start_time = self.get_clock().now()
        self.sensor_timeout = 30.0

        self.get_logger().info('T000 Automator initialized - DAQ mode system checks...')

    # ===== System Checks ===== #
    def check_systems(self):
        if self.systems_ready:
            return
        elapsed = (self.get_clock().now() - self.sensor_check_start_time).nanoseconds / 1e9
        self.get_logger().info('=== DAQ Mode Status Check ===')
        self.get_logger().info(f'Odometry:   {"ONLINE" if self.odom_online else "OFFLINE"}')
        self.get_logger().info(f'Joystick:   {"ONLINE" if self.joy_online else "OFFLINE"}')
        self.get_logger().info(f'GPS:        {"ONLINE" if self.gps_online else "OFFLINE"}')
        self.get_logger().info(f'IMU:        {"ONLINE" if self.imu_online else "OFFLINE"}')
        self.get_logger().info(f'Elapsed: {elapsed:.1f}s / {self.sensor_timeout}s')
        self.get_logger().info('================================')
        if self.odom_online and self.joy_online and self.gps_online and self.imu_online:
            self.systems_ready = True
            self.status_timer.cancel()
            self.get_logger().info('\n' + '!'*50)
            self.get_logger().info('!!!DAQ MODE READY — Press A to start!!!')
            self.get_logger().info('!'*50 + '\n')
            self.waiting_for_trigger = True
            self.waiting_timer = self.create_timer(2.0, self.print_waiting_message)
            return
        if elapsed > self.sensor_timeout:
            self.get_logger().error('Sensor timeout — odom/joy/gps/imu not online.')
            self.status_timer.cancel()

    def print_waiting_message(self):
        if self.waiting_for_trigger and not self.test_started and self.systems_ready:
            self.get_logger().info("Awaiting DAQ start — Press 'A'")

    # ===== Callbacks ===== #
    def odom_callback(self, msg: Odometry):
        if not self.odom_online:
            self.odom_online = True
            self.get_logger().info('Odometry online')

    def gps_callback(self, msg):
        if not self.gps_online:
            self.gps_online = True
            self.get_logger().info('GPS online')

    def imu_callback(self, msg):
        if not self.imu_online:
            self.imu_online = True
            self.get_logger().info('IMU online')

    def joy_callback(self, msg: Joy):
        if not self.joy_online:
            self.joy_online = True
            self.get_logger().info('Joystick online')
        if not hasattr(msg, 'buttons') or len(msg.buttons) <= self.A_BUTTON_INDEX:
            self.last_joy_buttons = list(msg.buttons) if hasattr(msg, 'buttons') else None
            return
        curr = list(msg.buttons)
        if self.last_joy_buttons is not None:
            prev = self.last_joy_buttons
            prev_val = prev[self.A_BUTTON_INDEX] if len(prev) > self.A_BUTTON_INDEX else 0
            curr_val = curr[self.A_BUTTON_INDEX]
            if curr_val == 1 and prev_val == 0:
                # Rising edge on A button
                if not self.test_started and not self.test_complete and self.systems_ready:
                    self.get_logger().info('A pressed — starting DAQ mode')
                    try:
                        self.start_test()
                    except Exception as e:
                        self.get_logger().warn(f'Failed to start DAQ: {e}')
                elif self.test_started and not self.test_complete:
                    self.get_logger().info('A pressed again — stopping DAQ mode')
                    try:
                        self.stop_test()
                    except Exception as e:
                        self.get_logger().warn(f'Failed to stop DAQ: {e}')
        self.last_joy_buttons = curr

    # ===== Test Lifecycle ===== #
    def test_actions(self):
        # No autonomous actions; data collection only under operator control
        self.get_logger().info('DAQ Mode active — operator controls everything. Press A again to stop.')


def main(args=None):
    rclpy.init(args=args)
    automator = None
    try:
        automator = T000Automator()
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
