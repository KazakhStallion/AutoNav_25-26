#!/usr/bin/env python3
"""
t001_automator.py - GPS Calibration Test Automator

This script implements the GPS calibration test with standardized CSV format:
- ROS2_Clock: ROS2 timestamp when data was received
- Topic_Name: The ROS2 topic the data came from  
- Data_Keys: Comma-separated list of data field names
- Data_Values: The actual data values (split into multiple columns)

TEST: GPS calibration and accuracy validation
Robot drives in a 3x3 meter square pattern:
- Waypoint 0: Starting position (current location)
- Waypoint 1: 3m North of Waypoint 0
- Waypoint 2: 3m East of Waypoint 1
- Waypoint 3: 3m South of Waypoint 2
- Return to Waypoint 0: 3m West of Waypoint 3

Sequence: 0 -> 1 (wait 5s) -> 2 (wait 5s) -> 3 (wait 5s) -> 0 (complete)
"""

import rclpy
from base_automator import BaseAutomator
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import math

class T001Automator(BaseAutomator):
    def __init__(self):
        # Initialize base class with test-specific info
        super().__init__('t001_automater', 't001', 'GPS_Cal')
        
        # ===== Test-Specific Variables ===== #
        # GPS waypoint navigation variables
        self.waypoints = []  # List of GPS waypoints (lat, lon)
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 0.5  # meters - how close to be "at" waypoint
        self.wait_time_at_waypoint = 5.0  # seconds to wait at each waypoint
        self.waiting_at_waypoint = False
        self.waypoint_wait_start = None
        self.square_size = 3.0  # meters - size of square pattern
        
        # Position tracking
        self.start_gps_position = None
        self.current_gps_position = None
        self.start_odom_position = None
        self.current_odom_position = None
        self.gps_distance_traveled = 0.0
        self.odom_distance_traveled = 0.0
        
        # For Joy rising-edge detection (A button)
        self.A_BUTTON_INDEX = 0
        self.last_joy_buttons = None
        self.waiting_for_trigger = False
        self.systems_ready = False
        # =================================== #
        
        # ===== System Status Tracking ===== #
        self.gps_online = False
        self.odom_online = False
        self.encoder_online = False
        self.joy_online = False
        
        # Timeouts for sensor checks (seconds)
        self.sensor_check_start_time = self.get_clock().now()
        self.sensor_timeout = 30.0  # 30 seconds to get all sensors online
        # =================================== #
        
        # ===== Test Specific Publishers ===== #
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publish a one-time Joy toggle to enable autonomous mode in control node
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        # Publish navigation goals to Nav2
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # ==================================== #

        # ===== Test Specific Subscribers ===== #
        from autonav_interfaces.msg import Encoders
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.encoder_sub = self.create_subscription(
            Encoders, '/encoders', self.encoder_callback, 10)
        # Listen to external /joy to start the test when A button is pressed
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        # ===================================== #
        
        # Create a timer to check system status
        self.status_timer = self.create_timer(1.0, self.check_systems)
        
        self.get_logger().info('T001 GPS Calibration Automator initialized - running system checks...')

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
        self.get_logger().info(f'Encoders:       {"ONLINE" if self.encoder_online else "OFFLINE"}')
        self.get_logger().info(f'Joystick:       {"ONLINE" if self.joy_online else "OFFLINE"}')
        self.get_logger().info(f'Elapsed: {elapsed:.1f}s / {self.sensor_timeout}s')
        self.get_logger().info('===========================')
        
        # Check if all systems are ready
        if (self.gps_online and self.odom_online and self.encoder_online and self.joy_online):
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
    def encoder_callback(self, msg):
        """Encoder data received - mark as online"""
        if not self.encoder_online:
            self.encoder_online = True
            self.get_logger().info('Encoders came online')

    def test_manager(self):
        """Override base test manager - add GPS waypoint navigation logic"""
        # Call parent test manager for standard behavior
        super().test_manager()
        
        # Waypoint navigation logic
        if self.test_started and not self.test_complete and len(self.waypoints) > 0:
            # Check if we're waiting at a waypoint
            if self.waiting_at_waypoint:
                elapsed = (self.get_clock().now() - self.waypoint_wait_start).nanoseconds / 1e9
                if elapsed >= self.wait_time_at_waypoint:
                    self.get_logger().info(f'Wait complete at waypoint {self.current_waypoint_index}')
                    self.waiting_at_waypoint = False
                    self.current_waypoint_index += 1
                    
                    # Check if we've completed the square
                    if self.current_waypoint_index >= len(self.waypoints):
                        self.get_logger().info('Square pattern complete! Returning to start.')
                        self.stop_test()
                        return
                    
                    # Publish goal for next waypoint
                    self.publish_waypoint_goal(self.current_waypoint_index)
            else:
                # Check if we've reached current waypoint
                if self.current_gps_position and self.current_waypoint_index < len(self.waypoints):
                    distance = self.calculate_gps_distance_to_point(
                        self.current_gps_position,
                        self.waypoints[self.current_waypoint_index]
                    )
                    
                    if distance <= self.waypoint_tolerance:
                        self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}! Distance: {distance:.2f}m')
                        self.waiting_at_waypoint = True
                        self.waypoint_wait_start = self.get_clock().now()
                        # Stop robot
                        stop_cmd = Twist()
                        self.cmd_vel_pub.publish(stop_cmd)

    def test_actions(self):
        """Start the GPS calibration test - setup waypoints and begin navigation"""
        self.get_logger().info('Starting GPS calibration test')
        self.waiting_for_trigger = False
        
        if hasattr(self, 'waiting_timer'):
            self.waiting_timer.cancel()
        
        # Setup waypoints based on current GPS position
        if self.current_gps_position:
            self.setup_waypoints()
            
            # Enable data collection
            toggle_msg = Bool()
            toggle_msg.data = True
            self.toggle_pub.publish(toggle_msg)
            self.get_logger().info('Data collection enabled')
            
            # Start navigation to first waypoint (waypoint 1)
            self.current_waypoint_index = 1
            self.publish_waypoint_goal(1)
        else:
            self.get_logger().error('No GPS position available - cannot setup waypoints!')
    
    def setup_waypoints(self):
        """Setup 3x3m square waypoints based on current GPS position"""
        # Waypoint 0 is current position
        start_lat = self.current_gps_position.latitude
        start_lon = self.current_gps_position.longitude
        
        # Calculate offsets in degrees for 3 meters
        # Approximate: 1 degree latitude ≈ 111,320 meters
        # Longitude varies by latitude: meters_per_degree = 111,320 * cos(lat)
        lat_offset = self.square_size / 111320.0
        lon_offset = self.square_size / (111320.0 * math.cos(math.radians(start_lat)))
        
        # Waypoint 0: Starting position
        self.waypoints.append((start_lat, start_lon))
        
        # Waypoint 1: 3m North (increase latitude)
        wp1_lat = start_lat + lat_offset
        wp1_lon = start_lon
        self.waypoints.append((wp1_lat, wp1_lon))
        
        # Waypoint 2: 3m East of Waypoint 1 (increase longitude)
        wp2_lat = wp1_lat
        wp2_lon = wp1_lon + lon_offset
        self.waypoints.append((wp2_lat, wp2_lon))
        
        # Waypoint 3: 3m South of Waypoint 2 (decrease latitude)
        wp3_lat = wp2_lat - lat_offset
        wp3_lon = wp2_lon
        self.waypoints.append((wp3_lat, wp3_lon))
        
        # Waypoint 0 again (return to start) - already in list
        
        self.get_logger().info(f'Waypoints setup:')
        for i, (lat, lon) in enumerate(self.waypoints):
            self.get_logger().info(f'  WP{i}: {lat:.6f}, {lon:.6f}')
    
    def publish_waypoint_goal(self, waypoint_index):
        """Publish navigation goal for given waypoint"""
        if waypoint_index >= len(self.waypoints):
            return
        
        lat, lon = self.waypoints[waypoint_index]
        self.get_logger().info(f'Publishing goal for waypoint {waypoint_index}: {lat:.6f}, {lon:.6f}')
        
        # Convert GPS coordinates to cartesian coordinates relative to starting position
        # Starting position (waypoint 0) is the origin in the map frame
        start_lat, start_lon = self.waypoints[0]
        x, y = self.gps_to_cartesian(start_lat, start_lon, lat, lon)
        
        # Create and publish PoseStamped message to Nav2
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        self.goal_pose_pub.publish(goal_pose)
        self.get_logger().info(f'Published goal pose: x={x:.2f}, y={y:.2f}')

    def gps_to_cartesian(self, ref_lat, ref_lon, target_lat, target_lon):
        """
        Convert GPS coordinates to cartesian coordinates relative to a reference point.
        
        Args:
            ref_lat: Reference latitude (origin)
            ref_lon: Reference longitude (origin)
            target_lat: Target latitude
            target_lon: Target longitude
            
        Returns:
            tuple: (x, y) in meters relative to reference point
        """
        # Earth's radius in meters
        R = 6371000
        
        # Convert to radians
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)
        target_lat_rad = math.radians(target_lat)
        target_lon_rad = math.radians(target_lon)
        
        # Calculate differences
        delta_lat = target_lat_rad - ref_lat_rad
        delta_lon = target_lon_rad - ref_lon_rad
        
        # Y direction (latitude/North) distance in meters
        y = delta_lat * R
        
        # X direction (longitude/East) distance in meters, adjusted for latitude
        x = delta_lon * R * math.cos(ref_lat_rad)
        
        return x, y

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
    
    def calculate_gps_distance_to_point(self, current_pos: NavSatFix, target_point: tuple) -> float:
        """Calculate distance from current GPS position to a target lat/lon point"""
        R = 6371000  # Earth's radius in meters
        
        lat1 = math.radians(current_pos.latitude)
        lat2 = math.radians(target_point[0])
        dlat = math.radians(target_point[0] - current_pos.latitude)
        dlon = math.radians(target_point[1] - current_pos.longitude)
        
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
        automator = T001Automator()
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
