#!/usr/bin/env python3
"""
t002_automater_new.py - Line Compliance Test Automater (Standardized Version)

This script implements the line compliance test with the new standardized CSV format:
- ROS2_Clock: ROS2 timestamp when data was received
- Topic_Name: The ROS2 topic the data came from  
- Data_Keys: Comma-separated list of data field names
- Data_Values: The actual data values (split into multiple columns)

TEST: Robot follows white lines for 110 feet while monitoring sensors
"""

import rclpy
from base_automater import BaseAutomater
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import math
import csv

class T002Automater(BaseAutomater):
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
        # =================================== #
        
        # ===== Test Specific Publishers ===== #
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # ==================================== #

        # ===== Test Specific Subscribers ===== #
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        # ===================================== #

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

    # ===== Everything within these lines are specific to the t002 test ===== #
    def test_actions(self):
        """
        Test-specific actions for T002 Line Compliance
        - Start line following behavior
        - Robot should stay between white lines using camera input
        - Continue until 110ft distance is traveled
        """
        self.get_logger().info('Starting line compliance test - robot will follow white lines')
        self.line_following_active = True
        
        # Send initial forward command to start moving
        # The line detection and control should take over from the bringup launch
        initial_cmd = Twist()
        initial_cmd.linear.x = 0.5  # Start with slow forward motion
        initial_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(initial_cmd)
        
        self.get_logger().info('Line following activated - bringup.launch.py should handle line detection and control')

    def gps_callback(self, msg: NavSatFix):
        """Track GPS position for distance calculation"""
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
    # ======================================================================= #




def main(args=None):
    rclpy.init(args=args)
    
    try:
        automater = T002Automater()
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