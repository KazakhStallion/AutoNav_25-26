#!/usr/bin/env python3
"""
t001_automater.py - GPS Calibration Test Automater (Standardized Version)

This script implements the GPS calibration test with the new standardized CSV format.

TEST Briefing (T001):
- A square waypoint pattern is made north of the robot and on the east side.
- WP0 (0,0) → WP1 (10,0) → WP2 (10,-10) → WP3 (0,-10) → WP0 (0,0)
- 5-second pause at each waypoint except final return to start.
"""

import rclpy
from base_automater import BaseAutomater
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import time

class T001Automater(BaseAutomater):
    def __init__(self):
        # Initialize base class with test-specific info
        super().__init__('t001_automater', 't001', 'GPS_Calibration')

        # ===== Test-Specific Variables ===== #
        # Waypoint tracking
        self.waypoints = {}
        self.current_waypoint_index = 0
        self.waypoint_reached = False
        self.pause_end_time = None
        
        # ===== Test Specific Publishers ===== #
        self.waypoint_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ===== Test Specific Subscribers ===== #
        # Add any test-specific subscribers here if needed
        
        # Initialize waypoints
        self.setup_waypoints()

    def test_manager(self):
        """Override base test manager to handle waypoint progression"""
        # Call parent test manager for standard behavior
        super().test_manager()
        
        # Handle waypoint progression
        if self.test_started and not self.test_complete:
            # Check if we're in a pause
            if self.pause_end_time is not None:
                if time.time() >= self.pause_end_time:
                    self.pause_end_time = None
                    self.get_logger().info('Pause completed, continuing to next waypoint')
                    self.proceed_to_next_waypoint()
            
            # Check if we've completed all waypoints
            if self.current_waypoint_index > max(self.waypoints.keys()):
                self.get_logger().info('All waypoints completed!')
                self.stop_test()

    # ===== Everything within these lines are specific to the t001 test ===== #
    def test_actions(self):
        """
        Test-specific actions for T001 GPS Calibration
        - Send waypoint commands in sequence
        - Handle pauses at each waypoint
        """
        self.get_logger().info('Starting GPS calibration test - waypoint navigation')
        
        # Start with first waypoint
        self.current_waypoint_index = 1
        self.send_waypoint(self.current_waypoint_index)

    def setup_waypoints(self):
        """Setup the waypoint sequence for GPS calibration"""
        # Square pattern: (0,0) → (10,0) → (10,-10) → (0,-10) → (0,0)
        self.waypoints = {
            0: {'x': 0.0, 'y': 0.0, 'pause': False},    # Start
            1: {'x': 10.0, 'y': 0.0, 'pause': True},    # North
            2: {'x': 10.0, 'y': -10.0, 'pause': True},  # East
            3: {'x': 0.0, 'y': -10.0, 'pause': True},   # South
            4: {'x': 0.0, 'y': 0.0, 'pause': False}     # Return to start
        }

    def send_waypoint(self, waypoint_index):
        """Send a waypoint goal to the navigation system"""
        if waypoint_index in self.waypoints:
            wp = self.waypoints[waypoint_index]
            
            # Create waypoint message
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = wp['x']
            goal_pose.pose.position.y = wp['y']
            goal_pose.pose.position.z = 0.0
            
            # Set orientation (facing forward)
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            
            # Publish waypoint
            self.waypoint_pub.publish(goal_pose)
            self.get_logger().info(f'Sent waypoint {waypoint_index}: ({wp["x"]}, {wp["y"]})')
            
            # Set pause if required
            if wp['pause']:
                self.pause_end_time = time.time() + 5.0  # 5 second pause
                self.get_logger().info('Pausing for 5 seconds at waypoint')

    def proceed_to_next_waypoint(self):
        """Move to the next waypoint in the sequence"""
        self.current_waypoint_index += 1
        if self.current_waypoint_index <= max(self.waypoints.keys()):
            self.send_waypoint(self.current_waypoint_index)
        else:
            self.get_logger().info('Waypoint sequence completed')
    # ======================================================================= #

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