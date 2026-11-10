#!/usr/bin/env python3
"""
Intelligent Autonomous Exploration Node for Andino AMR & ROS 2 SLAM
Properly accounts for Andino's RPLiDAR coordinate frame (180° yaw relative to base_link).
Provides:
- Real robot-frame obstacle detection (Front / Left / Right)
- Dynamic Safety Corridor with Smooth Arc Bypassing
- Lateral Wall Repulsion to avoid scraping side panels
- Deadlock / Corner Escape Recovery
- Initial 360° panoramic scan for SLAM anchor
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SmartAutoExplorer(Node):
    def __init__(self):
        super().__init__('smart_auto_explorer')

        # Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # High-frequency control loop (15 Hz)
        self.timer = self.create_timer(1.0 / 15.0, self.control_loop)

        # Robot geometry & safety parameters (in robot body frame 'base_link')
        # Andino width is ~0.30m -> half-width 0.15m + 0.13m buffer = 0.28m
        self.corridor_half_width = 0.28      # meters
        self.d_critical_stop = 0.45          # meters (stop forward motion & rotate)
        self.d_slowdown_start = 0.95         # meters (start slowing down & curving away)
        self.side_safety_dist = 0.32         # meters (repulsive distance from lateral walls)

        self.max_linear_speed = 0.26         # m/s
        self.min_linear_speed = 0.08         # m/s
        self.max_angular_speed = 0.75        # rad/s

        # States: 'INITIAL_SPIN', 'EXPLORING', 'ESCAPE_BACKUP', 'ESCAPE_SPIN'
        self.state = 'INITIAL_SPIN'
        self.state_start_time = time.time()
        self.latest_scan = None

        # Hysteresis for turning direction to prevent jitter
        self.preferred_turn_dir = 1.0        # +1.0 = Turn Left, -1.0 = Turn Right
        self.turn_lock_until = 0.0

        # Deadlock / trapped detection
        self.stopped_time = None
        self.escape_turn_dir = 1.0

        self.get_logger().info('========================================================')
        self.get_logger().info(' Smart Autonomous Explorer Online!                      ')
        self.get_logger().info(' Sensor orientation: RPLiDAR (180° calibrated)          ')
        self.get_logger().info(' Step 1: Performing 360° scan to seed SLAM map...      ')
        self.get_logger().info('========================================================')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            self.state_start_time = time.time()
            self.get_logger().info(f'State changed -> {new_state}')

    def control_loop(self):
        if self.latest_scan is None:
            return

        twist = Twist()
        now = time.time()
        elapsed = now - self.state_start_time

        # --- 1. INITIAL 360° PANORAMA ---
        if self.state == 'INITIAL_SPIN':
            if elapsed < 8.5:
                twist.linear.x = 0.0
                twist.angular.z = 0.75
                self.cmd_pub.publish(twist)
                return
            else:
                self.get_logger().info('Panoramic scan complete! Autonomously moving forward...')
                self.set_state('EXPLORING')

        # --- 2. ESCAPE BACKUP ---
        if self.state == 'ESCAPE_BACKUP':
            if elapsed < 1.5:
                twist.linear.x = -0.12
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return
            else:
                self.set_state('ESCAPE_SPIN')
                return

        # --- 3. ESCAPE SPIN ---
        if self.state == 'ESCAPE_SPIN':
            if elapsed < 2.0:
                twist.linear.x = 0.0
                twist.angular.z = self.escape_turn_dir * 0.85
                self.cmd_pub.publish(twist)
                return
            else:
                self.set_state('EXPLORING')
                self.stopped_time = None

        # --- 4. SENSOR DATA PROCESSING & COORDINATE TRANSFORMATION ---
        ranges = np.array(self.latest_scan.ranges, dtype=np.float32)
        n = len(ranges)
        if n < 100:
            return

        # Filter out-of-range & invalid readings
        r_min = max(self.latest_scan.range_min, 0.12)
        r_max = min(self.latest_scan.range_max, 12.0)
        invalid = np.isnan(ranges) | np.isinf(ranges) | (ranges < r_min)
        ranges[invalid] = r_max

        # Laser sensor angles in rplidar_laser_link frame
        laser_angles = self.latest_scan.angle_min + np.arange(n) * self.latest_scan.angle_increment

        # IMPORTANT: Andino's RPLiDAR is mounted with yaw = pi (180°) relative to base_link!
        # Therefore, in robot body frame (base_link):
        # x_robot = -range * cos(laser_angle) + 0.05
        # y_robot = -range * sin(laser_angle)
        # where +x_robot is FORWARD, +y_robot is LEFT, -y_robot is RIGHT.
        x_robot = -ranges * np.cos(laser_angles) + 0.05
        y_robot = -ranges * np.sin(laser_angles)

        # --- A. FRONT SAFETY CORRIDOR EVALUATION ---
        # Obstacles directly in front of the robot within our driving width
        corridor_mask = (x_robot > 0.05) & (x_robot < self.d_slowdown_start) & (np.abs(y_robot) < self.corridor_half_width)
        in_corridor_xs = x_robot[corridor_mask]
        min_front_dist = np.min(in_corridor_xs) if len(in_corridor_xs) > 0 else 999.0

        # --- B. LATERAL WALL DETECTION (Left vs Right) ---
        # Detect if any wall is brushing against our side body (|x_robot| < 0.35m)
        left_side_close = np.any((np.abs(x_robot) < 0.35) & (y_robot > 0.0) & (y_robot < self.side_safety_dist))
        right_side_close = np.any((np.abs(x_robot) < 0.35) & (y_robot < 0.0) & (y_robot > -self.side_safety_dist))

        # --- C. FORWARD CLEARANCE IN LEFT VS RIGHT QUADRANTS ---
        # Evaluate which direction offers more open space to navigate towards
        left_forward_mask = (x_robot > 0.1) & (x_robot < 2.5) & (y_robot > 0.1) & (y_robot < 1.8)
        right_forward_mask = (x_robot > 0.1) & (x_robot < 2.5) & (y_robot < -0.1) & (y_robot > -1.8)

        left_space = np.mean(ranges[left_forward_mask]) if np.any(left_forward_mask) else 1.5
        right_space = np.mean(ranges[right_forward_mask]) if np.any(right_forward_mask) else 1.5

        # Update preferred turning direction with hysteresis (stay locked for 0.7 sec)
        if now > self.turn_lock_until:
            if left_space > right_space + 0.15:
                self.preferred_turn_dir = 1.0   # Turn Left (+Z)
            elif right_space > left_space + 0.15:
                self.preferred_turn_dir = -1.0  # Turn Right (-Z)
            self.turn_lock_until = now + 0.7

        # Lateral bias to keep robot centered between walls
        lateral_steer = 0.0
        if left_side_close and not right_side_close:
            lateral_steer = -0.30  # Steer right to push away from left wall
        elif right_side_close and not left_side_close:
            lateral_steer = 0.30   # Steer left to push away from right wall

        # --- D. MOTION DECISION & SMOOTH BYPASS ---
        if min_front_dist <= self.d_critical_stop:
            # Obstacle directly ahead inside critical zone: STOP forward drive, rotate to clear path
            twist.linear.x = 0.0
            twist.angular.z = self.preferred_turn_dir * self.max_angular_speed

            # Stuck in corner detection (> 4.0 seconds blocked)
            if self.stopped_time is None:
                self.stopped_time = now
            elif now - self.stopped_time > 4.0:
                self.get_logger().warn('Corner/Trap detected! Backing up and turning around...')
                self.escape_turn_dir = -self.preferred_turn_dir
                self.set_state('ESCAPE_BACKUP')
                return

        elif min_front_dist < self.d_slowdown_start:
            # Obstacle detected in corridor ahead: SMOOTH ARC BYPASS
            self.stopped_time = None

            # Dynamically scale speed: closer obstacle = slower speed
            speed_ratio = (min_front_dist - self.d_critical_stop) / (self.d_slowdown_start - self.d_critical_stop)
            speed_ratio = float(np.clip(speed_ratio, 0.0, 1.0))

            twist.linear.x = self.min_linear_speed + speed_ratio * (self.max_linear_speed - self.min_linear_speed)
            # Smoothly curve away from the obstacle while still moving forward
            turn_intensity = (1.0 - speed_ratio) * self.max_angular_speed
            twist.angular.z = (self.preferred_turn_dir * turn_intensity) + lateral_steer

        else:
            # Path ahead is completely open: cruise forward at full speed
            self.stopped_time = None
            twist.linear.x = self.max_linear_speed
            twist.angular.z = lateral_steer  # Keep centered between side walls

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SmartAutoExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            if rclpy.ok():
                stop_twist = Twist()
                node.cmd_pub.publish(stop_twist)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
