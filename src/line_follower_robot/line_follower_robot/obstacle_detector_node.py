#!/usr/bin/env python3
"""
Obstacle Detector Node — Improved
===================================
Subscribes to LIDAR /scan topic and detects obstacles in the front arc.
Publishes obstacle status, distance, and direction information.

Improvements over v1:
  - Wider detection arc for early warning
  - Separate close/far thresholds for urgency levels
  - Publishes obstacle direction (left/right/center) for smarter avoidance
  - Rate-limited logging to avoid spam
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
import math


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_node')

        # Declare parameters
        self.declare_parameter('obstacle_distance_threshold', 0.50)
        self.declare_parameter('emergency_distance_threshold', 0.25)
        self.declare_parameter('front_arc_degrees', 40.0)
        self.declare_parameter('side_check_arc_degrees', 90.0)

        # Get parameters
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.emergency_threshold = self.get_parameter('emergency_distance_threshold').value
        self.front_arc_deg = self.get_parameter('front_arc_degrees').value
        self.side_arc_deg = self.get_parameter('side_check_arc_degrees').value

        # Publishers
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.emergency_pub = self.create_publisher(Bool, '/obstacle_emergency', 10)
        self.distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.direction_pub = self.create_publisher(String, '/obstacle_direction', 10)
        # Side clearance publishers (used during bypass)
        self.left_clear_pub = self.create_publisher(Bool, '/left_side_clear', 10)
        self.right_clear_pub = self.create_publisher(Bool, '/right_side_clear', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.log_counter = 0

        self.get_logger().info('Obstacle Detector Node started (improved)')
        self.get_logger().info(f'  threshold={self.obstacle_threshold}m, emergency={self.emergency_threshold}m')
        self.get_logger().info(f'  front_arc={self.front_arc_deg}°, side_arc={self.side_arc_deg}°')

    def get_ranges_in_arc(self, msg, center_deg, half_arc_deg):
        """
        Extract valid ranges from a LIDAR scan within a given arc.
        center_deg: center of arc (0=front, 90=left, 270=right)
        half_arc_deg: half-width of arc in degrees
        """
        num_readings = len(msg.ranges)
        angle_increment_deg = math.degrees(msg.angle_increment)

        center_idx = int(center_deg / angle_increment_deg) % num_readings
        half_samples = int(half_arc_deg / angle_increment_deg)

        ranges = []
        for offset in range(-half_samples, half_samples + 1):
            idx = (center_idx + offset) % num_readings
            r = msg.ranges[idx]
            if msg.range_min < r < msg.range_max:
                ranges.append(r)

        return ranges

    def scan_callback(self, msg):
        """Process LIDAR scan to detect obstacles."""
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        # ── Front arc detection ─────────────────────────────────────────
        front_ranges = self.get_ranges_in_arc(msg, 0.0, self.front_arc_deg)

        # ── Left front quadrant (for direction) ─────────────────────────
        left_front = self.get_ranges_in_arc(msg, 30.0, 15.0)
        right_front = self.get_ranges_in_arc(msg, 330.0, 15.0)

        # ── Side clearance checks (for bypass navigation) ───────────────
        left_side = self.get_ranges_in_arc(msg, 90.0, 30.0)
        right_side = self.get_ranges_in_arc(msg, 270.0, 30.0)

        # ── Determine obstacle status ───────────────────────────────────
        obstacle_msg = Bool()
        emergency_msg = Bool()
        distance_msg = Float32()
        direction_msg = String()
        left_clear_msg = Bool()
        right_clear_msg = Bool()

        if front_ranges:
            min_distance = min(front_ranges)
            distance_msg.data = min_distance

            # Normal detection
            if min_distance < self.obstacle_threshold:
                obstacle_msg.data = True

                # Determine direction — which side has more clearance?
                left_min = min(left_front) if left_front else 999.0
                right_min = min(right_front) if right_front else 999.0

                if left_min > right_min:
                    direction_msg.data = 'RIGHT'  # obstacle is more to the right
                elif right_min > left_min:
                    direction_msg.data = 'LEFT'  # obstacle is more to the left
                else:
                    direction_msg.data = 'CENTER'

                # Log (rate limited)
                self.log_counter += 1
                if self.log_counter % 5 == 0:
                    self.get_logger().info(
                        f'OBSTACLE at {min_distance:.2f}m [{direction_msg.data}]'
                    )
            else:
                obstacle_msg.data = False
                direction_msg.data = 'NONE'

            # Emergency (too close!)
            emergency_msg.data = (min_distance < self.emergency_threshold)
        else:
            obstacle_msg.data = False
            emergency_msg.data = False
            distance_msg.data = float('inf')
            direction_msg.data = 'NONE'

        # Side clearance
        left_clear_msg.data = (min(left_side) > 0.4) if left_side else True
        right_clear_msg.data = (min(right_side) > 0.4) if right_side else True

        # Publish everything
        self.obstacle_pub.publish(obstacle_msg)
        self.emergency_pub.publish(emergency_msg)
        self.distance_pub.publish(distance_msg)
        self.direction_pub.publish(direction_msg)
        self.left_clear_pub.publish(left_clear_msg)
        self.right_clear_pub.publish(right_clear_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
