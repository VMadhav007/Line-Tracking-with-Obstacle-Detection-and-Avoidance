#!/usr/bin/env python3
"""
Obstacle Detector Node
======================
Subscribes to the LIDAR /scan topic and detects obstacles in the robot's
front arc. Publishes obstacle status and distance.

Algorithm:
  1. Receive LaserScan from /scan
  2. Extract ranges in the front arc (-30° to +30°)
  3. Find minimum distance in the front arc
  4. If min distance < threshold → obstacle detected
  5. Publish Bool on /obstacle_detected and Float32 on /obstacle_distance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import math


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_node')

        # Declare parameters
        self.declare_parameter('obstacle_distance_threshold', 0.35)
        self.declare_parameter('front_arc_degrees', 30.0)

        # Get parameters
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.front_arc_deg = self.get_parameter('front_arc_degrees').value

        # Publishers
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Obstacle Detector Node started')
        self.get_logger().info(f'  threshold={self.obstacle_threshold}m, front_arc={self.front_arc_deg}°')

    def scan_callback(self, msg):
        """Process LIDAR scan to detect front obstacles."""
        # Calculate which indices correspond to the front arc
        # The TurtleBot3 LIDAR has 360 samples covering 0 to 2π
        # Index 0 is directly in front
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        angle_increment = msg.angle_increment
        front_arc_rad = math.radians(self.front_arc_deg)

        # Number of samples in the front arc (half on each side)
        num_front_samples = int(front_arc_rad / angle_increment)

        # Collect ranges from front arc:
        # Left side of front: indices 0 to num_front_samples
        # Right side of front: indices (num_readings - num_front_samples) to num_readings
        front_ranges = []

        for i in range(num_front_samples + 1):
            if i < num_readings:
                r = msg.ranges[i]
                if msg.range_min < r < msg.range_max:
                    front_ranges.append(r)

        for i in range(num_readings - num_front_samples, num_readings):
            if 0 <= i < num_readings:
                r = msg.ranges[i]
                if msg.range_min < r < msg.range_max:
                    front_ranges.append(r)

        # Determine if obstacle is present
        obstacle_msg = Bool()
        distance_msg = Float32()

        if front_ranges:
            min_distance = min(front_ranges)
            distance_msg.data = min_distance

            if min_distance < self.obstacle_threshold:
                obstacle_msg.data = True
                self.get_logger().info(
                    f'OBSTACLE DETECTED at {min_distance:.2f}m (threshold: {self.obstacle_threshold}m)'
                )
            else:
                obstacle_msg.data = False
        else:
            obstacle_msg.data = False
            distance_msg.data = float('inf')

        self.obstacle_pub.publish(obstacle_msg)
        self.distance_pub.publish(distance_msg)


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
