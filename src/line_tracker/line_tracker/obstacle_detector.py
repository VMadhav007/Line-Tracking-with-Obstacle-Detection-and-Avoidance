#!/usr/bin/env python3
"""
obstacle_detector.py
Subscribes to /scan (LaserScan).
Publishes /obstacle_flag (Bool) — True if obstacle within threshold.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        # --- Parameters (easy to tune) ---
        self.declare_parameter('threshold_m',     0.5)   # stop distance in metres
        self.declare_parameter('front_angle_deg', 30)    # half-width of front arc

        self.threshold  = self.get_parameter('threshold_m').value
        self.half_angle = self.get_parameter('front_angle_deg').value

        # --- ROS interfaces ---
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.pub = self.create_publisher(Bool, '/obstacle_flag', 10)

        self.get_logger().info(
            f'ObstacleDetector ready | threshold={self.threshold}m '
            f'| front arc=\u00b1{self.half_angle}\u00b0')

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        n      = len(ranges)          # usually 360

        # Build front arc indices
        arc_size  = self.half_angle   # e.g. 30
        front_idx = (list(range(n - arc_size, n)) +
                     list(range(0, arc_size + 1)))

        # Filter NaN / inf / out-of-range readings
        valid = [
            ranges[i]
            for i in front_idx
            if i < n and
               ranges[i] == ranges[i] and           # not NaN
               0.01 < ranges[i] < msg.range_max     # valid range
        ]

        # Detect obstacle
        obstacle  = bool(valid) and any(r < self.threshold for r in valid)

        flag      = Bool()
        flag.data = obstacle
        self.pub.publish(flag)

        if obstacle:
            min_dist = min(valid)
            self.get_logger().debug(
                f'OBSTACLE detected | closest={min_dist:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
