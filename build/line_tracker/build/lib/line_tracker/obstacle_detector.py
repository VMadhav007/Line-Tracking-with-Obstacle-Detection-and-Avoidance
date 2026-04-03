#!/usr/bin/env python3
"""
Obstacle Detector Node
----------------------
Subscribes to /scan (sensor_msgs/LaserScan) and publishes a Bool on
/obstacle_flag.  Checks the front ±30° arc (60° total) for any reading
below the configurable threshold.

NaN / Inf / out-of-range values are discarded.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('threshold', 0.5)       # metres
        self.declare_parameter('front_half_angle', 30.0)  # degrees each side

        self.threshold = self.get_parameter('threshold').value
        self.half_deg = self.get_parameter('front_half_angle').value

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Bool, '/obstacle_flag', 10)

        self.get_logger().info(
            f'Obstacle Detector started — threshold={self.threshold} m, '
            f'arc=±{self.half_deg}°')

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        # Convert ±half_deg to number of indices
        angle_increment = msg.angle_increment  # rad per index
        half_idx = int(math.radians(self.half_deg) / angle_increment)

        # Front arc: indices near 0 and near n
        front_indices = (
            list(range(0, half_idx + 1)) +
            list(range(max(0, n - half_idx), n))
        )

        obstacle = False
        for i in front_indices:
            r = msg.ranges[i]
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            if r < self.threshold:
                obstacle = True
                break

        flag = Bool()
        flag.data = obstacle
        self.pub.publish(flag)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
