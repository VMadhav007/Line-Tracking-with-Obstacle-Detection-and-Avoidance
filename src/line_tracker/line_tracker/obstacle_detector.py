#!/usr/bin/env python3
"""
obstacle_detector.py
=====================
Reads LiDAR /scan and detects obstacles in the front cone.

IMPORTANT: On this robot, angle_min = -180deg, so:
  index 0   = directly BEHIND the robot
  index 180 = directly IN FRONT of the robot

All arc calculations are centered on index 180 (front).

Publishes:
  /obstacle/detected      (Bool)    — True when obstacle in front cone
  /obstacle/min_distance  (Float32) — minimum distance in front cone
  /obstacle/left_min      (Float32) — min dist in left sector
  /obstacle/right_min     (Float32) — min dist in right sector
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('obstacle_distance_threshold', 0.55)
        self.declare_parameter('front_fov_deg',               90)

        self.threshold = self.get_parameter('obstacle_distance_threshold').value
        self.front_fov = int(self.get_parameter('front_fov_deg').value)
        self.half_fov  = self.front_fov // 2

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_detected  = self.create_publisher(Bool,    '/obstacle/detected',     10)
        self.pub_min_dist  = self.create_publisher(Float32, '/obstacle/min_distance', 10)
        self.pub_left_min  = self.create_publisher(Float32, '/obstacle/left_min',     10)
        self.pub_right_min = self.create_publisher(Float32, '/obstacle/right_min',    10)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.get_logger().info(
            f'ObstacleDetector ready | '
            f'threshold={self.threshold}m | front_fov={self.front_fov}deg | '
            f'front_index=180 (angle_min=-pi convention)')

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _min_in_arc(ranges, start_idx, end_idx, n, range_max):
        """
        Return minimum valid reading in index arc [start_idx, end_idx) mod n.
        Returns math.inf if no valid readings.
        """
        vals  = []
        idx   = start_idx % n
        steps = (end_idx - start_idx) % n
        for _ in range(steps):
            r = ranges[idx]
            if not math.isnan(r) and not math.isinf(r) and 0.005 < r < range_max:
                vals.append(r)
            idx = (idx + 1) % n
        return min(vals) if vals else math.inf

    # ── Main callback ─────────────────────────────────────────────────────────

    def scan_cb(self, msg: LaserScan):
        ranges = msg.ranges
        n      = len(ranges)   # 360
        rmax   = msg.range_max

        # Front of robot = index 180 (since angle_min = -pi = index 0 = behind)
        front_center = n // 2   # = 180
        half         = self.half_fov

        # Front cone: centered at index 180
        front_right = self._min_in_arc(ranges, front_center - half, front_center,        n, rmax)
        front_left  = self._min_in_arc(ranges, front_center,        front_center + half, n, rmax)
        d_min_front = min(front_left, front_right)

        obstacle = d_min_front < self.threshold

        # Side sectors for turn direction decision
        # Left:  front+half to front+135
        # Right: front-135  to front-half
        left_min  = self._min_in_arc(ranges, front_center + half, front_center + 135, n, rmax)
        right_min = self._min_in_arc(ranges, front_center - 135,  front_center - half, n, rmax)

        # ── Publish ───────────────────────────────────────────────────────────
        det = Bool()
        det.data = obstacle
        self.pub_detected.publish(det)

        md = Float32()
        md.data = float(d_min_front if d_min_front != math.inf else 9.99)
        self.pub_min_dist.publish(md)

        lm = Float32()
        lm.data = float(left_min if left_min != math.inf else 9.99)
        self.pub_left_min.publish(lm)

        rm = Float32()
        rm.data = float(right_min if right_min != math.inf else 9.99)
        self.pub_right_min.publish(rm)

        if obstacle:
            self.get_logger().info(
                f'OBSTACLE | d_min={d_min_front:.2f}m '
                f'| left={left_min:.2f}m right={right_min:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()