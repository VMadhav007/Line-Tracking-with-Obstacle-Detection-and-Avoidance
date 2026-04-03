#!/usr/bin/env python3
"""
Line Detector Node
------------------
Subscribes to /camera/image_raw, applies HSV thresholding to detect a black
line on a white floor, computes the lateral error (centroid_x - image_center_x),
and publishes it on /line_error as a Float32.

A NaN value is published when the line is lost.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')

        # Parameters (tunable at launch time)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('roi_top_fraction', 0.6)   # use bottom 40 %
        self.declare_parameter('hsv_lower', [0, 0, 0])
        self.declare_parameter('hsv_upper', [180, 255, 50])
        self.declare_parameter('min_contour_area', 500.0)

        self.bridge = CvBridge()
        self.image_width = self.get_parameter('image_width').value
        self.roi_top = self.get_parameter('roi_top_fraction').value
        lower = self.get_parameter('hsv_lower').value
        upper = self.get_parameter('hsv_upper').value
        self.lower_black = np.array(lower, dtype=np.uint8)
        self.upper_black = np.array(upper, dtype=np.uint8)
        self.min_area = self.get_parameter('min_contour_area').value

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Float32, '/line_error', 10)

        self.get_logger().info('Line Detector node started.')

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge error: {e}')
            return

        h, w = frame.shape[:2]
        roi_y = int(h * self.roi_top)
        roi = frame[roi_y:h, :]

        # ── HSV thresholding for dark/black line ──
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        error_msg = Float32()

        # Filter contours by area to avoid noise
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_area]

        if valid:
            largest = max(valid, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                error_msg.data = float(cx - w // 2)   # +ve → line right of centre
            else:
                error_msg.data = float('nan')
        else:
            error_msg.data = float('nan')   # line lost

        self.pub.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
