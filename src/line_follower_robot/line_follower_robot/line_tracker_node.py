#!/usr/bin/env python3
"""
Line Tracker Node — PID Controller with Dynamic Speed
======================================================
Uses OpenCV to detect a black line on a white surface via the camera,
then applies a full PID controller with dynamic speed for smooth,
reliable line following.

Key features:
  - Clean vision pipeline: grayscale → blur → threshold → ROI → centroid
  - Normalized error for stable PID behavior
  - PID control (Kp + Ki + Kd) for smooth steering
  - Dynamic speed: fast on straights, slow on curves
  - Line-lost recovery: slow rotation to re-acquire
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineTrackerNode(Node):

    def __init__(self):
        super().__init__('line_tracker_node')

        # ── PID Parameters ──────────────────────────────────────────────
        self.declare_parameter('kp', 0.3)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)

        # ── Camera Offset Compensation ──────────────────────────────────
        # The Waffle camera is 4.7cm to the RIGHT of robot center.
        # This shifts the target point in the image to compensate.
        # Positive value = shift target rightward in image.
        self.declare_parameter('camera_offset_pixels', 25)

        # ── Derivative Filter ───────────────────────────────────────────
        # Low-pass filter alpha for derivative (0.0–1.0)
        # Lower = smoother (less wobble), higher = more responsive
        self.declare_parameter('derivative_filter_alpha', 0.3)

        # ── Dynamic Speed Parameters ────────────────────────────────────
        self.declare_parameter('max_speed', 0.22)
        self.declare_parameter('min_speed', 0.08)
        self.declare_parameter('speed_sensitivity', 2.5)

        # ── Vision Parameters ───────────────────────────────────────────
        self.declare_parameter('crop_top_ratio', 0.60)
        self.declare_parameter('black_threshold', 100)
        self.declare_parameter('min_contour_area', 500)

        # ── Recovery Parameters ─────────────────────────────────────────
        self.declare_parameter('recovery_linear_speed', 0.05)
        self.declare_parameter('recovery_angular_speed', 0.3)

        # Get all parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.camera_offset_pixels = self.get_parameter('camera_offset_pixels').value
        self.derivative_alpha = self.get_parameter('derivative_filter_alpha').value

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.speed_sensitivity = self.get_parameter('speed_sensitivity').value

        self.crop_top_ratio = self.get_parameter('crop_top_ratio').value
        self.black_threshold = self.get_parameter('black_threshold').value
        self.min_contour_area = self.get_parameter('min_contour_area').value

        self.recovery_linear = self.get_parameter('recovery_linear_speed').value
        self.recovery_angular = self.get_parameter('recovery_angular_speed').value

        # ── PID State Variables ─────────────────────────────────────────
        self.last_error = 0.0
        self.integral = 0.0
        self.filtered_derivative = 0.0  # low-pass filtered derivative
        self.dt = 0.033  # ~30 Hz camera update rate

        # ── CV Bridge ───────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Publishers ──────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/line_cmd_vel', 10)
        self.line_detected_pub = self.create_publisher(Bool, '/line_detected', 10)

        # ── Subscribers ─────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.line_detected = False

        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('  Line Tracker Node — PID + Dynamic Speed')
        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info(f'  PID: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')
        self.get_logger().info(f'  Derivative filter alpha={self.derivative_alpha}')
        self.get_logger().info(f'  Camera offset={self.camera_offset_pixels}px')
        self.get_logger().info(f'  Speed: max={self.max_speed}, min={self.min_speed}, sens={self.speed_sensitivity}')
        self.get_logger().info(f'  Vision: crop={self.crop_top_ratio}, thresh={self.black_threshold}')

    def image_callback(self, msg):
        """Process camera image → detect line → PID steering → publish velocity."""

        # ── Step 1: Convert ROS Image to OpenCV ─────────────────────────
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        height, width, _ = cv_image.shape

        # ── Step 2: Clean Vision Pipeline ───────────────────────────────
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Gaussian blur to reduce noise (critical for stable centroid)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Binary threshold — invert so black line becomes white
        _, binary = cv2.threshold(blur, self.black_threshold, 255, cv2.THRESH_BINARY_INV)

        # ── Step 3: Region of Interest (bottom portion only) ────────────
        roi = binary[int(height * self.crop_top_ratio):height, :]

        # ── Step 4: Find Centroid ───────────────────────────────────────
        M = cv2.moments(roi)

        twist = Twist()
        line_msg = Bool()

        if M['m00'] > self.min_contour_area:
            # ── LINE DETECTED ───────────────────────────────────────────
            cx = int(M['m10'] / M['m00'])

            # Compensate for camera being offset to the right of robot center
            # Shift the target point so the robot centers its BODY on the line
            center = (width // 2) + self.camera_offset_pixels

            # Normalized error: range roughly [-1.0, +1.0]
            error = (cx - center) / (width // 2)

            # ── Step 5: PID Control ─────────────────────────────────────
            # Integral term (with anti-windup clamping)
            self.integral += error * self.dt
            self.integral = max(-1.0, min(1.0, self.integral))  # anti-windup

            # Derivative term with LOW-PASS FILTER to kill wobble
            raw_derivative = (error - self.last_error) / self.dt
            self.filtered_derivative = (
                self.derivative_alpha * raw_derivative
                + (1.0 - self.derivative_alpha) * self.filtered_derivative
            )

            # PID output → angular velocity
            angular_z = (
                (self.kp * error)
                + (self.ki * self.integral)
                + (self.kd * self.filtered_derivative)
            )

            # Clamp angular velocity to safe range
            angular_z = max(-1.5, min(1.5, angular_z))

            # Save for next iteration
            self.last_error = error

            # ── Step 6: Dynamic Speed ───────────────────────────────────
            # Fast on straights (small error), slow on curves (large error)
            linear_x = self.max_speed / (1.0 + abs(error) * self.speed_sensitivity)
            linear_x = max(self.min_speed, min(self.max_speed, linear_x))

            twist.linear.x = linear_x
            twist.angular.z = angular_z

            self.line_detected = True
            line_msg.data = True

            self.get_logger().debug(
                f'LINE: cx={cx} err={error:.3f} P={self.kp*error:.3f} '
                f'D={self.kd*self.filtered_derivative:.3f} ang={angular_z:.3f} vel={linear_x:.3f}'
            )

        else:
            # ── LINE LOST — Recovery Mode ───────────────────────────────
            # Slow forward + gentle rotation to search for line
            twist.linear.x = self.recovery_linear
            twist.angular.z = self.recovery_angular

            self.line_detected = False
            line_msg.data = False

            # Reset integral to avoid windup during recovery
            self.integral = 0.0

            self.get_logger().info('LINE LOST — rotating to recover...')

        # ── Publish ─────────────────────────────────────────────────────
        self.cmd_pub.publish(twist)
        self.line_detected_pub.publish(line_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
