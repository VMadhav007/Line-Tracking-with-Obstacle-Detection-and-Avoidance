#!/usr/bin/env python3
"""
controller.py  —  4-State FSM
==============================
States:
  FOLLOW_LINE       : PID line tracking (UNTOUCHED)
  OBSTACLE_DETECTED : immediate stop + pick turn direction
  AVOID_OBSTACLE    : open-loop timed maneuver
  SEARCH_LINE       : spin to re-acquire line

Key fixes vs original:
  - obstacle_distance_threshold : 0.45 → 0.70  (detect earlier)
  - avoid_turn_duration         : 0.8  → 1.4   (turn more)
  - avoid_forward_duration      : 0.7  → 1.2   (clear obstacle fully)
  - timing_scale                : 2.0  → 1.5   (less rushed)
  - avoid_turn_spd              : 1.80 → 2.20  (sharper turn)
  - avoid_fwd_spd               : 0.20 → 0.12  (slower forward, safer)
  - front_fov                   : 60°  → 90°   (wider detection cone)
  - Added safety re-trigger if obstacle still close during forward drive
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool

# ── State labels ──────────────────────────────────────────────────────────────
FOLLOW_LINE       = 'FOLLOW_LINE'
OBSTACLE_DETECTED = 'OBSTACLE_DETECTED'
AVOID_OBSTACLE    = 'AVOID_OBSTACLE'
SEARCH_LINE       = 'SEARCH_LINE'


class ControllerNode(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        # ── PID gains — UNTOUCHED ─────────────────────────────────────────────
        self.Kp         = self.declare_parameter('Kp',    0.005).value
        self.Ki         = self.declare_parameter('Ki',    0.0001).value
        self.Kd         = self.declare_parameter('Kd',    0.001).value
        self.prev_error = 0.0
        self.integral   = 0.0
        self.prev_time  = self.get_clock().now()

        # ── Line following speed — UNTOUCHED ──────────────────────────────────
        self.line_speed  = self.declare_parameter('linear_speed', 0.15).value
        self.max_angular = self.declare_parameter('max_angular',  2.0).value

        # ── Avoidance parameters ──────────────────────────────────────────────
        self.obs_threshold  = self.declare_parameter(
            'obstacle_distance_threshold', 0.70).value
        self.avoid_turn_dur = self.declare_parameter(
            'avoid_turn_duration',         1.4).value
        self.avoid_fwd_dur  = self.declare_parameter(
            'avoid_forward_duration',      1.2).value
        self.timing_scale   = self.declare_parameter(
            'timing_scale',                1.5).value

        # Physical speeds
        self.avoid_turn_spd = 2.20   # rad/s — turn in place
        self.avoid_fwd_spd  = 0.12   # m/s   — forward to clear obstacle
        self.search_rot_spd = 0.50   # rad/s — spin while searching line

        # ── Internal state ────────────────────────────────────────────────────
        self.state            = FOLLOW_LINE
        self.state_entry_time = time.time()

        self.line_error        = float('nan')
        self.obstacle_detected = False
        self.obs_min_dist      = 9.99
        self.left_min          = 9.99
        self.right_min         = 9.99

        self.turn_sign = -1   # +1=left, -1=right

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Float32, '/line_error',
                                 self.line_cb, 10)
        self.create_subscription(Bool,    '/obstacle/detected',
                                 self.obs_det_cb, 10)
        self.create_subscription(Float32, '/obstacle/min_distance',
                                 self.obs_min_cb, 10)
        self.create_subscription(Float32, '/obstacle/left_min',
                                 self.left_cb, 10)
        self.create_subscription(Float32, '/obstacle/right_min',
                                 self.right_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── 20 Hz control loop ────────────────────────────────────────────────
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Controller ready | state={FOLLOW_LINE} | '
            f'threshold={self.obs_threshold}m | '
            f'turn_dur={self.avoid_turn_dur}s | '
            f'fwd_dur={self.avoid_fwd_dur}s | '
            f'timing_scale={self.timing_scale}')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def line_cb(self, msg: Float32):
        self.line_error = msg.data

    def obs_det_cb(self, msg: Bool):
        self.obstacle_detected = msg.data

    def obs_min_cb(self, msg: Float32):
        self.obs_min_dist = msg.data

    def left_cb(self, msg: Float32):
        self.left_min = msg.data

    def right_cb(self, msg: Float32):
        self.right_min = msg.data

    # ── PID — UNTOUCHED ───────────────────────────────────────────────────────

    def pid(self, error: float) -> float:
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9
        dt  = max(dt, 0.001)

        self.integral += error * dt
        self.integral  = max(-500.0, min(500.0, self.integral))
        deriv          = (error - self.prev_error) / dt

        output = (self.Kp * error +
                  self.Ki * self.integral +
                  self.Kd * deriv)

        self.prev_error = error
        self.prev_time  = now
        return float(max(-self.max_angular, min(self.max_angular, output)))

    # ── Helpers ───────────────────────────────────────────────────────────────

    def publish_vel(self, linear: float, angular: float):
        msg            = Twist()
        msg.linear.x   = linear
        msg.angular.z  = angular
        self.cmd_pub.publish(msg)

    def transition(self, new_state: str):
        self.get_logger().info(f'STATE: {self.state} → {new_state}')
        self.state            = new_state
        self.state_entry_time = time.time()
        self.integral         = 0.0
        self.prev_error       = 0.0

    def line_visible(self) -> bool:
        return not math.isnan(self.line_error)

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        elapsed = time.time() - self.state_entry_time

        # ════════════════════════════════════════════════════════════════════
        # STATE: FOLLOW_LINE — UNTOUCHED
        # PID line tracking. Obstacle detected → stop → OBSTACLE_DETECTED
        # ════════════════════════════════════════════════════════════════════
        if self.state == FOLLOW_LINE:

            if self.obstacle_detected:
                self.publish_vel(0.0, 0.0)
                self.transition(OBSTACLE_DETECTED)
                return

            if not self.line_visible():
                self.publish_vel(0.04, 0.25)
                return

            angular = -self.pid(self.line_error)
            self.publish_vel(self.line_speed, angular)

        # ════════════════════════════════════════════════════════════════════
        # STATE: OBSTACLE_DETECTED
        # Full stop. Pick turn direction from side LiDAR sectors.
        #   left_min >= right_min → more room left  → turn left  (+1)
        #   left_min <  right_min → more room right → turn right (-1)
        # ════════════════════════════════════════════════════════════════════
        elif self.state == OBSTACLE_DETECTED:

            self.publish_vel(0.0, 0.0)

            if self.left_min >= self.right_min:
                self.turn_sign = 1
                side = 'LEFT'
            else:
                self.turn_sign = -1
                side = 'RIGHT'

            self.get_logger().info(
                f'Obstacle at {self.obs_min_dist:.2f}m → turning {side} '
                f'(L={self.left_min:.2f}m  R={self.right_min:.2f}m)')

            self.transition(AVOID_OBSTACLE)

        # ════════════════════════════════════════════════════════════════════
        # STATE: AVOID_OBSTACLE
        # Timed open-loop maneuver:
        #   t < avoid_turn_dur              → turn in place
        #   t < avoid_turn_dur + fwd_dur    → drive forward
        #   safety: if obstacle still close while going forward → retry
        # ════════════════════════════════════════════════════════════════════
        elif self.state == AVOID_OBSTACLE:

            t = elapsed * self.timing_scale

            # Safety: still blocked while trying to go forward → re-evaluate
            if t >= self.avoid_turn_dur and self.obs_min_dist < 0.40:
                self.publish_vel(0.0, 0.0)
                self.transition(OBSTACLE_DETECTED)
                return

            turn_end = self.avoid_turn_dur
            fwd_end  = self.avoid_turn_dur + self.avoid_fwd_dur

            if t < turn_end:
                # Segment 1: turn in place
                self.publish_vel(0.0, self.turn_sign * self.avoid_turn_spd)

            elif t < fwd_end:
                # Segment 2: drive forward to clear obstacle
                self.publish_vel(self.avoid_fwd_spd, 0.0)

            else:
                # Maneuver complete → search for line
                self.publish_vel(0.0, 0.0)
                self.transition(SEARCH_LINE)

        # ════════════════════════════════════════════════════════════════════
        # STATE: SEARCH_LINE
        # Spin slowly to sweep camera and re-acquire the line.
        #   obstacle still present → OBSTACLE_DETECTED
        #   line found             → FOLLOW_LINE
        #   otherwise              → keep spinning
        # ════════════════════════════════════════════════════════════════════
        elif self.state == SEARCH_LINE:

            if self.obstacle_detected:
                self.publish_vel(0.0, 0.0)
                self.transition(OBSTACLE_DETECTED)
                return

            if self.line_visible():
                self.get_logger().info(
                    f'Line re-acquired | error={self.line_error:.1f}px → FOLLOW_LINE')
                self.transition(FOLLOW_LINE)
                return

            # Keep spinning in same direction as avoidance turn
            self.publish_vel(0.0, self.turn_sign * self.search_rot_spd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()