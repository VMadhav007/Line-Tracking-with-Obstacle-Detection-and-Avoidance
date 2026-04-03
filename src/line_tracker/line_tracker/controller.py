#!/usr/bin/env python3
"""
controller.py
Combines PID line-following with Bug0 obstacle avoidance.
States: LINE_FOLLOW → STOP → AVOID → RETURN → LINE_FOLLOW
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


# ── State constants ──────────────────────────────────────────────
LINE_FOLLOW = 'LINE_FOLLOW'
STOP        = 'STOP'
AVOID       = 'AVOID'
RETURN      = 'RETURN'


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # ── PID parameters ────────────────────────────────────
        self.Kp         = 0.005
        self.Ki         = 0.0001
        self.Kd         = 0.001
        self.prev_error = 0.0
        self.integral   = 0.0
        self.prev_time  = self.get_clock().now()

        # ── Motion parameters ─────────────────────────────────
        self.linear_speed    = 0.15    # m/s forward speed on line
        self.avoid_turn_spd  = -0.8   # rad/s  (negative = turn right)
        self.avoid_fwd_spd   = 0.10   # m/s creep speed past obstacle
        self.return_spd      = 0.08   # m/s search speed
        self.return_turn     = 0.5    # rad/s curve left when searching

        # ── Timing ────────────────────────────────────────────
        self.stop_hold_s  = 0.5   # seconds to hold STOP before avoidance
        self.avoid_fwd_s  = 2.0   # seconds to move forward past obstacle

        # ── State machine ─────────────────────────────────────
        self.state         = LINE_FOLLOW
        self.line_error    = 0.0
        self.obstacle      = False
        self.state_entry_t = time.time()

        # Track when path first becomes clear (inside AVOID)
        self.avoid_clear_t = None

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(
            Float32, '/line_error',    self.line_error_cb, 10)
        self.create_subscription(
            Bool,    '/obstacle_flag', self.obstacle_cb,   10)

        # ── Publisher ─────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Control loop at 20 Hz ─────────────────────────────
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Controller ready — state: LINE_FOLLOW')

    # ── Callbacks ────────────────────────────────────────────────

    def line_error_cb(self, msg: Float32):
        self.line_error = msg.data          # NaN if line not visible

    def obstacle_cb(self, msg: Bool):
        self.obstacle = msg.data

    # ── PID (unchanged from base line-follow implementation) ──────

    def pid(self, error: float) -> float:
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9
        dt  = max(dt, 0.001)                # guard divide-by-zero

        self.integral  += error * dt
        # Anti-windup
        self.integral   = max(-500.0, min(500.0, self.integral))
        derivative      = (error - self.prev_error) / dt

        output = (self.Kp * error +
                  self.Ki * self.integral +
                  self.Kd * derivative)

        self.prev_error = error
        self.prev_time  = now

        return float(max(-2.0, min(2.0, output)))   # clamp to ±2 rad/s

    # ── Velocity publisher ────────────────────────────────────────

    def publish_vel(self, linear: float, angular: float):
        msg           = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    # ── State transition helper ───────────────────────────────────

    def transition(self, new_state: str):
        self.get_logger().info(f'{self.state} → {new_state}')
        self.state         = new_state
        self.state_entry_t = time.time()

    # ── Main control loop ─────────────────────────────────────────

    def control_loop(self):
        now     = time.time()
        elapsed = now - self.state_entry_t

        # ━━ LINE_FOLLOW ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if self.state == LINE_FOLLOW:

            if self.obstacle:
                self.publish_vel(0.0, 0.0)
                self.transition(STOP)
                return

            if math.isnan(self.line_error):
                # Line briefly lost — crawl and sweep
                self.publish_vel(0.05, 0.3)
                return

            angular = -self.pid(self.line_error)
            self.publish_vel(self.linear_speed, angular)

        # ━━ STOP ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.state == STOP:

            self.publish_vel(0.0, 0.0)

            if elapsed >= self.stop_hold_s:
                self.integral      = 0.0    # flush PID before avoidance
                self.avoid_clear_t = None
                self.transition(AVOID)

        # ━━ AVOID (Bug0) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.state == AVOID:

            if not self.obstacle:
                # Front is clear — creep forward past the obstacle
                if self.avoid_clear_t is None:
                    self.avoid_clear_t = now

                self.publish_vel(self.avoid_fwd_spd, 0.0)

                if now - self.avoid_clear_t >= self.avoid_fwd_s:
                    self.transition(RETURN)
            else:
                # Still blocked — turn right to go around
                self.avoid_clear_t = None
                self.publish_vel(0.05, self.avoid_turn_spd)

        # ━━ RETURN ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.state == RETURN:

            if not math.isnan(self.line_error):
                # Line re-acquired — reset PID and resume
                self.integral   = 0.0
                self.prev_error = 0.0
                self.transition(LINE_FOLLOW)
            else:
                # Curve left slowly until camera sees the line
                self.publish_vel(self.return_spd, self.return_turn)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
