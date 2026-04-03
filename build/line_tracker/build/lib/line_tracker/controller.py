#!/usr/bin/env python3
"""
Robot Controller Node — FSM + PID + Bug0
-----------------------------------------
States
------
  LINE_FOLLOW  : PID-driven line tracking  (normal operation)
  STOP         : Brief halt when obstacle first detected
  AVOID        : Bug0 wall-bypass (turn right, then move forward until clear)
  RETURN       : Slow sweep left to re-acquire the line

Transitions
-----------
  LINE_FOLLOW ──[obstacle]──► STOP
  STOP        ──[0.5 s]────► AVOID
  AVOID       ──[clear + 2 s forward]──► RETURN
  RETURN      ──[line reacquired]──► LINE_FOLLOW

Topics
------
  Subscribed : /line_error   (std_msgs/Float32)
               /obstacle_flag (std_msgs/Bool)
  Published  : /cmd_vel      (geometry_msgs/Twist)
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool


# ── State constants ────────────────────────────────────────────────────
LINE_FOLLOW = 0
STOP        = 1
AVOID       = 2
RETURN      = 3

STATE_NAMES = {LINE_FOLLOW: 'LINE_FOLLOW', STOP: 'STOP',
               AVOID: 'AVOID', RETURN: 'RETURN'}


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # ── ROS parameters ──────────────────────────────────────────────
        self.declare_parameter('Kp', 0.005)
        self.declare_parameter('Ki', 0.0001)
        self.declare_parameter('Kd', 0.001)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('stop_hold_s', 0.5)
        self.declare_parameter('avoid_fwd_s', 2.0)
        self.declare_parameter('control_hz', 20.0)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular  = self.get_parameter('max_angular').value
        self.stop_hold_s  = self.get_parameter('stop_hold_s').value
        self.avoid_fwd_s  = self.get_parameter('avoid_fwd_s').value
        hz = self.get_parameter('control_hz').value

        # ── PID internal state ──────────────────────────────────────────
        self.prev_error  = 0.0
        self.integral    = 0.0
        self.prev_time   = self.get_clock().now()

        # ── FSM state ──────────────────────────────────────────────────
        self.state       = LINE_FOLLOW
        self.line_error  = 0.0           # latest from /line_error
        self.obstacle    = False         # latest from /obstacle_flag
        self.stop_start  = None          # wall-clock time of STOP entry
        self.avoid_clear_start = None    # time when path became clear in AVOID

        # ── Subscribers ────────────────────────────────────────────────
        self.create_subscription(Float32, '/line_error',    self.line_cb,     10)
        self.create_subscription(Bool,    '/obstacle_flag', self.obstacle_cb, 10)

        # ── Publisher ──────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Control loop ───────────────────────────────────────────────
        self.create_timer(1.0 / hz, self.control_loop)

        self.get_logger().info(
            f'Controller started — Kp={self.Kp} Ki={self.Ki} Kd={self.Kd} '
            f'v={self.linear_speed} m/s  @ {hz} Hz')

    # ── Callbacks ──────────────────────────────────────────────────────
    def line_cb(self, msg: Float32):
        self.line_error = msg.data

    def obstacle_cb(self, msg: Bool):
        self.obstacle = msg.data

    # ── PID ────────────────────────────────────────────────────────────
    def _pid(self, error: float) -> float:
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9
        dt  = max(dt, 1e-4)   # guard against zero dt

        self.integral   += error * dt
        # Anti-windup clamp
        self.integral    = max(-500.0, min(500.0, self.integral))

        derivative       = (error - self.prev_error) / dt
        output           = (self.Kp * error +
                            self.Ki * self.integral +
                            self.Kd * derivative)

        self.prev_error  = error
        self.prev_time   = now
        return output

    # ── Velocity publisher ─────────────────────────────────────────────
    def _publish(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    # ── State transition helper ─────────────────────────────────────────
    def _transition(self, new_state: int):
        self.get_logger().info(
            f'{STATE_NAMES[self.state]} → {STATE_NAMES[new_state]}')
        self.state = new_state

    # ── Main control loop ──────────────────────────────────────────────
    def control_loop(self):
        now = time.monotonic()

        # ━━ LINE_FOLLOW ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if self.state == LINE_FOLLOW:
            if self.obstacle:
                self._publish(0.0, 0.0)
                self.stop_start = now
                self._transition(STOP)
                return

            if math.isnan(self.line_error):
                # Line lost — creep forward and sweep slightly
                self._publish(0.05, 0.25)
                return

            angular = -self._pid(self.line_error)
            angular = max(-self.max_angular, min(self.max_angular, angular))
            self._publish(self.linear_speed, angular)

        # ━━ STOP ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.state == STOP:
            self._publish(0.0, 0.0)
            if now - self.stop_start >= self.stop_hold_s:
                self.avoid_clear_start = None
                self._transition(AVOID)

        # ━━ AVOID (Bug0) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.state == AVOID:
            if self.obstacle:
                # Still blocked — turn right to go around
                self.avoid_clear_start = None
                self._publish(0.05, -0.8)
            else:
                # Path clear — move straight, start timer
                if self.avoid_clear_start is None:
                    self.avoid_clear_start = now
                self._publish(self.linear_speed, 0.0)
                if now - self.avoid_clear_start >= self.avoid_fwd_s:
                    # Reset PID integral before returning to line follow
                    self.integral = 0.0
                    self._transition(RETURN)

        # ━━ RETURN ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        elif self.state == RETURN:
            if not math.isnan(self.line_error):
                self.integral = 0.0
                self._transition(LINE_FOLLOW)
            else:
                # Slowly sweep left to find line
                self._publish(0.08, 0.5)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
