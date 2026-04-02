#!/usr/bin/env python3
"""
Navigation Controller Node — Improved Obstacle Avoidance
=========================================================
Arbitrates between line following and obstacle avoidance.

Key improvements over v1:
  - Uses obstacle distance for proportional response (slow down before stopping)
  - Uses obstacle direction to decide turn direction (turn AWAY from obstacle)
  - Checks side clearance during bypass to avoid secondary collisions
  - Emergency stop if obstacle is critically close
  - LIDAR-guided forward during bypass (stops if something is in the way)
  - Shorter, tuned timings for the bypass maneuver

States:
  FOLLOW_LINE     → Normal line following with obstacle distance monitoring
  SLOW_DOWN       → Obstacle nearby: reduce speed, prepare
  STOP            → Obstacle close: full stop
  TURN_AWAY       → Turn away from obstacle (direction based on LIDAR)
  BYPASS_FORWARD  → Drive forward to clear the obstacle
  TURN_BACK       → Turn back toward the line
  APPROACH_LINE   → Drive forward to reach the line
  TURN_TO_LINE    → Final turn to align with line
  RETURN_TO_LINE  → Seek the line by driving forward slowly
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String
import math


class State:
    FOLLOW_LINE = 'FOLLOW_LINE'
    SLOW_DOWN = 'SLOW_DOWN'
    STOP = 'STOP'
    TURN_AWAY = 'TURN_AWAY'
    BYPASS_FORWARD = 'BYPASS_FORWARD'
    TURN_BACK = 'TURN_BACK'
    APPROACH_LINE = 'APPROACH_LINE'
    TURN_TO_LINE = 'TURN_TO_LINE'
    RETURN_TO_LINE = 'RETURN_TO_LINE'


class NavigationControllerNode(Node):

    def __init__(self):
        super().__init__('navigation_controller_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('stop_duration', 0.5)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('turn_duration_90', 2.5)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('bypass_forward_duration', 1.5)
        self.declare_parameter('approach_line_duration', 2.5)
        self.declare_parameter('return_speed', 0.12)
        self.declare_parameter('return_timeout', 8.0)
        self.declare_parameter('slowdown_distance', 0.60)
        self.declare_parameter('slowdown_speed', 0.08)

        # Get parameters
        self.stop_duration = self.get_parameter('stop_duration').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_duration_90 = self.get_parameter('turn_duration_90').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.bypass_fwd_dur = self.get_parameter('bypass_forward_duration').value
        self.approach_dur = self.get_parameter('approach_line_duration').value
        self.return_speed = self.get_parameter('return_speed').value
        self.return_timeout = self.get_parameter('return_timeout').value
        self.slowdown_distance = self.get_parameter('slowdown_distance').value
        self.slowdown_speed = self.get_parameter('slowdown_speed').value

        # ── State machine ───────────────────────────────────────────────
        self.state = State.FOLLOW_LINE
        self.state_start_time = self.get_clock().now()

        # ── Turn direction: +1 = left, -1 = right ──────────────────────
        self.avoid_direction = 1.0  # default: turn left

        # ── Cached sensor data ──────────────────────────────────────────
        self.line_cmd_vel = Twist()
        self.obstacle_detected = False
        self.obstacle_emergency = False
        self.obstacle_distance = float('inf')
        self.obstacle_direction = 'NONE'
        self.line_detected = False
        self.left_clear = True
        self.right_clear = True

        # ── Publisher ───────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(Twist, '/line_cmd_vel', self._cb_line_cmd, 10)
        self.create_subscription(Bool, '/obstacle_detected', self._cb_obstacle, 10)
        self.create_subscription(Bool, '/obstacle_emergency', self._cb_emergency, 10)
        self.create_subscription(Float32, '/obstacle_distance', self._cb_distance, 10)
        self.create_subscription(String, '/obstacle_direction', self._cb_direction, 10)
        self.create_subscription(Bool, '/line_detected', self._cb_line_detected, 10)
        self.create_subscription(Bool, '/left_side_clear', self._cb_left_clear, 10)
        self.create_subscription(Bool, '/right_side_clear', self._cb_right_clear, 10)

        # ── Timer (20 Hz) ───────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info('Navigation Controller (improved) started')

    # ── Callbacks ───────────────────────────────────────────────────────
    def _cb_line_cmd(self, msg):
        self.line_cmd_vel = msg

    def _cb_obstacle(self, msg):
        self.obstacle_detected = msg.data

    def _cb_emergency(self, msg):
        self.obstacle_emergency = msg.data

    def _cb_distance(self, msg):
        self.obstacle_distance = msg.data

    def _cb_direction(self, msg):
        self.obstacle_direction = msg.data

    def _cb_line_detected(self, msg):
        self.line_detected = msg.data

    def _cb_left_clear(self, msg):
        self.left_clear = msg.data

    def _cb_right_clear(self, msg):
        self.right_clear = msg.data

    # ── Helpers ─────────────────────────────────────────────────────────
    def elapsed(self):
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

    def go_to(self, new_state):
        self.get_logger().info(f'[NAV] {self.state} → {new_state}')
        self.state = new_state
        self.state_start_time = self.get_clock().now()

    def decide_turn_direction(self):
        """Choose which way to turn based on obstacle direction and side clearance."""
        if self.obstacle_direction == 'LEFT':
            # Obstacle is on our left → turn right to avoid
            if self.right_clear:
                self.avoid_direction = -1.0
            else:
                self.avoid_direction = 1.0  # fallback
        elif self.obstacle_direction == 'RIGHT':
            # Obstacle is on our right → turn left to avoid
            if self.left_clear:
                self.avoid_direction = 1.0
            else:
                self.avoid_direction = -1.0  # fallback
        else:
            # Center or unknown → pick whichever side has more clearance
            if self.left_clear:
                self.avoid_direction = 1.0
            else:
                self.avoid_direction = -1.0

        direction_name = "LEFT" if self.avoid_direction > 0 else "RIGHT"
        self.get_logger().info(f'[NAV] Turning {direction_name} to avoid obstacle')

    # ── State Machine ───────────────────────────────────────────────────
    def update(self):
        twist = Twist()
        dt = self.elapsed()

        # ── EMERGENCY OVERRIDE (any state) ──────────────────────────────
        if self.obstacle_emergency and self.state in (State.FOLLOW_LINE, State.SLOW_DOWN):
            self.decide_turn_direction()
            self.go_to(State.STOP)
            self.cmd_pub.publish(Twist())  # instant stop
            return

        # ────────────────────────────────────────────────────────────────
        if self.state == State.FOLLOW_LINE:
            twist = self.line_cmd_vel

            # Check if obstacle is approaching
            if self.obstacle_detected:
                self.decide_turn_direction()
                self.go_to(State.STOP)
                twist = Twist()
            elif self.obstacle_distance < self.slowdown_distance:
                # Obstacle nearby but not critical — slow down
                twist.linear.x = self.slowdown_speed
                # Keep angular from line tracker for steering

        # ────────────────────────────────────────────────────────────────
        elif self.state == State.STOP:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if dt >= self.stop_duration:
                self.go_to(State.TURN_AWAY)

        # ────────────────────────────────────────────────────────────────
        elif self.state == State.TURN_AWAY:
            # Turn away from obstacle (direction decided earlier)
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed * self.avoid_direction

            if dt >= self.turn_duration_90:
                self.go_to(State.BYPASS_FORWARD)

        # ────────────────────────────────────────────────────────────────
        elif self.state == State.BYPASS_FORWARD:
            # Drive forward to move beside the obstacle
            # BUT check if something is in front — don't drive into walls
            if self.obstacle_distance < 0.25:
                # Something is in the way — stop and re-turn
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed * self.avoid_direction * 0.5
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0

            if dt >= self.bypass_fwd_dur:
                self.go_to(State.TURN_BACK)

        # ────────────────────────────────────────────────────────────────
        elif self.state == State.TURN_BACK:
            # Turn back toward the original path direction
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed * self.avoid_direction

            if dt >= self.turn_duration_90:
                self.go_to(State.APPROACH_LINE)

        # ────────────────────────────────────────────────────────────────
        elif self.state == State.APPROACH_LINE:
            # Drive forward past the obstacle
            if self.obstacle_distance < 0.25:
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed * self.avoid_direction * 0.5
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0

            if dt >= self.approach_dur:
                self.go_to(State.TURN_TO_LINE)

        # ────────────────────────────────────────────────────────────────
        elif self.state == State.TURN_TO_LINE:
            # Turn back toward the line
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed * self.avoid_direction

            if dt >= self.turn_duration_90:
                self.go_to(State.RETURN_TO_LINE)

        # ────────────────────────────────────────────────────────────────
        elif self.state == State.RETURN_TO_LINE:
            # Drive forward slowly until line is detected
            twist.linear.x = self.return_speed
            twist.angular.z = 0.0

            if self.line_detected:
                self.get_logger().info('[NAV] ✓ LINE RE-ACQUIRED!')
                self.go_to(State.FOLLOW_LINE)
            elif dt >= self.return_timeout:
                self.get_logger().warn('[NAV] Return timeout — resuming line follow')
                self.go_to(State.FOLLOW_LINE)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
