#!/usr/bin/env python3
"""
controller.py  —  FSM with committed rectangular bypass + cooldown
===================================================================
States:
  FOLLOW_LINE       : PID line tracking
  OBSTACLE_DETECTED : immediate stop + pick turn direction
  AVOID_OBSTACLE    : 4-phase committed bypass maneuver
  SEARCH_LINE       : spin BACK toward line + drive to re-acquire

Key fix: After avoidance, the robot often re-acquires the line while
still alongside the obstacle, causing an immediate re-trigger loop.
Solution:
  - Phase 4 only exits early if line visible AND obstacle NOT detected
  - SEARCH_LINE only resumes following if obstacle is NOT in front
  - After avoidance → FOLLOW_LINE, a 3-second cooldown suppresses
    obstacle re-triggering to let the robot drive past the obstacle
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
        self.obs_threshold = self.declare_parameter(
            'obstacle_distance_threshold', 0.55).value

        # Phase durations (real-time seconds)
        self.phase1_turn_dur = self.declare_parameter(
            'phase1_turn_duration',   0.78).value   # Turn away ~90°
        self.phase2_fwd_dur  = self.declare_parameter(
            'phase2_forward_duration', 2.0).value   # Drive alongside: 0.40m
        self.phase3_turn_dur = self.declare_parameter(
            'phase3_turn_duration',   0.78).value   # Turn back ~90°
        self.phase4_fwd_dur  = self.declare_parameter(
            'phase4_forward_duration', 2.5).value   # Drive past: 0.50m

        # Physical speeds
        self.avoid_turn_spd   = 2.00   # rad/s — turn in place
        self.avoid_fwd_spd    = 0.20   # m/s   — forward during bypass
        self.search_rot_spd   = 0.80   # rad/s — spin while searching line
        self.search_creep_spd = 0.10   # m/s   — creep forward while searching

        # Emergency collision distance
        self.emergency_dist = 0.12

        # Search timeout — after this, drive forward faster
        self.search_timeout = 6.0

        # Cooldown after avoidance — suppress obstacle re-trigger
        self.avoidance_cooldown = 3.0   # seconds

        # ── Internal state ────────────────────────────────────────────────────
        self.state            = FOLLOW_LINE
        self.state_entry_time = time.time()

        self.line_error        = float('nan')
        self.obstacle_detected = False
        self.obs_min_dist      = 9.99
        self.left_min          = 9.99
        self.right_min         = 9.99

        self.turn_sign = -1   # +1=left, -1=right

        # Cooldown timer: when > current time, obstacle detection suppressed
        self.cooldown_until = 0.0

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
            f'Controller ready | threshold={self.obs_threshold}m | '
            f'bypass: turn1={self.phase1_turn_dur}s '
            f'fwd1={self.phase2_fwd_dur}s '
            f'turn2={self.phase3_turn_dur}s '
            f'fwd2={self.phase4_fwd_dur}s | '
            f'fwd_spd={self.avoid_fwd_spd}m/s | '
            f'cooldown={self.avoidance_cooldown}s')

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

    def in_cooldown(self) -> bool:
        return time.time() < self.cooldown_until

    def start_cooldown(self):
        """Start post-avoidance cooldown to suppress obstacle re-trigger."""
        self.cooldown_until = time.time() + self.avoidance_cooldown
        self.get_logger().info(
            f'Cooldown started ({self.avoidance_cooldown}s) — '
            f'obstacle detection suppressed')

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        elapsed = time.time() - self.state_entry_time

        # ════════════════════════════════════════════════════════════════════
        # STATE: FOLLOW_LINE
        # PID line tracking.
        # Obstacle detection is suppressed during cooldown period.
        # ════════════════════════════════════════════════════════════════════
        if self.state == FOLLOW_LINE:

            # Check obstacle only if NOT in cooldown
            if self.obstacle_detected and not self.in_cooldown():
                self.publish_vel(0.0, 0.0)
                self.transition(OBSTACLE_DETECTED)
                return

            # During cooldown, still warn about very close obstacles
            if self.in_cooldown() and self.obs_min_dist < 0.20:
                self.publish_vel(0.0, 0.0)
                self.get_logger().warn(
                    f'Emergency during cooldown: obs at '
                    f'{self.obs_min_dist:.2f}m → OBSTACLE_DETECTED')
                self.cooldown_until = 0.0  # cancel cooldown
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
        # 4-phase COMMITTED rectangular bypass.
        # Phase 4 only exits early if line visible AND obstacle clear.
        # ════════════════════════════════════════════════════════════════════
        elif self.state == AVOID_OBSTACLE:

            t = elapsed

            # Phase boundary times
            p1_end = self.phase1_turn_dur
            p2_end = p1_end + self.phase2_fwd_dur
            p3_end = p2_end + self.phase3_turn_dur
            p4_end = p3_end + self.phase4_fwd_dur

            # ── Emergency collision check (any phase) ─────────────────────
            if self.obs_min_dist < self.emergency_dist:
                self.publish_vel(0.0, 0.0)
                self.get_logger().warn(
                    f'EMERGENCY: obstacle at {self.obs_min_dist:.2f}m — '
                    f'stopping and re-evaluating')
                self.transition(OBSTACLE_DETECTED)
                return

            if t < p1_end:
                # ── Phase 1: Turn AWAY from obstacle ──────────────────────
                self.publish_vel(0.0, self.turn_sign * self.avoid_turn_spd)

            elif t < p2_end:
                # ── Phase 2: Drive STRAIGHT forward ───────────────────────
                self.publish_vel(self.avoid_fwd_spd, 0.0)

            elif t < p3_end:
                # ── Phase 3: Turn BACK toward line ────────────────────────
                self.publish_vel(0.0, -self.turn_sign * self.avoid_turn_spd)

            elif t < p4_end:
                # ── Phase 4: Drive STRAIGHT toward line ───────────────────
                # Only exit early if line visible AND obstacle is CLEAR
                if (self.line_visible() and
                        not self.obstacle_detected and
                        self.obs_min_dist > self.obs_threshold):
                    self.get_logger().info(
                        f'Line found + obstacle clear in phase 4 '
                        f'(error={self.line_error:.0f}px, '
                        f'obs={self.obs_min_dist:.2f}m) → FOLLOW_LINE')
                    self.start_cooldown()
                    self.transition(FOLLOW_LINE)
                    return
                self.publish_vel(self.avoid_fwd_spd, 0.0)

            else:
                # ── All 4 phases done → search for line ───────────────────
                self.publish_vel(0.0, 0.0)
                self.get_logger().info('Bypass complete → SEARCH_LINE')
                self.transition(SEARCH_LINE)

        # ════════════════════════════════════════════════════════════════════
        # STATE: SEARCH_LINE
        # Spin OPPOSITE to avoidance turn while creeping forward.
        # Only resume line following when obstacle is NOT in front.
        # ════════════════════════════════════════════════════════════════════
        elif self.state == SEARCH_LINE:

            # Emergency: very close obstacle
            if self.obs_min_dist < 0.20:
                self.publish_vel(0.0, 0.0)
                self.get_logger().info(
                    f'Obstacle at {self.obs_min_dist:.2f}m during search '
                    f'→ re-evaluate')
                self.transition(OBSTACLE_DETECTED)
                return

            # Re-acquire line ONLY if obstacle is NOT detected in front
            if self.line_visible() and not self.obstacle_detected:
                self.get_logger().info(
                    f'Line re-acquired + clear path | '
                    f'error={self.line_error:.1f}px '
                    f'obs={self.obs_min_dist:.2f}m → FOLLOW_LINE')
                self.start_cooldown()
                self.transition(FOLLOW_LINE)
                return

            # If line is visible but obstacle is still in front,
            # keep moving to get past the obstacle first
            if self.line_visible() and self.obstacle_detected:
                # Drive forward to get past the obstacle
                self.publish_vel(self.search_creep_spd * 1.5, 0.0)
                if int(elapsed * 20) % 40 == 0:
                    self.get_logger().info(
                        f'Line visible but obstacle still ahead '
                        f'({self.obs_min_dist:.2f}m) — driving past...')
                return

            # Spin OPPOSITE to avoidance turn + creep forward
            spin = -self.turn_sign * self.search_rot_spd

            if elapsed < self.search_timeout:
                self.publish_vel(self.search_creep_spd, spin)
            else:
                # Extended search: drive forward faster
                self.publish_vel(self.line_speed, spin * 0.5)

            if int(elapsed * 20) % 40 == 0:
                self.get_logger().info(
                    f'Searching... ({elapsed:.1f}s) '
                    f'obs={self.obs_min_dist:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()