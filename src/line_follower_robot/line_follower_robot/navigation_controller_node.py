#!/usr/bin/env python3
"""
Navigation Controller Node (State Machine)
============================================
Arbitrates between line following and obstacle avoidance.
Subscribes to line tracker commands and obstacle detector alerts,
publishes final /cmd_vel to drive the robot.

States:
  FOLLOW_LINE     → Normal: forward /line_cmd_vel to /cmd_vel
  STOP            → Obstacle detected: stop the robot for 1 second
  TURN_LEFT       → Turn left ~90° to begin bypassing
  GO_FORWARD_1    → Drive forward to move beside the obstacle
  TURN_RIGHT_1    → Turn right ~90° to face parallel to original path
  GO_FORWARD_2    → Drive forward past the obstacle
  TURN_RIGHT_2    → Turn right ~90° to face back toward the line
  RETURN_TO_LINE  → Drive forward slowly until line is re-detected
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
import math


class State:
    FOLLOW_LINE = 'FOLLOW_LINE'
    STOP = 'STOP'
    TURN_LEFT = 'TURN_LEFT'
    GO_FORWARD_1 = 'GO_FORWARD_1'
    TURN_RIGHT_1 = 'TURN_RIGHT_1'
    GO_FORWARD_2 = 'GO_FORWARD_2'
    TURN_RIGHT_2 = 'TURN_RIGHT_2'
    RETURN_TO_LINE = 'RETURN_TO_LINE'


class NavigationControllerNode(Node):

    def __init__(self):
        super().__init__('navigation_controller_node')

        # Declare parameters
        self.declare_parameter('stop_duration', 1.0)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('turn_duration_90', 3.0)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('bypass_forward_duration_1', 2.0)
        self.declare_parameter('bypass_forward_duration_2', 3.5)
        self.declare_parameter('return_speed', 0.10)
        self.declare_parameter('return_timeout', 10.0)

        # Get parameters
        self.stop_duration = self.get_parameter('stop_duration').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_duration_90 = self.get_parameter('turn_duration_90').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.bypass_fwd_dur_1 = self.get_parameter('bypass_forward_duration_1').value
        self.bypass_fwd_dur_2 = self.get_parameter('bypass_forward_duration_2').value
        self.return_speed = self.get_parameter('return_speed').value
        self.return_timeout = self.get_parameter('return_timeout').value

        # State machine
        self.state = State.FOLLOW_LINE
        self.state_start_time = self.get_clock().now()

        # Cached data
        self.line_cmd_vel = Twist()
        self.obstacle_detected = False
        self.line_detected = False

        # Publisher for final velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.line_cmd_sub = self.create_subscription(
            Twist, '/line_cmd_vel', self.line_cmd_callback, 10
        )
        self.obstacle_sub = self.create_subscription(
            Bool, '/obstacle_detected', self.obstacle_callback, 10
        )
        self.line_detected_sub = self.create_subscription(
            Bool, '/line_detected', self.line_detected_callback, 10
        )

        # Timer for state machine update (20 Hz)
        self.timer = self.create_timer(0.05, self.state_machine_update)

        self.get_logger().info('Navigation Controller Node started')
        self.get_logger().info(f'  State: {self.state}')

    def line_cmd_callback(self, msg):
        """Cache the latest line-following command."""
        self.line_cmd_vel = msg

    def obstacle_callback(self, msg):
        """Cache obstacle detection status."""
        self.obstacle_detected = msg.data

    def line_detected_callback(self, msg):
        """Cache line detection status."""
        self.line_detected = msg.data

    def elapsed_since_state_start(self):
        """Seconds elapsed since the current state began."""
        now = self.get_clock().now()
        return (now - self.state_start_time).nanoseconds / 1e9

    def transition_to(self, new_state):
        """Transition to a new state."""
        self.get_logger().info(f'State transition: {self.state} → {new_state}')
        self.state = new_state
        self.state_start_time = self.get_clock().now()

    def state_machine_update(self):
        """Main state machine loop."""
        twist = Twist()
        elapsed = self.elapsed_since_state_start()

        if self.state == State.FOLLOW_LINE:
            # Normal line following — pass through the line tracker commands
            twist = self.line_cmd_vel

            # Check for obstacle
            if self.obstacle_detected:
                self.transition_to(State.STOP)
                twist = Twist()  # Immediate stop

        elif self.state == State.STOP:
            # Stop for a brief period
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if elapsed >= self.stop_duration:
                self.transition_to(State.TURN_LEFT)

        elif self.state == State.TURN_LEFT:
            # Turn left ~90 degrees
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed  # Positive = left turn

            if elapsed >= self.turn_duration_90:
                self.transition_to(State.GO_FORWARD_1)

        elif self.state == State.GO_FORWARD_1:
            # Drive forward to move beside the obstacle
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

            if elapsed >= self.bypass_fwd_dur_1:
                self.transition_to(State.TURN_RIGHT_1)

        elif self.state == State.TURN_RIGHT_1:
            # Turn right ~90 degrees to face parallel to original path
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed  # Negative = right turn

            if elapsed >= self.turn_duration_90:
                self.transition_to(State.GO_FORWARD_2)

        elif self.state == State.GO_FORWARD_2:
            # Drive forward past the obstacle
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

            if elapsed >= self.bypass_fwd_dur_2:
                self.transition_to(State.TURN_RIGHT_2)

        elif self.state == State.TURN_RIGHT_2:
            # Turn right ~90 degrees to face back toward the line
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed  # Negative = right turn

            if elapsed >= self.turn_duration_90:
                self.transition_to(State.RETURN_TO_LINE)

        elif self.state == State.RETURN_TO_LINE:
            # Drive forward slowly until line is detected by camera
            twist.linear.x = self.return_speed
            twist.angular.z = 0.0

            if self.line_detected:
                self.get_logger().info('LINE RE-ACQUIRED! Resuming line following.')
                self.transition_to(State.FOLLOW_LINE)
            elif elapsed >= self.return_timeout:
                # Timeout — assume we've passed the obstacle, resume line following
                self.get_logger().warn(
                    'RETURN_TO_LINE timeout reached. Resuming line following anyway.'
                )
                self.transition_to(State.FOLLOW_LINE)

        # Publish the final velocity command
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
