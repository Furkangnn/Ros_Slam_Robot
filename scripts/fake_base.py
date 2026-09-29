#!/usr/bin/env python3
"""A lightweight differential-drive base simulator for ROS-free hardware tests."""

from math import atan2, cos, sin
import random

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def clamp(value, lower, upper):
    """Restrict value to the inclusive range."""
    return max(lower, min(upper, value))


def move_towards(current, target, maximum_change):
    """Move current toward target without exceeding maximum_change."""
    error = target - current
    return current + clamp(error, -maximum_change, maximum_change)


def yaw_quaternion(yaw):
    """Create a geometry quaternion for a planar heading."""
    return Quaternion(z=sin(yaw / 2.0), w=cos(yaw / 2.0))


class FakeBase(Node):
    """Simulate command handling, acceleration and wheel odometry."""

    def __init__(self):
        super().__init__("fake_base")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.225)
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("linear_acceleration", 0.8)
        self.declare_parameter("angular_acceleration", 2.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("odometry_noise", 0.0)
        self.declare_parameter("publish_tf", True)

        self.radius = float(self.get_parameter("wheel_radius").value)
        self.separation = float(self.get_parameter("wheel_separation").value)
        self.rate = float(self.get_parameter("update_rate").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.linear_acceleration = float(
            self.get_parameter("linear_acceleration").value
        )
        self.angular_acceleration = float(
            self.get_parameter("angular_acceleration").value
        )
        self.timeout = float(self.get_parameter("command_timeout").value)
        self.noise = float(self.get_parameter("odometry_noise").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        if self.radius <= 0.0 or self.separation <= 0.0 or self.rate <= 0.0:
            raise ValueError("wheel dimensions and update rate must be positive")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.left_position = 0.0
        self.right_position = 0.0
        self.last_command = self.get_clock().now()
        self.last_update = self.get_clock().now()

        self.odom_publisher = self.create_publisher(Odometry, "odom", 20)
        self.joint_publisher = self.create_publisher(
            JointState, "joint_states", 20
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_command, 20)
        self.create_timer(1.0 / self.rate, self.update)
        self.get_logger().info(
            f"Fake base running at {self.rate:.1f} Hz; "
            "publish velocity commands on /cmd_vel"
        )

    def on_command(self, message):
        """Store a bounded target velocity."""
        self.target_linear = clamp(
            message.linear.x, -self.max_linear, self.max_linear
        )
        self.target_angular = clamp(
            message.angular.z, -self.max_angular, self.max_angular
        )
        self.last_command = self.get_clock().now()

    def update(self):
        """Advance simulation state and publish one sensor sample."""
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now
        if dt <= 0.0 or dt > 0.5:
            return

        command_age = (now - self.last_command).nanoseconds / 1e9
        target_linear = self.target_linear if command_age <= self.timeout else 0.0
        target_angular = (
            self.target_angular if command_age <= self.timeout else 0.0
        )
        self.linear = move_towards(
            self.linear, target_linear, self.linear_acceleration * dt
        )
        self.angular = move_towards(
            self.angular, target_angular, self.angular_acceleration * dt
        )

        measured_linear = self.linear + random.gauss(0.0, self.noise)
        measured_angular = self.angular + random.gauss(0.0, self.noise)
        midpoint_heading = self.yaw + measured_angular * dt / 2.0
        self.x += measured_linear * cos(midpoint_heading) * dt
        self.y += measured_linear * sin(midpoint_heading) * dt
        self.yaw = atan2(
            sin(self.yaw + measured_angular * dt),
            cos(self.yaw + measured_angular * dt),
        )

        left_speed = (
            measured_linear - measured_angular * self.separation / 2.0
        )
        right_speed = (
            measured_linear + measured_angular * self.separation / 2.0
        )
        self.left_position += left_speed / self.radius * dt
        self.right_position += right_speed / self.radius * dt
        self.publish_odometry(now, measured_linear, measured_angular)
        self.publish_joints(now, left_speed, right_speed)

    def publish_odometry(self, timestamp, linear, angular):
        """Publish odometry and optionally its TF equivalent."""
        rotation = yaw_quaternion(self.yaw)
        message = Odometry()
        message.header.stamp = timestamp.to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation = rotation
        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.04
        self.odom_publisher.publish(message)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = message.child_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = rotation
            self.transform_broadcaster.sendTransform(transform)

    def publish_joints(self, timestamp, left_speed, right_speed):
        """Publish wheel angle and velocity for robot_state_publisher."""
        message = JointState()
        message.header.stamp = timestamp.to_msg()
        message.name = ["left_wheel_joint", "right_wheel_joint"]
        message.position = [self.left_position, self.right_position]
        message.velocity = [left_speed / self.radius, right_speed / self.radius]
        self.joint_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FakeBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
