"""ROS 2 serial bridge for a differential-drive Arduino controller."""

from math import pi
import threading
import time

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
from tf2_ros import TransformBroadcaster

from .kinematics import Pose2D, body_velocity, integrate, wheel_speeds


def yaw_quaternion(yaw):
    """Return a quaternion for a rotation around the Z axis."""
    return Quaternion(z=__import__("math").sin(yaw / 2.0), w=__import__("math").cos(yaw / 2.0))


class SerialBase(Node):
    """Exchange velocity commands and encoder counts with the motor controller."""

    def __init__(self):
        super().__init__("serial_base")
        self.declare_parameter("port", "/dev/ttyUSB_ARDUINO")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.225)
        self.declare_parameter("ticks_per_revolution", 600)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("publish_tf", True)

        self.radius = float(self.get_parameter("wheel_radius").value)
        self.separation = float(self.get_parameter("wheel_separation").value)
        self.ticks_per_rev = int(self.get_parameter("ticks_per_revolution").value)
        self.timeout = float(self.get_parameter("command_timeout").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.pose = Pose2D()
        self.target = (0.0, 0.0)
        self.last_command = time.monotonic()
        self.last_ticks = None
        self.last_encoder_time = None
        self.lock = threading.Lock()

        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 20)

        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        try:
            self.serial = Serial(port, baudrate, timeout=0.02)
        except SerialException as error:
            raise RuntimeError(f"Cannot open motor controller at {port}: {error}") from error

        self.create_timer(0.05, self.exchange)
        self.get_logger().info(f"Motor controller connected at {port} ({baudrate} baud)")

    def on_cmd_vel(self, message):
        left, right = wheel_speeds(message.linear.x, message.angular.z, self.separation)
        with self.lock:
            self.target = (left, right)
            self.last_command = time.monotonic()

    def exchange(self):
        now = time.monotonic()
        with self.lock:
            speeds = self.target if now - self.last_command <= self.timeout else (0.0, 0.0)
        try:
            self.serial.write(f"V {speeds[0]:.4f} {speeds[1]:.4f}\n".encode("ascii"))
            line = self.serial.readline().decode("ascii", errors="ignore").strip()
        except SerialException as error:
            self.get_logger().error(f"Serial communication failed: {error}")
            return
        if line.startswith("E "):
            self.update_odometry(line, now)

    def update_odometry(self, line, stamp):
        try:
            _, left_text, right_text = line.split()
            ticks = (int(left_text), int(right_text))
        except ValueError:
            self.get_logger().warning(f"Ignoring malformed encoder frame: {line}")
            return
        if self.last_ticks is None:
            self.last_ticks, self.last_encoder_time = ticks, stamp
            return

        dt = stamp - self.last_encoder_time
        if dt <= 0.0:
            return
        metres_per_tick = 2.0 * pi * self.radius / self.ticks_per_rev
        left = (ticks[0] - self.last_ticks[0]) * metres_per_tick / dt
        right = (ticks[1] - self.last_ticks[1]) * metres_per_tick / dt
        linear, angular = body_velocity(left, right, self.separation)
        integrate(self.pose, linear, angular, dt)
        self.last_ticks, self.last_encoder_time = ticks, stamp
        self.publish_odometry(linear, angular)

    def publish_odometry(self, linear, angular):
        timestamp = self.get_clock().now().to_msg()
        rotation = yaw_quaternion(self.pose.yaw)
        message = Odometry()
        message.header.stamp = timestamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self.pose.x
        message.pose.pose.position.y = self.pose.y
        message.pose.pose.orientation = rotation
        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular
        self.odom_pub.publish(message)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = timestamp
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = self.pose.x
            transform.transform.translation.y = self.pose.y
            transform.transform.rotation = rotation
            self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        if hasattr(self, "serial") and self.serial.is_open:
            self.serial.write(b"V 0.0 0.0\n")
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
