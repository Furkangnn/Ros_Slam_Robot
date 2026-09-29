#!/usr/bin/env python3
"""Monitor odometry quality and print a compact terminal dashboard."""

from collections import deque
from dataclasses import dataclass
from math import atan2, degrees, hypot
import statistics
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass
class Sample:
    """One timestamped odometry observation."""

    received_at: float
    x: float
    y: float
    linear: float
    angular: float


class OdomMonitor(Node):
    """Report odometry frequency, travel distance and lidar freshness."""

    def __init__(self):
        super().__init__("odom_monitor")
        self.declare_parameter("report_interval", 1.0)
        self.declare_parameter("stale_after", 1.0)
        self.declare_parameter("history_size", 200)

        interval = float(self.get_parameter("report_interval").value)
        self.stale_after = float(self.get_parameter("stale_after").value)
        history_size = int(self.get_parameter("history_size").value)
        if interval <= 0.0 or self.stale_after <= 0.0 or history_size < 2:
            raise ValueError("monitor parameters must be positive")

        self.samples = deque(maxlen=history_size)
        self.total_distance = 0.0
        self.last_scan_time = None
        self.scan_ranges = 0
        self.scan_minimum = float("inf")
        self.started_at = time.monotonic()

        self.create_subscription(Odometry, "odom", self.on_odometry, 50)
        self.create_subscription(LaserScan, "scan", self.on_scan, 10)
        self.create_timer(interval, self.report)
        self.get_logger().info("Monitoring /odom and /scan")

    def on_odometry(self, message):
        """Record an odometry message and accumulate travelled distance."""
        sample = Sample(
            received_at=time.monotonic(),
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            linear=message.twist.twist.linear.x,
            angular=message.twist.twist.angular.z,
        )
        if self.samples:
            previous = self.samples[-1]
            movement = hypot(sample.x - previous.x, sample.y - previous.y)
            if movement < 0.5:
                self.total_distance += movement
            else:
                self.get_logger().warning(
                    f"Odometry jump ignored: {movement:.2f} m"
                )
        self.samples.append(sample)

    def on_scan(self, message):
        """Record lidar message age, size and nearest finite return."""
        self.last_scan_time = time.monotonic()
        valid = [
            distance
            for distance in message.ranges
            if message.range_min <= distance <= message.range_max
        ]
        self.scan_ranges = len(valid)
        self.scan_minimum = min(valid, default=float("inf"))

    def odometry_rate(self):
        """Estimate receive frequency from the current history window."""
        if len(self.samples) < 2:
            return 0.0
        elapsed = self.samples[-1].received_at - self.samples[0].received_at
        return (len(self.samples) - 1) / elapsed if elapsed > 0.0 else 0.0

    def odometry_jitter(self):
        """Return standard deviation of odometry message intervals."""
        if len(self.samples) < 3:
            return 0.0
        intervals = [
            current.received_at - previous.received_at
            for previous, current in zip(self.samples, list(self.samples)[1:])
        ]
        return statistics.pstdev(intervals) * 1000.0

    def report(self):
        """Print one dashboard line and warn about stale sensor streams."""
        now = time.monotonic()
        if self.samples:
            latest = self.samples[-1]
            odom_age = now - latest.received_at
            heading = self.heading_from_velocity(latest.linear, latest.angular)
            position = f"x={latest.x:+.2f} y={latest.y:+.2f}"
            motion = (
                f"v={latest.linear:+.2f}m/s "
                f"w={degrees(latest.angular):+.1f}deg/s "
                f"motion={heading}"
            )
        else:
            odom_age = now - self.started_at
            position = "x=-- y=--"
            motion = "v=-- w=-- motion=unknown"

        scan_age = (
            now - self.last_scan_time
            if self.last_scan_time is not None
            else now - self.started_at
        )
        nearest = (
            f"{self.scan_minimum:.2f}m"
            if self.scan_minimum != float("inf")
            else "--"
        )
        self.get_logger().info(
            f"{position} | {motion} | "
            f"odom={self.odometry_rate():.1f}Hz "
            f"jitter={self.odometry_jitter():.1f}ms | "
            f"travel={self.total_distance:.2f}m | "
            f"scan={self.scan_ranges} nearest={nearest}"
        )

        if odom_age > self.stale_after:
            self.get_logger().warning(
                f"/odom is stale ({odom_age:.1f} seconds without data)"
            )
        if scan_age > self.stale_after:
            self.get_logger().warning(
                f"/scan is stale ({scan_age:.1f} seconds without data)"
            )

    @staticmethod
    def heading_from_velocity(linear, angular):
        """Describe current motion for a human-readable status line."""
        if abs(linear) < 0.01 and abs(angular) < 0.02:
            return "stopped"
        if abs(linear) < 0.01:
            return "turning-left" if angular > 0.0 else "turning-right"
        if linear < 0.0:
            return "reverse"
        curvature = atan2(angular, max(abs(linear), 1e-6))
        if abs(curvature) < 0.1:
            return "forward"
        return "forward-left" if angular > 0.0 else "forward-right"


def main(args=None):
    rclpy.init(args=args)
    node = OdomMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
