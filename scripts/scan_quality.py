#!/usr/bin/env python3
"""Measure RPLIDAR scan coverage and stability from the command line."""

from collections import deque
from math import degrees, isfinite
import statistics
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanQuality(Node):
    """Summarize lidar frequency, valid returns and nearest obstacles."""

    def __init__(self):
        super().__init__("scan_quality")
        self.declare_parameter("report_interval", 2.0)
        self.declare_parameter("history_size", 30)
        self.declare_parameter("warning_coverage", 0.60)

        interval = float(self.get_parameter("report_interval").value)
        history_size = int(self.get_parameter("history_size").value)
        self.warning_coverage = float(
            self.get_parameter("warning_coverage").value
        )
        if interval <= 0.0 or history_size < 2:
            raise ValueError("interval must be positive and history_size >= 2")

        self.timestamps = deque(maxlen=history_size)
        self.coverages = deque(maxlen=history_size)
        self.nearest_ranges = deque(maxlen=history_size)
        self.last_scan = None
        self.create_subscription(
            LaserScan, "scan", self.on_scan, rclpy.qos.qos_profile_sensor_data
        )
        self.create_timer(interval, self.report)
        self.get_logger().info("Listening for LaserScan messages on /scan")

    def on_scan(self, message):
        """Store quality values from one complete laser revolution."""
        now = time.monotonic()
        valid = [
            value
            for value in message.ranges
            if isfinite(value) and message.range_min <= value <= message.range_max
        ]
        total = len(message.ranges)
        self.timestamps.append(now)
        self.coverages.append(len(valid) / total if total else 0.0)
        self.nearest_ranges.append(min(valid, default=float("inf")))
        self.last_scan = message

    def frequency(self):
        """Estimate scan frequency over the retained time window."""
        if len(self.timestamps) < 2:
            return 0.0
        elapsed = self.timestamps[-1] - self.timestamps[0]
        return (len(self.timestamps) - 1) / elapsed if elapsed > 0.0 else 0.0

    def report(self):
        """Print a compact report and warn when scan coverage is weak."""
        if not self.coverages or self.last_scan is None:
            self.get_logger().warning("No /scan messages received")
            return

        coverage = statistics.fmean(self.coverages)
        minimum = min(self.nearest_ranges)
        finite_nearest = [value for value in self.nearest_ranges if isfinite(value)]
        median = (
            statistics.median(finite_nearest)
            if finite_nearest
            else float("inf")
        )
        field_of_view = degrees(
            self.last_scan.angle_max - self.last_scan.angle_min
        )
        points = len(self.last_scan.ranges)
        nearest_text = f"{minimum:.2f} m" if isfinite(minimum) else "none"
        median_text = f"{median:.2f} m" if isfinite(median) else "none"

        self.get_logger().info(
            f"rate={self.frequency():.1f} Hz | "
            f"coverage={coverage * 100:.1f}% | "
            f"points={points} | fov={field_of_view:.1f} deg | "
            f"nearest={nearest_text} | median-nearest={median_text}"
        )
        if coverage < self.warning_coverage:
            self.get_logger().warning(
                "Low lidar coverage: clean the sensor window and check "
                "range_min/range_max settings"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScanQuality()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
