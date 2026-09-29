"""Differential-drive kinematics without ROS dependencies."""

from dataclasses import dataclass
from math import cos, sin


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def wheel_speeds(linear: float, angular: float, wheel_separation: float):
    """Convert body velocity to left and right wheel linear velocity."""
    half_turn = angular * wheel_separation / 2.0
    return linear - half_turn, linear + half_turn


def body_velocity(left: float, right: float, wheel_separation: float):
    """Convert left and right wheel velocity to body velocity."""
    return (left + right) / 2.0, (right - left) / wheel_separation


def integrate(pose: Pose2D, linear: float, angular: float, dt: float):
    """Advance a planar pose with midpoint integration."""
    heading = pose.yaw + angular * dt / 2.0
    pose.x += linear * cos(heading) * dt
    pose.y += linear * sin(heading) * dt
    pose.yaw += angular * dt
    return pose
