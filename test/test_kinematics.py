from math import isclose

from ros_slam_robot.kinematics import Pose2D, body_velocity, integrate, wheel_speeds


def test_straight_wheel_speeds():
    left, right = wheel_speeds(0.2, 0.0, 0.225)
    assert isclose(left, 0.2)
    assert isclose(right, 0.2)


def test_rotation_round_trip():
    left, right = wheel_speeds(0.1, 0.8, 0.225)
    linear, angular = body_velocity(left, right, 0.225)
    assert isclose(linear, 0.1)
    assert isclose(angular, 0.8)


def test_pose_integration():
    pose = integrate(Pose2D(), 0.5, 0.0, 2.0)
    assert isclose(pose.x, 1.0)
    assert isclose(pose.y, 0.0)
    assert isclose(pose.yaw, 0.0)
