#pragma once

#include <utility>

namespace ros_slam_robot
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

std::pair<double, double> wheel_speeds(
  double linear, double angular, double wheel_separation);

std::pair<double, double> body_velocity(
  double left, double right, double wheel_separation);

Pose2D integrate(Pose2D pose, double linear, double angular, double dt);

}  // namespace ros_slam_robot
