#include "ros_slam_robot/kinematics.hpp"

#include <cmath>
#include <stdexcept>

namespace ros_slam_robot
{

std::pair<double, double> wheel_speeds(
  const double linear, const double angular, const double wheel_separation)
{
  if (wheel_separation <= 0.0) {
    throw std::invalid_argument("wheel_separation must be positive");
  }
  const double half_turn = angular * wheel_separation / 2.0;
  return {linear - half_turn, linear + half_turn};
}

std::pair<double, double> body_velocity(
  const double left, const double right, const double wheel_separation)
{
  if (wheel_separation <= 0.0) {
    throw std::invalid_argument("wheel_separation must be positive");
  }
  return {(left + right) / 2.0, (right - left) / wheel_separation};
}

Pose2D integrate(
  Pose2D pose, const double linear, const double angular, const double dt)
{
  if (dt < 0.0) {
    throw std::invalid_argument("dt cannot be negative");
  }
  const double midpoint_heading = pose.yaw + angular * dt / 2.0;
  pose.x += linear * std::cos(midpoint_heading) * dt;
  pose.y += linear * std::sin(midpoint_heading) * dt;
  pose.yaw = std::atan2(
    std::sin(pose.yaw + angular * dt),
    std::cos(pose.yaw + angular * dt));
  return pose;
}

}  // namespace ros_slam_robot
