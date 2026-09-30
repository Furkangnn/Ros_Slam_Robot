#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include "ros_slam_robot/kinematics.hpp"

TEST(Kinematics, StraightMotionUsesEqualWheelSpeeds)
{
  const auto speeds = ros_slam_robot::wheel_speeds(0.2, 0.0, 0.225);
  EXPECT_DOUBLE_EQ(speeds.first, 0.2);
  EXPECT_DOUBLE_EQ(speeds.second, 0.2);
}

TEST(Kinematics, ConversionRoundTripPreservesVelocity)
{
  const auto wheels = ros_slam_robot::wheel_speeds(0.1, 0.8, 0.225);
  const auto body =
    ros_slam_robot::body_velocity(wheels.first, wheels.second, 0.225);
  EXPECT_NEAR(body.first, 0.1, 1e-12);
  EXPECT_NEAR(body.second, 0.8, 1e-12);
}

TEST(Kinematics, MidpointIntegrationAdvancesPose)
{
  const ros_slam_robot::Pose2D start;
  const auto result = ros_slam_robot::integrate(start, 0.5, 0.0, 2.0);
  EXPECT_NEAR(result.x, 1.0, 1e-12);
  EXPECT_NEAR(result.y, 0.0, 1e-12);
  EXPECT_NEAR(result.yaw, 0.0, 1e-12);
}

TEST(Kinematics, InvalidGeometryIsRejected)
{
  EXPECT_THROW(
    ros_slam_robot::wheel_speeds(0.1, 0.2, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    ros_slam_robot::body_velocity(0.1, 0.2, -1.0),
    std::invalid_argument);
}

TEST(Kinematics, PureRotationUsesOppositeWheelSpeeds)
{
  const auto speeds = ros_slam_robot::wheel_speeds(0.0, 1.0, 0.4);
  EXPECT_NEAR(speeds.first, -0.2, 1e-12);
  EXPECT_NEAR(speeds.second, 0.2, 1e-12);
}

TEST(Kinematics, ReverseMotionRemainsNegative)
{
  const auto speeds = ros_slam_robot::wheel_speeds(-0.25, 0.0, 0.225);
  const auto body =
    ros_slam_robot::body_velocity(speeds.first, speeds.second, 0.225);
  EXPECT_NEAR(body.first, -0.25, 1e-12);
  EXPECT_NEAR(body.second, 0.0, 1e-12);
}

TEST(Kinematics, CurvedMotionChangesPositionAndHeading)
{
  const ros_slam_robot::Pose2D start;
  const auto result = ros_slam_robot::integrate(start, 0.4, 0.5, 1.0);
  EXPECT_GT(result.x, 0.0);
  EXPECT_GT(result.y, 0.0);
  EXPECT_NEAR(result.yaw, 0.5, 1e-12);
}

TEST(Kinematics, HeadingIsNormalizedAfterIntegration)
{
  ros_slam_robot::Pose2D start;
  start.yaw = 3.0;
  const auto result = ros_slam_robot::integrate(start, 0.0, 1.0, 1.0);
  constexpr double pi = 3.14159265358979323846;
  EXPECT_GE(result.yaw, -pi);
  EXPECT_LE(result.yaw, pi);
}

TEST(Kinematics, ZeroDurationDoesNotMovePose)
{
  ros_slam_robot::Pose2D start;
  start.x = 1.2;
  start.y = -0.7;
  start.yaw = 0.3;
  const auto result = ros_slam_robot::integrate(start, 5.0, 2.0, 0.0);
  EXPECT_DOUBLE_EQ(result.x, start.x);
  EXPECT_DOUBLE_EQ(result.y, start.y);
  EXPECT_DOUBLE_EQ(result.yaw, start.yaw);
}

TEST(Kinematics, NegativeDurationIsRejected)
{
  EXPECT_THROW(
    ros_slam_robot::integrate(ros_slam_robot::Pose2D{}, 0.1, 0.0, -0.01),
    std::invalid_argument);
}
