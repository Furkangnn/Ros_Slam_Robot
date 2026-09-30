#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

namespace ros_slam_robot
{

class RobotHealth : public rclcpp::Node
{
public:
  RobotHealth()
  : Node("robot_health")
  {
    stale_after_ = declare_parameter("stale_after", 1.0);
    minimum_odom_rate_ = declare_parameter("minimum_odom_rate", 10.0);
    minimum_scan_rate_ = declare_parameter("minimum_scan_rate", 3.0);
    obstacle_warning_distance_ =
      declare_parameter("obstacle_warning_distance", 0.25);

    if (stale_after_ <= 0.0 || minimum_odom_rate_ <= 0.0 ||
      minimum_scan_rate_ <= 0.0 || obstacle_warning_distance_ <= 0.0)
    {
      throw std::invalid_argument("health thresholds must be positive");
    }

    diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        const auto now = std::chrono::steady_clock::now();
        update_rate(odometry_state_, now);
        odometry_state_.received = true;
        odometry_state_.last_message = now;
        linear_velocity_ = message->twist.twist.linear.x;
        angular_velocity_ = message->twist.twist.angular.z;
      });
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        const auto now = std::chrono::steady_clock::now();
        update_rate(scan_state_, now);
        scan_state_.received = true;
        scan_state_.last_message = now;
        valid_ranges_ = 0;
        nearest_obstacle_ = std::numeric_limits<double>::infinity();
        for (const float distance : message->ranges) {
          if (std::isfinite(distance) &&
            distance >= message->range_min && distance <= message->range_max)
          {
            ++valid_ranges_;
            nearest_obstacle_ =
              std::min(nearest_obstacle_, static_cast<double>(distance));
          }
        }
      });

    report_timer_ = create_wall_timer(1s, [this]() {publish_report();});
    RCLCPP_INFO(get_logger(), "Robot health monitoring started");
  }

private:
  struct StreamState
  {
    bool received{false};
    std::size_t messages_in_window{0};
    double measured_rate{0.0};
    std::chrono::steady_clock::time_point last_message{};
    std::chrono::steady_clock::time_point window_start{
      std::chrono::steady_clock::now()};
  };

  static diagnostic_msgs::msg::KeyValue value(
    const std::string & key, const std::string & text)
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = text;
    return result;
  }

  static void update_rate(
    StreamState & state,
    const std::chrono::steady_clock::time_point now)
  {
    ++state.messages_in_window;
    const double elapsed =
      std::chrono::duration<double>(now - state.window_start).count();
    if (elapsed >= 1.0) {
      state.measured_rate =
        static_cast<double>(state.messages_in_window) / elapsed;
      state.messages_in_window = 0;
      state.window_start = now;
    }
  }

  diagnostic_msgs::msg::DiagnosticStatus stream_status(
    const std::string & name, const StreamState & state,
    const double minimum_rate) const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "Ros Slam Robot/" + name;
    status.hardware_id = "ros_slam_robot";

    const auto now = std::chrono::steady_clock::now();
    const double age = state.received ?
      std::chrono::duration<double>(now - state.last_message).count() :
      std::numeric_limits<double>::infinity();

    if (!state.received || age > stale_after_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = name + " stream is stale";
    } else if (state.measured_rate > 0.0 &&
      state.measured_rate < minimum_rate)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = name + " rate is below target";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = name + " stream is healthy";
    }

    status.values.push_back(value("rate_hz", std::to_string(state.measured_rate)));
    status.values.push_back(value("age_seconds", std::to_string(age)));
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus obstacle_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "Ros Slam Robot/Obstacle clearance";
    status.hardware_id = "rplidar";
    if (!std::isfinite(nearest_obstacle_)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "No valid lidar returns";
    } else if (nearest_obstacle_ < obstacle_warning_distance_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Obstacle is close to the robot";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Obstacle clearance is sufficient";
    }
    status.values.push_back(
      value("nearest_metres", std::to_string(nearest_obstacle_)));
    status.values.push_back(
      value("valid_ranges", std::to_string(valid_ranges_)));
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus motion_status() const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "Ros Slam Robot/Motion";
    status.hardware_id = "differential_drive";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message =
      std::abs(linear_velocity_) < 0.01 &&
      std::abs(angular_velocity_) < 0.02 ? "Robot is stopped" : "Robot is moving";
    status.values.push_back(
      value("linear_mps", std::to_string(linear_velocity_)));
    status.values.push_back(
      value("angular_radps", std::to_string(angular_velocity_)));
    return status;
  }

  void publish_report()
  {
    diagnostic_msgs::msg::DiagnosticArray report;
    report.header.stamp = now();
    report.status.push_back(
      stream_status("Odometry", odometry_state_, minimum_odom_rate_));
    report.status.push_back(
      stream_status("Laser scan", scan_state_, minimum_scan_rate_));
    report.status.push_back(obstacle_status());
    report.status.push_back(motion_status());
    diagnostic_publisher_->publish(report);
  }

  double stale_after_{1.0};
  double minimum_odom_rate_{10.0};
  double minimum_scan_rate_{3.0};
  double obstacle_warning_distance_{0.25};
  double nearest_obstacle_{std::numeric_limits<double>::infinity()};
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};
  std::size_t valid_ranges_{0};
  StreamState odometry_state_;
  StreamState scan_state_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    scan_subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace ros_slam_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ros_slam_robot::RobotHealth>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_health"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
