#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_slam_robot/kinematics.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace ros_slam_robot
{

class SerialPort
{
public:
  SerialPort(const std::string & device, const int baudrate)
  {
    descriptor_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor_ < 0) {
      throw std::runtime_error(
              "cannot open " + device + ": " + std::strerror(errno));
    }

    termios options{};
    if (tcgetattr(descriptor_, &options) != 0) {
      close();
      throw std::runtime_error("cannot read serial-port attributes");
    }

    const speed_t speed = baudrate_to_termios(baudrate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
      ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(descriptor_, TCSANOW, &options) != 0) {
      close();
      throw std::runtime_error("cannot configure serial port");
    }
    tcflush(descriptor_, TCIOFLUSH);
  }

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  ~SerialPort()
  {
    close();
  }

  void write_line(const std::string & line)
  {
    const std::string frame = line + '\n';
    std::size_t sent = 0;
    while (sent < frame.size()) {
      const ssize_t count = ::write(
        descriptor_, frame.data() + sent, frame.size() - sent);
      if (count > 0) {
        sent += static_cast<std::size_t>(count);
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        throw std::runtime_error(
                "serial write failed: " + std::string(std::strerror(errno)));
      } else {
        break;
      }
    }
  }

  std::string read_line()
  {
    char bytes[128];
    const ssize_t count = ::read(descriptor_, bytes, sizeof(bytes));
    if (count > 0) {
      buffer_.append(bytes, static_cast<std::size_t>(count));
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      throw std::runtime_error(
              "serial read failed: " + std::string(std::strerror(errno)));
    }

    const std::size_t newline = buffer_.find('\n');
    if (newline == std::string::npos) {
      if (buffer_.size() > 512) {
        buffer_.clear();
      }
      return {};
    }
    std::string line = buffer_.substr(0, newline);
    buffer_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    return line;
  }

private:
  static speed_t baudrate_to_termios(const int baudrate)
  {
    switch (baudrate) {
      case 9600:
        return B9600;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      default:
        throw std::invalid_argument("supported baud rates: 9600, 57600, 115200");
    }
  }

  void close()
  {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
      descriptor_ = -1;
    }
  }

  int descriptor_{-1};
  std::string buffer_;
};

class SerialBase : public rclcpp::Node
{
public:
  SerialBase()
  : Node("serial_base")
  {
    const std::string port = declare_parameter("port", "/dev/ttyUSB_ARDUINO");
    const int baudrate = declare_parameter("baudrate", 115200);
    wheel_radius_ = declare_parameter("wheel_radius", 0.033);
    wheel_separation_ = declare_parameter("wheel_separation", 0.225);
    ticks_per_revolution_ = declare_parameter("ticks_per_revolution", 600);
    command_timeout_ = declare_parameter("command_timeout", 0.5);
    publish_tf_ = declare_parameter("publish_tf", true);

    if (wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0 ||
      ticks_per_revolution_ <= 0)
    {
      throw std::invalid_argument("wheel dimensions and encoder resolution must be positive");
    }

    serial_ = std::make_unique<SerialPort>(port, baudrate);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    transform_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 20,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        const auto speeds = wheel_speeds(
          message->linear.x, message->angular.z, wheel_separation_);
        std::scoped_lock lock(command_mutex_);
        target_left_ = speeds.first;
        target_right_ = speeds.second;
        last_command_time_ = std::chrono::steady_clock::now();
      });

    last_command_time_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(50ms, std::bind(&SerialBase::exchange, this));
    RCLCPP_INFO(get_logger(), "Motor controller connected at %s", port.c_str());
  }

  ~SerialBase() override
  {
    try {
      if (serial_) {
        serial_->write_line("V 0.0 0.0");
      }
    } catch (const std::exception &) {
      // Destructors must not throw while ROS is shutting down.
    }
  }

private:
  void exchange()
  {
    const auto monotonic_now = std::chrono::steady_clock::now();
    double left = 0.0;
    double right = 0.0;
    {
      std::scoped_lock lock(command_mutex_);
      const double age =
        std::chrono::duration<double>(monotonic_now - last_command_time_).count();
      if (age <= command_timeout_) {
        left = target_left_;
        right = target_right_;
      }
    }

    std::ostringstream command;
    command.setf(std::ios::fixed);
    command.precision(4);
    command << "V " << left << ' ' << right;

    try {
      serial_->write_line(command.str());
      for (int frame = 0; frame < 4; ++frame) {
        const std::string line = serial_->read_line();
        if (line.empty()) {
          break;
        }
        parse_encoder_frame(line, monotonic_now);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s", error.what());
    }
  }

  void parse_encoder_frame(
    const std::string & line,
    const std::chrono::steady_clock::time_point sample_time)
  {
    std::istringstream frame(line);
    char frame_type = '\0';
    long left_ticks = 0;
    long right_ticks = 0;
    if (!(frame >> frame_type >> left_ticks >> right_ticks) ||
      frame_type != 'E')
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring malformed encoder frame: %s", line.c_str());
      return;
    }

    if (!have_encoder_sample_) {
      previous_left_ticks_ = left_ticks;
      previous_right_ticks_ = right_ticks;
      previous_encoder_time_ = sample_time;
      have_encoder_sample_ = true;
      return;
    }

    const double dt =
      std::chrono::duration<double>(sample_time - previous_encoder_time_).count();
    if (dt <= 0.0) {
      return;
    }
    constexpr double pi = 3.14159265358979323846;
    const double metres_per_tick =
      2.0 * pi * wheel_radius_ / static_cast<double>(ticks_per_revolution_);
    const double left_velocity =
      (left_ticks - previous_left_ticks_) * metres_per_tick / dt;
    const double right_velocity =
      (right_ticks - previous_right_ticks_) * metres_per_tick / dt;
    const auto velocity =
      body_velocity(left_velocity, right_velocity, wheel_separation_);
    pose_ = integrate(pose_, velocity.first, velocity.second, dt);

    previous_left_ticks_ = left_ticks;
    previous_right_ticks_ = right_ticks;
    previous_encoder_time_ = sample_time;
    publish_odometry(velocity.first, velocity.second);
  }

  void publish_odometry(const double linear, const double angular)
  {
    const rclcpp::Time stamp = now();
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, pose_.yaw);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = pose_.x;
    odometry.pose.pose.position.y = pose_.y;
    odometry.pose.pose.orientation.x = rotation.x();
    odometry.pose.pose.orientation.y = rotation.y();
    odometry.pose.pose.orientation.z = rotation.z();
    odometry.pose.pose.orientation.w = rotation.w();
    odometry.twist.twist.linear.x = linear;
    odometry.twist.twist.angular.z = angular;
    odometry_publisher_->publish(odometry);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = odometry.child_frame_id;
      transform.transform.translation.x = pose_.x;
      transform.transform.translation.y = pose_.y;
      transform.transform.rotation = odometry.pose.pose.orientation;
      transform_broadcaster_->sendTransform(transform);
    }
  }

  std::unique_ptr<SerialPort> serial_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex command_mutex_;
  double target_left_{0.0};
  double target_right_{0.0};
  std::chrono::steady_clock::time_point last_command_time_;

  double wheel_radius_{0.033};
  double wheel_separation_{0.225};
  int ticks_per_revolution_{600};
  double command_timeout_{0.5};
  bool publish_tf_{true};

  Pose2D pose_;
  bool have_encoder_sample_{false};
  long previous_left_ticks_{0};
  long previous_right_ticks_{0};
  std::chrono::steady_clock::time_point previous_encoder_time_;
};

}  // namespace ros_slam_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ros_slam_robot::SerialBase>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("serial_base"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
