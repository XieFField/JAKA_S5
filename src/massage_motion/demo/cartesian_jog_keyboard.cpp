#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

class TerminalMode
{
public:
  bool enable(std::string & error)
  {
    if (!isatty(STDIN_FILENO))
    {
      error = "标准输入不是交互终端";
      return false;
    }
    if (tcgetattr(STDIN_FILENO, &original_) != 0)
    {
      error = std::string{"读取终端属性失败: "} + std::strerror(errno);
      return false;
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
    {
      error = std::string{"设置终端模式失败: "} + std::strerror(errno);
      return false;
    }
    enabled_ = true;
    return true;
  }

  ~TerminalMode()
  {
    if (enabled_)
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

private:
  termios original_{};
  bool enabled_{false};
};

bool wait_for_key(char & key)
{
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(STDIN_FILENO, &read_set);
  timeval timeout{};
  timeout.tv_usec = 100000;
  const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
  if (ready <= 0)
  {
    return false;
  }
  return read(STDIN_FILENO, &key, 1U) == 1;
}

void publish_pulse(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & publisher,
  const geometry_msgs::msg::Twist & twist,
  const std::string & frame,
  double duration,
  double period)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(duration);
  const auto sleep_period = std::chrono::duration<double>(period);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = node->now();
    command.header.frame_id = frame;
    command.twist = twist;
    publisher->publish(command);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(sleep_period);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cartesian_jog_keyboard");
  const double translation_step =
    node->declare_parameter<double>("translation_step", 0.010);
  const double rotation_step_deg =
    node->declare_parameter<double>("rotation_step_deg", 2.0);
  const double command_duration =
    node->declare_parameter<double>("command_duration", 0.1);
  const double publish_period =
    node->declare_parameter<double>("publish_period", 0.02);
  const double subscriber_wait_timeout =
    node->declare_parameter<double>("subscriber_wait_timeout", 5.0);
  const std::string world_frame =
    node->declare_parameter<std::string>("world_frame", "world");
  const std::string tool_frame =
    node->declare_parameter<std::string>("tool_frame", "massage_tool_tip");

  if (!std::isfinite(translation_step) || translation_step <= 0.0 ||
    translation_step > 0.02 || !std::isfinite(rotation_step_deg) ||
    rotation_step_deg <= 0.0 || rotation_step_deg > 10.0 ||
    !std::isfinite(command_duration) || command_duration <= 0.0 ||
    !std::isfinite(publish_period) || publish_period <= 0.0 ||
    publish_period > command_duration ||
    !std::isfinite(subscriber_wait_timeout) || subscriber_wait_timeout <= 0.0 ||
    world_frame.empty() || tool_frame.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Cartesian jog keyboard parameters are invalid");
    rclcpp::shutdown();
    return 1;
  }

  const auto publisher =
    node->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/massage/cartesian_jog/command", 10);
  const auto subscriber_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(subscriber_wait_timeout);
  while (rclcpp::ok() && publisher->get_subscription_count() == 0U &&
    std::chrono::steady_clock::now() < subscriber_deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (publisher->get_subscription_count() == 0U)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "未发现笛卡尔控制桥接节点。请先在项目容器内运行 "
      "ros2 launch massage_bringup mujoco_sim.launch.py use_rviz:=false");
    rclcpp::shutdown();
    return 3;
  }

  TerminalMode terminal;
  std::string error;
  if (!terminal.enable(error))
  {
    RCLCPP_ERROR(node->get_logger(), "%s", error.c_str());
    rclcpp::shutdown();
    return 2;
  }

  std::string frame = world_frame;
  RCLCPP_INFO(
    node->get_logger(),
    "笛卡尔控制链已连接: XYZ=%.3f mm, RPY=%.3f deg, frame=World",
    translation_step * 1000.0, rotation_step_deg);
  RCLCPP_INFO(
    node->get_logger(),
    "w/s:X, a/d:Y, r/f:Z, u/j:Roll, i/k:Pitch, o/l:Yaw, "
    "1:World, 2:Tool, q:退出");

  const double linear_speed = translation_step / command_duration;
  const double angular_speed = rotation_step_deg * kPi / 180.0 / command_duration;
  while (rclcpp::ok())
  {
    char key = '\0';
    if (!wait_for_key(key))
    {
      rclcpp::spin_some(node);
      continue;
    }
    key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    if (key == 'q')
    {
      break;
    }
    if (key == '1' || key == '2')
    {
      frame = key == '1' ? world_frame : tool_frame;
      RCLCPP_INFO(
        node->get_logger(), "Cartesian frame=%s",
        key == '1' ? "World" : "Tool");
      continue;
    }

    geometry_msgs::msg::Twist twist;
    switch (key)
    {
      case 'w': twist.linear.x = linear_speed; break;
      case 's': twist.linear.x = -linear_speed; break;
      case 'a': twist.linear.y = linear_speed; break;
      case 'd': twist.linear.y = -linear_speed; break;
      case 'r': twist.linear.z = linear_speed; break;
      case 'f': twist.linear.z = -linear_speed; break;
      case 'u': twist.angular.x = angular_speed; break;
      case 'j': twist.angular.x = -angular_speed; break;
      case 'i': twist.angular.y = angular_speed; break;
      case 'k': twist.angular.y = -angular_speed; break;
      case 'o': twist.angular.z = angular_speed; break;
      case 'l': twist.angular.z = -angular_speed; break;
      default: continue;
    }
    publish_pulse(
      node, publisher, twist, frame, command_duration, publish_period);
  }

  rclcpp::shutdown();
  return 0;
}
