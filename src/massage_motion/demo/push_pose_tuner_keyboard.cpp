#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>
#include <memory>
#include <string>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

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
  if (ready < 0)
  {
    return errno == EINTR;
  }
  if (ready == 0)
  {
    return false;
  }
  return read(STDIN_FILENO, &key, 1U) == 1;
}

void call_trigger(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const std::string & operation)
{
  if (!client->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(node->get_logger(), "%s 服务不可用", operation.c_str());
    return;
  }
  auto future = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  const auto status = rclcpp::spin_until_future_complete(
    node, future, std::chrono::seconds(3));
  if (status != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "%s 服务调用超时", operation.c_str());
    return;
  }
  const auto response = future.get();
  if (response->success)
  {
    RCLCPP_INFO(
      node->get_logger(), "%s: %s",
      operation.c_str(), response->message.c_str());
  }
  else
  {
    RCLCPP_WARN(
      node->get_logger(), "%s: %s",
      operation.c_str(), response->message.c_str());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("push_pose_tuner_keyboard");
  const double translation_step =
    node->declare_parameter<double>("translation_step", 0.005);
  const double rotation_step_deg =
    node->declare_parameter<double>("rotation_step_deg", 2.0);
  if (!std::isfinite(translation_step) || translation_step <= 0.0 ||
    translation_step > 0.02 || !std::isfinite(rotation_step_deg) ||
    rotation_step_deg <= 0.0 || rotation_step_deg > 10.0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "步长无效: translation_step 必须在 (0, 0.02] m，"
      "rotation_step_deg 必须在 (0, 10] deg");
    rclcpp::shutdown();
    return 1;
  }

  const double rotation_step = rotation_step_deg * kPi / 180.0;
  const auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
    "/push_pose_tuner/nudge", 10);
  const auto confirm_client = node->create_client<std_srvs::srv::Trigger>(
    "/push_pose_tuner/confirm");
  const auto reset_client = node->create_client<std_srvs::srv::Trigger>(
    "/push_pose_tuner/reset");
  const auto replan_client = node->create_client<std_srvs::srv::Trigger>(
    "/push_pose_tuner/replan");

  TerminalMode terminal;
  std::string terminal_error;
  if (!terminal.enable(terminal_error))
  {
    RCLCPP_ERROR(node->get_logger(), "%s", terminal_error.c_str());
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "精确步进已就绪: translation=%.3f mm, rotation=%.3f deg",
    translation_step * 1000.0, rotation_step_deg);
  RCLCPP_INFO(
    node->get_logger(),
    "位置 w/s:+x/-x, a/d:+y/-y, r/f:+z/-z；"
    "姿态 u/j:+roll/-roll, i/k:+pitch/-pitch, o/l:+yaw/-yaw；"
    "p:重新规划, c:确认保存, 0:复位, q:退出键盘工具");

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
    if (key == 'c')
    {
      call_trigger(node, confirm_client, "确认保存");
      continue;
    }
    if (key == '0')
    {
      call_trigger(node, reset_client, "复位");
      continue;
    }
    if (key == 'p')
    {
      call_trigger(node, replan_client, "重新规划");
      continue;
    }

    geometry_msgs::msg::Twist nudge;
    switch (key)
    {
      case 'w':
        nudge.linear.x = translation_step;
        break;
      case 's':
        nudge.linear.x = -translation_step;
        break;
      case 'a':
        nudge.linear.y = translation_step;
        break;
      case 'd':
        nudge.linear.y = -translation_step;
        break;
      case 'r':
        nudge.linear.z = translation_step;
        break;
      case 'f':
        nudge.linear.z = -translation_step;
        break;
      case 'u':
        nudge.angular.x = rotation_step;
        break;
      case 'j':
        nudge.angular.x = -rotation_step;
        break;
      case 'i':
        nudge.angular.y = rotation_step;
        break;
      case 'k':
        nudge.angular.y = -rotation_step;
        break;
      case 'o':
        nudge.angular.z = rotation_step;
        break;
      case 'l':
        nudge.angular.z = -rotation_step;
        break;
      default:
        continue;
    }
    publisher->publish(nudge);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
