#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

class CartesianJogBridge : public rclcpp::Node
{
public:
  CartesianJogBridge()
  : Node("cartesian_jog_bridge"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/massage/cartesian_jog/command");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/servo_node/delta_twist_cmds");
    planning_frame_ = declare_parameter<std::string>(
      "planning_frame", "world");
    tool_frame_ = declare_parameter<std::string>(
      "tool_frame", "massage_tool_tip");
    servo_start_service_ = declare_parameter<std::string>(
      "servo_start_service", "/servo_node/start_servo");
    maximum_linear_speed_ = declare_parameter<double>(
      "maximum_linear_speed", 0.10);
    maximum_angular_speed_ = declare_parameter<double>(
      "maximum_angular_speed", 1.0);

    if (input_topic_.empty() || output_topic_.empty() ||
      planning_frame_.empty() || tool_frame_.empty() ||
      servo_start_service_.empty() ||
      !std::isfinite(maximum_linear_speed_) ||
      maximum_linear_speed_ <= 0.0 ||
      !std::isfinite(maximum_angular_speed_) ||
      maximum_angular_speed_ <= 0.0)
    {
      throw std::invalid_argument("Cartesian jog parameters are invalid");
    }

    command_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_, 10);
    command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic_, 10,
      std::bind(
        &CartesianJogBridge::handle_command, this, std::placeholders::_1));
    mode_subscription_ = create_subscription<std_msgs::msg::String>(
      "/massage_mujoco/control_mode",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message)
      {
        position_mode_ = message->data == "POSITION";
      });
    servo_start_client_ = create_client<std_srvs::srv::Trigger>(
      servo_start_service_);
    start_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CartesianJogBridge::start_servo, this));

    RCLCPP_INFO(
      get_logger(),
      "Cartesian jog bridge ready: input=%s, World=%s, Tool=%s",
      input_topic_.c_str(), planning_frame_.c_str(), tool_frame_.c_str());
  }

private:
  static bool finite_twist(const geometry_msgs::msg::Twist & twist)
  {
    return std::isfinite(twist.linear.x) &&
           std::isfinite(twist.linear.y) &&
           std::isfinite(twist.linear.z) &&
           std::isfinite(twist.angular.x) &&
           std::isfinite(twist.angular.y) &&
           std::isfinite(twist.angular.z);
  }

  static double vector_norm(const geometry_msgs::msg::Vector3 & vector)
  {
    return std::sqrt(
      vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
  }

  static geometry_msgs::msg::Vector3 rotate_vector(
    const geometry_msgs::msg::Vector3 & input,
    const geometry_msgs::msg::Quaternion & rotation)
  {
    tf2::Quaternion quaternion;
    tf2::fromMsg(rotation, quaternion);
    const tf2::Vector3 source(input.x, input.y, input.z);
    const tf2::Vector3 target = tf2::Matrix3x3(quaternion) * source;
    geometry_msgs::msg::Vector3 output;
    output.x = target.x();
    output.y = target.y();
    output.z = target.z();
    return output;
  }

  void start_servo()
  {
    if (servo_started_ || servo_start_pending_ ||
      !servo_start_client_->service_is_ready())
    {
      return;
    }
    servo_start_pending_ = true;
    servo_start_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
      {
        servo_start_pending_ = false;
        const auto response = future.get();
        if (!response->success)
        {
          RCLCPP_WARN(
            get_logger(), "MoveIt Servo start rejected: %s",
            response->message.c_str());
          return;
        }
        servo_started_ = true;
        start_timer_->cancel();
        RCLCPP_INFO(get_logger(), "MoveIt Servo started");
      });
  }

  void handle_command(
    const geometry_msgs::msg::TwistStamped::SharedPtr command)
  {
    if (!position_mode_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cartesian jog ignored because MuJoCo is not in POSITION mode");
      return;
    }
    if (!servo_started_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cartesian jog ignored because MoveIt Servo is not ready");
      return;
    }
    if (!finite_twist(command->twist) ||
      vector_norm(command->twist.linear) > maximum_linear_speed_ + 1e-9 ||
      vector_norm(command->twist.angular) > maximum_angular_speed_ + 1e-9)
    {
      RCLCPP_WARN(get_logger(), "Cartesian jog exceeds configured speed limits");
      return;
    }
    if (command->header.frame_id != planning_frame_ &&
      command->header.frame_id != tool_frame_)
    {
      RCLCPP_WARN(
        get_logger(), "Unsupported Cartesian jog frame: %s",
        command->header.frame_id.c_str());
      return;
    }

    geometry_msgs::msg::TwistStamped transformed = *command;
    transformed.header.stamp = now();
    transformed.header.frame_id = planning_frame_;
    if (command->header.frame_id == tool_frame_)
    {
      try
      {
        const auto transform = tf_buffer_.lookupTransform(
          planning_frame_, tool_frame_, tf2::TimePointZero);
        transformed.twist.linear = rotate_vector(
          command->twist.linear, transform.transform.rotation);
        transformed.twist.angular = rotate_vector(
          command->twist.angular, transform.transform.rotation);
      }
      catch (const tf2::TransformException & exception)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cartesian jog transform unavailable: %s", exception.what());
        return;
      }
    }
    command_publisher_->publish(transformed);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string planning_frame_;
  std::string tool_frame_;
  std::string servo_start_service_;
  double maximum_linear_speed_{0.10};
  double maximum_angular_speed_{1.0};
  bool position_mode_{true};
  bool servo_started_{false};
  bool servo_start_pending_{false};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    command_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<CartesianJogBridge>());
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(rclcpp::get_logger("cartesian_jog_bridge"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
