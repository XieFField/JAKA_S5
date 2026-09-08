#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "massage_motion/wrench_frame_transform.hpp"

namespace
{

class WrenchFrameTransformer
{
public:
  explicit WrenchFrameTransformer(const rclcpp::Node::SharedPtr & node)
  : node_(node), buffer_(node_->get_clock()), listener_(buffer_)
  {
    node_->get_parameter_or("input_topic", input_topic_,
      std::string{"/massage/ft_sensor/wrench_raw"});
    node_->get_parameter_or("output_topic", output_topic_,
      std::string{"/massage/ft_sensor/wrench_world"});
    node_->get_parameter_or("expression_frame", expression_frame_,
      std::string{"world"});
    node_->get_parameter_or("reference_point_frame", reference_point_frame_,
      std::string{"massage_tool_tip"});
    node_->get_parameter_or("output_frame", output_frame_,
      std::string{"massage_tool_tip_world_aligned"});
    node_->get_parameter_or("source_frame_override", source_frame_override_,
      std::string{});
    if (input_topic_.empty() || output_topic_.empty() ||
      expression_frame_.empty() || reference_point_frame_.empty() ||
      output_frame_.empty())
    {
      throw std::invalid_argument("六维力变换器话题和坐标系参数不能为空");
    }
    publisher_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      output_topic_, rclcpp::SensorDataQoS());
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    subscription_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::WrenchStamped::SharedPtr message)
      {
        process(*message);
      });
    RCLCPP_INFO(
      node_->get_logger(),
      "六维力世界系变换器已启动: input=%s, expression=%s, "
      "reference_point=%s, source_override=%s, output_frame=%s, output=%s",
      input_topic_.c_str(), expression_frame_.c_str(),
      reference_point_frame_.c_str(),
      source_frame_override_.empty() ? "<message frame>" :
      source_frame_override_.c_str(), output_frame_.c_str(),
      output_topic_.c_str());
  }

  std::size_t transformed_count() const
  {
    return transformed_count_.load();
  }

  std::size_t failure_count() const
  {
    return failure_count_.load();
  }

private:
  void process(const geometry_msgs::msg::WrenchStamped & message)
  {
    if (message.header.frame_id.empty())
    {
      ++failure_count_;
      RCLCPP_ERROR(node_->get_logger(), "收到 frame_id 为空的六维力样本");
      return;
    }
    try
    {
      const std::string & source_frame = source_frame_override_.empty() ?
        message.header.frame_id : source_frame_override_;
      const auto target_from_sensor = buffer_.lookupTransform(
        expression_frame_, source_frame, tf2::TimePointZero);
      const auto target_from_reference = buffer_.lookupTransform(
        expression_frame_, reference_point_frame_, tf2::TimePointZero);
      massage_motion::WrenchFrameTransformRequest request;
      request.source_wrench = {
        message.wrench.force.x, message.wrench.force.y,
        message.wrench.force.z, message.wrench.torque.x,
        message.wrench.torque.y, message.wrench.torque.z};
      request.target_from_source_rotation =
        target_from_sensor.transform.rotation;
      request.source_origin_in_target.x =
        target_from_sensor.transform.translation.x;
      request.source_origin_in_target.y =
        target_from_sensor.transform.translation.y;
      request.source_origin_in_target.z =
        target_from_sensor.transform.translation.z;
      request.reference_origin_in_target.x =
        target_from_reference.transform.translation.x;
      request.reference_origin_in_target.y =
        target_from_reference.transform.translation.y;
      request.reference_origin_in_target.z =
        target_from_reference.transform.translation.z;
      const auto transformed =
        massage_motion::transform_wrench_to_reference(request);
      if (!transformed.valid)
      {
        ++failure_count_;
        RCLCPP_ERROR(
          node_->get_logger(), "六维力变换失败: %s",
          transformed.message.c_str());
        return;
      }

      geometry_msgs::msg::WrenchStamped output;
      output.header.stamp = message.header.stamp;
      output.header.frame_id = output_frame_;
      output.wrench.force.x = transformed.wrench[0];
      output.wrench.force.y = transformed.wrench[1];
      output.wrench.force.z = transformed.wrench[2];
      output.wrench.torque.x = transformed.wrench[3];
      output.wrench.torque.y = transformed.wrench[4];
      output.wrench.torque.z = transformed.wrench[5];
      publisher_->publish(output);

      geometry_msgs::msg::TransformStamped virtual_frame;
      virtual_frame.header.stamp = message.header.stamp;
      virtual_frame.header.frame_id = expression_frame_;
      virtual_frame.child_frame_id = output_frame_;
      virtual_frame.transform.translation =
        target_from_reference.transform.translation;
      virtual_frame.transform.rotation.w = 1.0;
      broadcaster_->sendTransform(virtual_frame);
      const auto count = ++transformed_count_;
      if (count == 1U)
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "WRENCH WORLD TRANSFORM ACTIVE: source=%s, output=%s, "
          "force=[%.6f %.6f %.6f] N, torque=[%.6f %.6f %.6f] N*m",
          source_frame.c_str(), output_frame_.c_str(),
          transformed.wrench[0], transformed.wrench[1], transformed.wrench[2],
          transformed.wrench[3], transformed.wrench[4], transformed.wrench[5]);
      }
    }
    catch (const tf2::TransformException & exception)
    {
      ++failure_count_;
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "等待六维力坐标变换: %s", exception.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  std::string input_topic_;
  std::string output_topic_;
  std::string expression_frame_;
  std::string reference_point_frame_;
  std::string output_frame_;
  std::string source_frame_override_;
  std::atomic<std::size_t> transformed_count_{0U};
  std::atomic<std::size_t> failure_count_{0U};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "wrench_frame_transformer",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  int validation_sample_count = 0;
  double validation_timeout = 10.0;
  node->get_parameter_or(
    "validation_sample_count", validation_sample_count, 0);
  node->get_parameter_or("validation_timeout", validation_timeout, 10.0);
  if (validation_sample_count < 0 || !std::isfinite(validation_timeout) ||
    validation_timeout <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "变换器验收参数无效");
    rclcpp::shutdown();
    return 2;
  }
  try
  {
    WrenchFrameTransformer transformer(node);
    const auto started = std::chrono::steady_clock::now();
    rclcpp::WallRate rate(200.0);
    while (rclcpp::ok())
    {
      rclcpp::spin_some(node);
      if (validation_sample_count > 0 && transformer.transformed_count() >=
        static_cast<std::size_t>(validation_sample_count))
      {
        RCLCPP_INFO(
          node->get_logger(),
          "WRENCH WORLD TRANSFORM: PASS: transformed=%zu, failures=%zu",
          transformer.transformed_count(), transformer.failure_count());
        rclcpp::shutdown();
        return 0;
      }
      if (validation_sample_count > 0 &&
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count() >
        validation_timeout)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "WRENCH WORLD TRANSFORM: FAIL: transformed=%zu, failures=%zu",
          transformer.transformed_count(), transformer.failure_count());
        rclcpp::shutdown();
        return 3;
      }
      rate.sleep();
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(node->get_logger(), "六维力变换器异常: %s", exception.what());
  }
  rclcpp::shutdown();
  return 1;
}
