#include "massage_motion/wrench_subscriber.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace massage_motion
{

namespace
{

WrenchVector wrench_values(const geometry_msgs::msg::Wrench & wrench)
{
    return {
        wrench.force.x,
        wrench.force.y,
        wrench.force.z,
        wrench.torque.x,
        wrench.torque.y,
        wrench.torque.z};
}

}  // namespace

WrenchSubscriber::WrenchSubscriber(
    rclcpp::Node::SharedPtr node,
    WrenchSubscriberConfig config,
    const rclcpp::QoS & qos)
    : config_(std::move(config))
{
    if (!node)
    {
        throw std::invalid_argument("六维力订阅器节点不能为空");
    }
    if (config_.topic_name.empty())
    {
        throw std::invalid_argument("六维力话题名称不能为空");
    }
    if (!std::isfinite(config_.stale_timeout) ||
        config_.stale_timeout <= 0.0)
    {
        throw std::invalid_argument("六维力反馈超时时间必须是有限正数");
    }

    subscription_ =
        node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        config_.topic_name,
        qos,
        [this](geometry_msgs::msg::WrenchStamped::SharedPtr message)
        {
            handle_message(std::move(message));
        });
}

WrenchSubscriptionState WrenchSubscriber::latest() const
{
    WrenchSubscriptionState result;
    std::lock_guard<std::mutex> lock(mutex_);
    result.received = received_;
    result.accepted_samples = accepted_samples_;
    result.rejected_samples = rejected_samples_;
    if (!received_)
    {
        result.age = std::numeric_limits<double>::infinity();
        return result;
    }

    result.sample = latest_sample_;
    result.age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_receive_time_).count();
    result.stale = result.age > config_.stale_timeout;
    return result;
}

const WrenchSubscriberConfig & WrenchSubscriber::config() const
{
    return config_;
}

void WrenchSubscriber::handle_message(
    const geometry_msgs::msg::WrenchStamped::SharedPtr message)
{
    const auto values = wrench_values(message->wrench);
    const bool frame_valid =
        !message->header.frame_id.empty() &&
        (config_.expected_frame_id.empty() ||
        message->header.frame_id == config_.expected_frame_id);
    const bool values_valid = std::all_of(
        values.begin(),
        values.end(),
        [](double value) {return std::isfinite(value);});

    std::lock_guard<std::mutex> lock(mutex_);
    if (!frame_valid || !values_valid)
    {
        ++rejected_samples_;
        return;
    }

    latest_sample_.stamp_nanoseconds =
        rclcpp::Time(message->header.stamp).nanoseconds();
    latest_sample_.frame_id = message->header.frame_id;
    latest_sample_.values = values;
    latest_receive_time_ = std::chrono::steady_clock::now();
    ++accepted_samples_;
    received_ = true;
}

}  // namespace massage_motion
