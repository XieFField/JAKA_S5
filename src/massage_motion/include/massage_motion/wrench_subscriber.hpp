#ifndef MASSAGE_MOTION__WRENCH_SUBSCRIBER_HPP_
#define MASSAGE_MOTION__WRENCH_SUBSCRIBER_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "massage_motion/wrench_processor.hpp"

namespace massage_motion
{

// 六维力订阅配置。expected_frame_id 为空时接受消息中的任意非空坐标系，
// 便于同一订阅器复用于仿真传感器和不同品牌的真机驱动。
struct WrenchSubscriberConfig
{
    std::string topic_name;
    std::string expected_frame_id;
    double stale_timeout{0.5};
};

// 订阅器在调用时刻的只读快照。
struct WrenchSubscriptionState
{
    WrenchSample sample;
    double age{0.0};
    bool received{false};
    bool stale{true};
    std::uint64_t accepted_samples{0};
    std::uint64_t rejected_samples{0};
};

// 通用 ROS 2 六维力订阅器。
//
// 本类只负责消息接入、坐标系/有限值检查和新鲜度判定，不执行标定、
// 滤波、重力补偿或控制。后续处理继续交给 WrenchProcessor 或控制后端。
class WrenchSubscriber final
{
public:
    WrenchSubscriber(
        rclcpp::Node::SharedPtr node,
        WrenchSubscriberConfig config,
        const rclcpp::QoS & qos = rclcpp::SensorDataQoS());

    WrenchSubscriptionState latest() const;

    const WrenchSubscriberConfig & config() const;

private:
    void handle_message(
        const geometry_msgs::msg::WrenchStamped::SharedPtr message);

    WrenchSubscriberConfig config_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
        subscription_;

    mutable std::mutex mutex_;
    WrenchSample latest_sample_;
    std::chrono::steady_clock::time_point latest_receive_time_{};
    std::uint64_t accepted_samples_{0};
    std::uint64_t rejected_samples_{0};
    bool received_{false};
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__WRENCH_SUBSCRIBER_HPP_
