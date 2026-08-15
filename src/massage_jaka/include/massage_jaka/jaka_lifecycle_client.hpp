#ifndef MASSAGE_JAKA__JAKA_LIFECYCLE_CLIENT_HPP_
#define MASSAGE_JAKA__JAKA_LIFECYCLE_CLIENT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace massage_jaka
{

struct LifecycleResult
{
    bool success{false};
    std::string message;
};

// 只封装显式生命周期服务。构造本类不会连接、上电或使能机械臂。
class JakaLifecycleClient
{
public:
    explicit JakaLifecycleClient(
        rclcpp::Node::SharedPtr node,
        std::string service_prefix = "/jaka_driver");

    LifecycleResult login();
    LifecycleResult power_on();
    LifecycleResult enable_robot();
    LifecycleResult disable_robot();
    LifecycleResult power_off();
    LifecycleResult logout();
    LifecycleResult stop_motion();

private:
    LifecycleResult call(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
        const std::string & operation);

    rclcpp::Node::SharedPtr node_;
    double service_timeout_{3.0};
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr login_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disable_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr logout_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
};

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__JAKA_LIFECYCLE_CLIENT_HPP_
