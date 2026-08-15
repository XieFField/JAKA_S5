#include "massage_jaka/jaka_lifecycle_client.hpp"

#include <chrono>
#include <future>
#include <utility>

namespace massage_jaka
{

JakaLifecycleClient::JakaLifecycleClient(
    rclcpp::Node::SharedPtr node,
    std::string service_prefix)
    : node_(std::move(node))
{
    service_timeout_ = node_->declare_parameter<double>(
        "jaka_service_timeout", service_timeout_);
    login_client_ = node_->create_client<std_srvs::srv::Trigger>(
        service_prefix + "/login");
    power_on_client_ = node_->create_client<std_srvs::srv::Trigger>(
        service_prefix + "/power_on");
    enable_client_ = node_->create_client<std_srvs::srv::Trigger>(
        service_prefix + "/enable_robot");
    disable_client_ = node_->create_client<std_srvs::srv::Trigger>(
        service_prefix + "/disable_robot");
    power_off_client_ = node_->create_client<std_srvs::srv::Trigger>(
        service_prefix + "/power_off");
    logout_client_ = node_->create_client<std_srvs::srv::Trigger>(
        service_prefix + "/logout");
    stop_client_ = node_->create_client<std_srvs::srv::Trigger>(
        service_prefix + "/stop_move");
}

LifecycleResult JakaLifecycleClient::call(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & operation)
{
    const auto timeout = std::chrono::duration<double>(service_timeout_);
    if (!client->wait_for_service(timeout))
    {
        return {false, operation + " service unavailable"};
    }
    auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        return {false, operation + " service timeout"};
    }
    const auto response = future.get();
    return {response->success, response->message};
}

LifecycleResult JakaLifecycleClient::login() {return call(login_client_, "login");}
LifecycleResult JakaLifecycleClient::power_on() {return call(power_on_client_, "power_on");}
LifecycleResult JakaLifecycleClient::enable_robot() {return call(enable_client_, "enable_robot");}
LifecycleResult JakaLifecycleClient::disable_robot() {return call(disable_client_, "disable_robot");}
LifecycleResult JakaLifecycleClient::power_off() {return call(power_off_client_, "power_off");}
LifecycleResult JakaLifecycleClient::logout() {return call(logout_client_, "logout");}
LifecycleResult JakaLifecycleClient::stop_motion() {return call(stop_client_, "stop_motion");}

}  // namespace massage_jaka
