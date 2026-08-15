#ifndef MASSAGE_JAKA__JAKA_COMPLIANCE_CONTROLLER_HPP_
#define MASSAGE_JAKA__JAKA_COMPLIANCE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "jaka_msgs/srv/set_admittance_config.hpp"
#include "jaka_msgs/srv/set_torque_sensor_soft_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "massage_motion/compliance_controller.hpp"

namespace massage_jaka
{

struct JakaComplianceConfig
{
    std::string wrench_topic{"/jaka_driver/wrench"};
    std::string soft_limit_service{"/jaka_driver/set_ft_soft_limit"};
    std::string config_service{"/jaka_driver/set_admittance_config"};
    std::string enable_service{"/jaka_driver/enable_admittance"};
    double service_timeout{3.0};
    double feedback_timeout{0.5};
    double monitor_period{0.01};
    std::array<double, 6> disabled_axis_soft_limits{
        5.0, 5.0, 5.0, 1.0, 1.0, 1.0};
    std::array<double, 6> constant{};
    std::array<double, 6> rebound{};
};

// JAKA 真机柔顺适配器。SDK 调用全部由 jaka_driver 服务执行，本类不链接 SDK。
class JakaComplianceController final :
    public massage_motion::IComplianceController
{
public:
    JakaComplianceController(
        rclcpp::Node::SharedPtr node,
        JakaComplianceConfig config = {});
    ~JakaComplianceController() override;

    massage_motion::ComplianceResult start(
        const massage_motion::ComplianceRequest & request) override;
    massage_motion::ComplianceResult stop() override;
    bool update_reference(
        const massage_motion::ComplianceReference & reference) override;
    massage_motion::ComplianceFeedback feedback() const override;
    bool reset() override;
    massage_motion::ComplianceStatus status() const override;

private:
    bool set_soft_limits(const massage_motion::ComplianceRequest & request);
    bool configure_axes(const massage_motion::ComplianceRequest & request);
    bool set_enabled(bool enabled, std::string & message);
    void monitor_loop();
    void set_result(const massage_motion::ComplianceResult & result);

    rclcpp::Node::SharedPtr node_;
    JakaComplianceConfig config_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Client<jaka_msgs::srv::SetTorqueSensorSoftLimit>::SharedPtr
        soft_limit_client_;
    rclcpp::Client<jaka_msgs::srv::SetAdmittanceConfig>::SharedPtr config_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;

    mutable std::mutex data_mutex_;
    geometry_msgs::msg::WrenchStamped latest_wrench_;
    std::chrono::steady_clock::time_point latest_wrench_time_{};
    bool has_wrench_{false};
    massage_motion::ComplianceRequest request_;
    massage_motion::ComplianceReference latest_reference_;
    massage_motion::ComplianceResult last_result_;

    mutable std::mutex operation_mutex_;
    std::thread monitor_thread_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<massage_motion::ComplianceStatus> status_{
        massage_motion::ComplianceStatus::kIdle};
};

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__JAKA_COMPLIANCE_CONTROLLER_HPP_
