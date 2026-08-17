#ifndef MASSAGE_JAKA__JAKA_COMPLIANCE_CONTROLLER_HPP_
#define MASSAGE_JAKA__JAKA_COMPLIANCE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "jaka_msgs/srv/set_admittance_config.hpp"
#include "jaka_msgs/srv/get_admittance_state.hpp"
#include "jaka_msgs/srv/set_force_control_frame.hpp"
#include "jaka_msgs/srv/set_torque_sensor_soft_limit.hpp"
#include "jaka_msgs/msg/robot_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "massage_motion/compliance_controller.hpp"
#include "massage_motion/wrench_subscriber.hpp"

namespace massage_jaka
{

struct JakaComplianceConfig
{
    std::string wrench_topic{"/jaka_driver/wrench"};
    std::string soft_limit_service{"/jaka_driver/set_ft_soft_limit"};
    std::string config_service{"/jaka_driver/set_admittance_config"};
    std::string enable_service{"/jaka_driver/enable_admittance"};
    std::string force_control_frame_service{
        "/jaka_driver/set_force_control_frame"};
    std::string admittance_state_service{
        "/jaka_driver/get_admittance_state"};
    std::string joint_state_topic{"/joint_states"};
    std::string robot_state_topic{"/jaka_driver/robot_states"};
    std::string wrench_frame{"Link_06"};
    std::string base_frame{"world"};
    std::string tool_frame{"massage_tool_tip"};
    std::array<std::string, 6> joint_names{
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"};
    double service_timeout{3.0};
    double feedback_timeout{0.5};
    double state_timeout{0.5};
    double monitor_period{0.01};
    std::int32_t force_control_frame{0};
    std::array<double, 6> disabled_axis_soft_limits{
        5.0, 5.0, 5.0, 1.0, 1.0, 1.0};
    // SDK ftUser：达到最大柔顺速度所需的外力/力矩，启用轴必须显式配置正值。
    std::array<double, 6> maximum_speed_wrench{};
    // SDK ftReboundFK：回到初始位置的恢复能力。
    std::array<double, 6> rebound_wrench{};
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
    // 只写入并读回真机导纳配置，不启用力控。
    massage_motion::ComplianceResult configure(
        const massage_motion::ComplianceRequest & request);
    massage_motion::ComplianceResult stop() override;
    bool update_reference(
        const massage_motion::ComplianceReference & reference) override;
    massage_motion::ComplianceFeedback feedback() const override;
    bool reset() override;
    massage_motion::ComplianceStatus status() const override;

private:
    bool set_soft_limits(const massage_motion::ComplianceRequest & request);
    bool set_force_control_frame(std::string & message);
    bool configure_axes(const massage_motion::ComplianceRequest & request);
    bool verify_configuration(
        const massage_motion::ComplianceRequest & request,
        bool expected_enabled,
        const std::string & expected_owner,
        std::string & message);
    bool robot_ready(
        std::string & message,
        bool require_stationary = true) const;
    bool set_enabled(
        bool enabled,
        std::string & message,
        bool * response_received = nullptr);
    bool capture_initial_state(std::string & message);
    bool current_tool_translation(
        std::array<double, 3> & translation,
        std::string & message);
    void monitor_loop();
    void set_result(const massage_motion::ComplianceResult & result);

    rclcpp::Node::SharedPtr node_;
    JakaComplianceConfig config_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<massage_motion::WrenchSubscriber> wrench_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<jaka_msgs::msg::RobotMsg>::SharedPtr robot_state_sub_;
    rclcpp::Client<jaka_msgs::srv::SetTorqueSensorSoftLimit>::SharedPtr
        soft_limit_client_;
    rclcpp::Client<jaka_msgs::srv::SetAdmittanceConfig>::SharedPtr config_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    rclcpp::Client<jaka_msgs::srv::SetForceControlFrame>::SharedPtr
        force_control_frame_client_;
    rclcpp::Client<jaka_msgs::srv::GetAdmittanceState>::SharedPtr
        admittance_state_client_;

    mutable std::mutex data_mutex_;
    std::vector<double> latest_joint_positions_;
    std::chrono::steady_clock::time_point latest_joint_state_time_{};
    bool has_joint_state_{false};
    jaka_msgs::msg::RobotMsg latest_robot_state_;
    std::chrono::steady_clock::time_point latest_robot_state_time_{};
    bool has_robot_state_{false};
    std::vector<double> initial_joint_positions_;
    std::array<double, 3> initial_tool_translation_{};
    massage_motion::ComplianceRequest request_;
    massage_motion::ComplianceResult last_result_;

    mutable std::mutex operation_mutex_;
    std::thread monitor_thread_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> enable_may_be_active_{false};
    std::atomic<massage_motion::ComplianceStatus> status_{
        massage_motion::ComplianceStatus::kIdle};
};

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__JAKA_COMPLIANCE_CONTROLLER_HPP_
