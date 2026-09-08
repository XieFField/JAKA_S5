#ifndef MASSAGE_MOTION__ROS2_CONTROL_COMPLIANCE_CONTROLLER_HPP_
#define MASSAGE_MOTION__ROS2_CONTROL_COMPLIANCE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "massage_motion/compliance_controller.hpp"

namespace massage_motion
{

struct Ros2ControlComplianceConfig
{
    std::vector<std::string> joint_names;
    std::string trajectory_controller{"jaka_s5_controller"};
    std::string admittance_controller{"massage_admittance_controller"};
    std::string controller_manager{"/controller_manager"};
    std::string joint_state_topic{"/joint_states"};
    std::string wrench_topic{"/massage_ft_broadcaster/wrench"};
    std::string status_topic{"/massage_admittance_controller/status"};
    std::string reference_topic{
        "/massage_admittance_controller/joint_references"};
    std::string cartesian_reference_frame{"world"};
    std::string cartesian_tool_frame{"massage_tool_tip"};
    double service_timeout{3.0};
    double feedback_timeout{0.5};
    double monitor_period{0.01};
    // 控制器接管初期先使用硬上限，避免低力工艺阈值被惯性瞬态误触发。
    double wrench_limit_arming_delay{1.0};
    double startup_max_absolute_wrench{5.0};
    // 工艺层通常关心接触增量而不是工具自重。启用后，以 start() 时的
    // 六维力作为零偏，feedback() 与安全监控都返回/检查去偏后的值。
    bool subtract_startup_wrench_bias{false};
};

// ros2_control 仿真后端。该类只负责控制权、参考输入和安全监控，
// 不包含推拿工艺，也不依赖 Gazebo 的专用服务。
class Ros2ControlComplianceController final : public IComplianceController
{
public:
    Ros2ControlComplianceController(
        rclcpp::Node::SharedPtr node,
        Ros2ControlComplianceConfig config);

    ~Ros2ControlComplianceController() override;

    ComplianceCapabilities capabilities() const override;

    ComplianceResult start(const ComplianceRequest & request) override;
    ComplianceResult stop() override;
    bool update_reference(const ComplianceReference & reference) override;
    ComplianceFeedback feedback() const override;
    bool reset() override;
    ComplianceStatus status() const override;

    ComplianceResult last_result() const;

    bool publish_joint_reference(
        const std::vector<double> & positions,
        const std::vector<double> & velocities = {});

    std::vector<double> current_joint_positions() const;

private:
    using SwitchController = controller_manager_msgs::srv::SwitchController;

    bool wait_for_initial_feedback();
    bool switch_controllers(
        const std::vector<std::string> & activate,
        const std::vector<std::string> & deactivate);
    void monitor_loop();
    void set_last_result(const ComplianceResult & result);
    std::vector<double> ordered_joint_positions(
        const sensor_msgs::msg::JointState & state) const;
    bool lookup_cartesian_position(
        std::array<double, 3> & position,
        std::string & error_message) const;
    bool wait_for_cartesian_position(
        std::array<double, 3> & position,
        std::string & error_message) const;

    rclcpp::Node::SharedPtr node_;
    Ros2ControlComplianceConfig config_;
    rclcpp::Logger logger_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Client<SwitchController>::SharedPtr switch_client_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
        reference_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
        wrench_subscription_;
    rclcpp::Subscription<control_msgs::msg::AdmittanceControllerState>::SharedPtr
        status_subscription_;

    mutable std::mutex data_mutex_;
    std::condition_variable feedback_condition_;
    sensor_msgs::msg::JointState latest_joint_state_;
    geometry_msgs::msg::WrenchStamped latest_wrench_;
    std::array<double, kCartesianDof> baseline_wrench_{};
    control_msgs::msg::AdmittanceControllerState latest_status_;
    std::chrono::steady_clock::time_point latest_joint_receive_time_{};
    std::chrono::steady_clock::time_point latest_wrench_receive_time_{};
    bool has_joint_state_{false};
    bool has_wrench_{false};
    bool has_status_{false};
    std::vector<double> baseline_joint_positions_;
    std::array<double, 3> baseline_cartesian_position_{};
    ComplianceRequest active_request_;
    ComplianceResult last_result_;

    mutable std::mutex operation_mutex_;
    std::thread monitor_thread_;
    std::atomic_bool stop_requested_{false};
    std::atomic<ComplianceStatus> status_{ComplianceStatus::kIdle};
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__ROS2_CONTROL_COMPLIANCE_CONTROLLER_HPP_
