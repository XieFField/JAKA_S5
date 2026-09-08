#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "jaka_msgs/msg/robot_msg.hpp"
#include "jaka_msgs/srv/get_admittance_state.hpp"
#include "jaka_msgs/srv/set_admittance_config.hpp"
#include "jaka_msgs/srv/set_compliance_profile.hpp"
#include "jaka_msgs/srv/set_force_control_frame.hpp"
#include "jaka_msgs/srv/set_torque_sensor_soft_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "massage_jaka/force_hold_analysis.hpp"
#include "massage_jaka/jaka_compliance_controller.hpp"

namespace
{

using SteadyClock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;
std::atomic<bool> interruption_requested{false};

void handle_signal(int)
{
    interruption_requested.store(true);
}

bool finite_positive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

double norm3(double x, double y, double z)
{
    return std::sqrt(x * x + y * y + z * z);
}

std::string default_csv_path()
{
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return "/tmp/massage_r10_contact_hold_" +
           std::to_string(milliseconds) + ".csv";
}

double tool_z_to_world_minus_z_error(
    const geometry_msgs::msg::Quaternion & quaternion)
{
    const double quaternion_norm = std::sqrt(
        quaternion.x * quaternion.x + quaternion.y * quaternion.y +
        quaternion.z * quaternion.z + quaternion.w * quaternion.w);
    if (!std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-9)
    {
        return std::numeric_limits<double>::infinity();
    }
    const double x = quaternion.x / quaternion_norm;
    const double y = quaternion.y / quaternion_norm;
    const double tool_z_world_z = 1.0 - 2.0 * (x * x + y * y);
    return std::acos(std::clamp(-tool_z_world_z, -1.0, 1.0));
}

struct RobotStateSnapshot
{
    jaka_msgs::msg::RobotMsg state;
    SteadyClock::time_point received_at{};
    bool received{false};
};

bool robot_ready(const RobotStateSnapshot & snapshot, double timeout)
{
    return snapshot.received &&
           std::chrono::duration<double>(
               SteadyClock::now() - snapshot.received_at).count() <= timeout &&
           snapshot.state.motion_state == 0 && snapshot.state.power_state == 1 &&
           snapshot.state.servo_state == 1 && snapshot.state.collision_state == 0;
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(
        argc, argv, rclcpp::InitOptions(),
        rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    auto node = std::make_shared<rclcpp::Node>(
        "real_contact_force_hold_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    int exit_code = 1;
    std::shared_ptr<massage_jaka::JakaComplianceController> controller;
    std::function<bool(std::string &)> restore_configuration;
    bool restore_required = false;
    std::ofstream csv;
    try
    {
        bool activate = false;
        bool parameters_confirmed = false;
        bool execute_contact_search = false;
        double target_wrench_z = -1.0;
        double contact_threshold = 0.5;
        double target_tolerance = 0.3;
        double maximum_force = 5.0;
        double maximum_torque = 1.0;
        double maximum_speed_wrench = 5.0;
        double rebound_wrench = 0.0;
        std::int64_t sensor_compensation = 1;
        std::int64_t compliance_type = 1;
        double compliance_linear_speed_limit_mm_s = 2.0;
        double compliance_angular_speed_limit_rad_s = 0.05;
        double approach_linear_speed_limit_mm_s = 2.0;
        double approach_angular_speed_limit_rad_s = 0.05;
        double baseline_duration = 2.0;
        double maximum_baseline_force_bias = 0.5;
        double maximum_baseline_torque_bias = 0.1;
        double contact_timeout = 5.0;
        double hold_duration = 3.0;
        double minimum_hold_in_tolerance_ratio = 0.8;
        double maximum_hold_standard_deviation = 0.25;
        double maximum_hold_sample_gap = 0.2;
        double maximum_hold_mean_absolute_error = 0.25;
        double maximum_hold_terminal_error = 0.3;
        std::int64_t minimum_hold_samples = 20;
        std::int64_t contact_confirmation_samples = 3;
        std::int64_t target_confirmation_samples = 5;
        double maximum_joint_displacement = 0.05;
        double maximum_linear_displacement = 0.012;
        double maximum_transverse_displacement = 0.002;
        double maximum_wrong_direction_displacement = 0.001;
        double expected_motion_direction_z = -1.0;
        double expected_measured_force_sign_z = 1.0;
        double maximum_tool_axis_error_degrees = 3.0;
        double contact_x = -0.471238630147741;
        double contact_y = 0.156275068796867;
        double contact_z = 0.281381721182422;
        double precontact_clearance = 0.002;
        double maximum_precontact_position_error = 0.003;
        double readiness_timeout = 8.0;
        double service_timeout = 3.0;
        double feedback_timeout = 0.5;
        double state_timeout = 0.5;
        std::string wrench_topic{"/massage/ft_sensor/wrench_world"};
        std::string wrench_frame{"massage_tool_tip_world_aligned"};
        std::string base_frame{"world"};
        std::string tool_frame{"massage_tool_tip"};
        std::string csv_path;
        node->get_parameter_or("activate", activate, activate);
        node->get_parameter_or(
            "parameters_confirmed", parameters_confirmed, parameters_confirmed);
        node->get_parameter_or(
            "execute_contact_search", execute_contact_search,
            execute_contact_search);
        node->get_parameter_or("target_wrench_z", target_wrench_z, target_wrench_z);
        node->get_parameter_or(
            "contact_threshold", contact_threshold, contact_threshold);
        node->get_parameter_or(
            "target_tolerance", target_tolerance, target_tolerance);
        node->get_parameter_or("maximum_force", maximum_force, maximum_force);
        node->get_parameter_or("maximum_torque", maximum_torque, maximum_torque);
        node->get_parameter_or(
            "maximum_speed_wrench", maximum_speed_wrench,
            maximum_speed_wrench);
        node->get_parameter_or("rebound_wrench", rebound_wrench, rebound_wrench);
        node->get_parameter_or(
            "sensor_compensation", sensor_compensation, sensor_compensation);
        node->get_parameter_or(
            "compliance_type", compliance_type, compliance_type);
        node->get_parameter_or(
            "compliance_linear_speed_limit_mm_s",
            compliance_linear_speed_limit_mm_s,
            compliance_linear_speed_limit_mm_s);
        node->get_parameter_or(
            "compliance_angular_speed_limit_rad_s",
            compliance_angular_speed_limit_rad_s,
            compliance_angular_speed_limit_rad_s);
        node->get_parameter_or(
            "approach_linear_speed_limit_mm_s",
            approach_linear_speed_limit_mm_s,
            approach_linear_speed_limit_mm_s);
        node->get_parameter_or(
            "approach_angular_speed_limit_rad_s",
            approach_angular_speed_limit_rad_s,
            approach_angular_speed_limit_rad_s);
        node->get_parameter_or(
            "baseline_duration", baseline_duration, baseline_duration);
        node->get_parameter_or(
            "maximum_baseline_force_bias", maximum_baseline_force_bias,
            maximum_baseline_force_bias);
        node->get_parameter_or(
            "maximum_baseline_torque_bias", maximum_baseline_torque_bias,
            maximum_baseline_torque_bias);
        node->get_parameter_or("contact_timeout", contact_timeout, contact_timeout);
        node->get_parameter_or("hold_duration", hold_duration, hold_duration);
        node->get_parameter_or(
            "minimum_hold_in_tolerance_ratio",
            minimum_hold_in_tolerance_ratio,
            minimum_hold_in_tolerance_ratio);
        node->get_parameter_or(
            "maximum_hold_standard_deviation",
            maximum_hold_standard_deviation,
            maximum_hold_standard_deviation);
        node->get_parameter_or(
            "maximum_hold_sample_gap", maximum_hold_sample_gap,
            maximum_hold_sample_gap);
        node->get_parameter_or(
            "maximum_hold_mean_absolute_error",
            maximum_hold_mean_absolute_error,
            maximum_hold_mean_absolute_error);
        node->get_parameter_or(
            "maximum_hold_terminal_error", maximum_hold_terminal_error,
            maximum_hold_terminal_error);
        node->get_parameter_or(
            "minimum_hold_samples", minimum_hold_samples, minimum_hold_samples);
        node->get_parameter_or(
            "contact_confirmation_samples", contact_confirmation_samples,
            contact_confirmation_samples);
        node->get_parameter_or(
            "target_confirmation_samples", target_confirmation_samples,
            target_confirmation_samples);
        node->get_parameter_or(
            "maximum_joint_displacement", maximum_joint_displacement,
            maximum_joint_displacement);
        node->get_parameter_or(
            "maximum_linear_displacement", maximum_linear_displacement,
            maximum_linear_displacement);
        node->get_parameter_or(
            "maximum_transverse_displacement", maximum_transverse_displacement,
            maximum_transverse_displacement);
        node->get_parameter_or(
            "maximum_wrong_direction_displacement",
            maximum_wrong_direction_displacement,
            maximum_wrong_direction_displacement);
        node->get_parameter_or(
            "expected_motion_direction_z", expected_motion_direction_z,
            expected_motion_direction_z);
        node->get_parameter_or(
            "expected_measured_force_sign_z", expected_measured_force_sign_z,
            expected_measured_force_sign_z);
        node->get_parameter_or(
            "maximum_tool_axis_error_degrees", maximum_tool_axis_error_degrees,
            maximum_tool_axis_error_degrees);
        node->get_parameter_or("contact_x", contact_x, contact_x);
        node->get_parameter_or("contact_y", contact_y, contact_y);
        node->get_parameter_or("contact_z", contact_z, contact_z);
        node->get_parameter_or(
            "precontact_clearance", precontact_clearance,
            precontact_clearance);
        node->get_parameter_or(
            "maximum_precontact_position_error",
            maximum_precontact_position_error,
            maximum_precontact_position_error);
        node->get_parameter_or(
            "readiness_timeout", readiness_timeout, readiness_timeout);
        node->get_parameter_or("service_timeout", service_timeout, service_timeout);
        node->get_parameter_or(
            "feedback_timeout", feedback_timeout, feedback_timeout);
        node->get_parameter_or("state_timeout", state_timeout, state_timeout);
        node->get_parameter_or("wrench_topic", wrench_topic, wrench_topic);
        node->get_parameter_or("wrench_frame", wrench_frame, wrench_frame);
        node->get_parameter_or("base_frame", base_frame, base_frame);
        node->get_parameter_or("tool_frame", tool_frame, tool_frame);
        node->get_parameter_or("csv_path", csv_path, std::string{});

        const double target_force = std::abs(target_wrench_z);
        const bool parameters_valid =
            std::isfinite(target_wrench_z) && target_force > 0.0 &&
            target_force <= 3.0 && finite_positive(contact_threshold) &&
            finite_positive(target_tolerance) &&
            contact_threshold < target_force - target_tolerance &&
            finite_positive(maximum_force) && maximum_force <= 10.0 &&
            target_force + target_tolerance < maximum_force &&
            finite_positive(maximum_torque) && maximum_torque <= 2.0 &&
            finite_positive(maximum_speed_wrench) &&
            std::isfinite(rebound_wrench) && rebound_wrench >= 0.0 &&
            sensor_compensation == 1 && compliance_type == 1 &&
            finite_positive(compliance_linear_speed_limit_mm_s) &&
            compliance_linear_speed_limit_mm_s <= 5.0 &&
            finite_positive(compliance_angular_speed_limit_rad_s) &&
            compliance_angular_speed_limit_rad_s <= 0.2 &&
            finite_positive(approach_linear_speed_limit_mm_s) &&
            approach_linear_speed_limit_mm_s <= 5.0 &&
            finite_positive(approach_angular_speed_limit_rad_s) &&
            approach_angular_speed_limit_rad_s <= 0.2 &&
            finite_positive(baseline_duration) &&
            finite_positive(maximum_baseline_force_bias) &&
            finite_positive(maximum_baseline_torque_bias) &&
            finite_positive(contact_timeout) && contact_timeout <= 10.0 &&
            finite_positive(hold_duration) && hold_duration <= 10.0 &&
            std::isfinite(minimum_hold_in_tolerance_ratio) &&
            minimum_hold_in_tolerance_ratio >= 0.5 &&
            minimum_hold_in_tolerance_ratio <= 1.0 &&
            finite_positive(maximum_hold_standard_deviation) &&
            finite_positive(maximum_hold_sample_gap) &&
            maximum_hold_sample_gap <= feedback_timeout &&
            std::isfinite(maximum_hold_mean_absolute_error) &&
            maximum_hold_mean_absolute_error >= 0.0 &&
            maximum_hold_mean_absolute_error <= target_tolerance &&
            std::isfinite(maximum_hold_terminal_error) &&
            maximum_hold_terminal_error >= 0.0 &&
            maximum_hold_terminal_error <= target_tolerance &&
            minimum_hold_samples >= 10 && contact_confirmation_samples >= 2 &&
            target_confirmation_samples >= 2 &&
            finite_positive(maximum_joint_displacement) &&
            maximum_joint_displacement <= 0.1 &&
            finite_positive(maximum_linear_displacement) &&
            maximum_linear_displacement <= 0.02 &&
            finite_positive(maximum_transverse_displacement) &&
            maximum_transverse_displacement <= 0.005 &&
            finite_positive(maximum_wrong_direction_displacement) &&
            maximum_wrong_direction_displacement <= 0.002 &&
            std::isfinite(expected_motion_direction_z) &&
            std::abs(std::abs(expected_motion_direction_z) - 1.0) <= 1.0e-9 &&
            target_wrench_z * expected_motion_direction_z > 0.0 &&
            std::isfinite(expected_measured_force_sign_z) &&
            std::abs(std::abs(expected_measured_force_sign_z) - 1.0) <= 1.0e-9 &&
            expected_motion_direction_z * expected_measured_force_sign_z < 0.0 &&
            finite_positive(maximum_tool_axis_error_degrees) &&
            maximum_tool_axis_error_degrees <= 10.0 &&
            std::isfinite(contact_x) && std::isfinite(contact_y) &&
            std::isfinite(contact_z) && finite_positive(precontact_clearance) &&
            precontact_clearance <= 0.020 &&
            finite_positive(maximum_precontact_position_error) &&
            maximum_precontact_position_error <= 0.010 &&
            finite_positive(readiness_timeout) && finite_positive(service_timeout) &&
            finite_positive(feedback_timeout) && finite_positive(state_timeout) &&
            !wrench_topic.empty() && !wrench_frame.empty() &&
            !base_frame.empty() && !tool_frame.empty();
        if (!parameters_valid)
        {
            throw std::invalid_argument("R10 接触保持参数无效或超过测试入口硬边界");
        }
        if (activate && !parameters_confirmed)
        {
            throw std::invalid_argument(
                "activate=true 时必须显式设置 parameters_confirmed=true");
        }
        if (execute_contact_search && !activate)
        {
            throw std::invalid_argument(
                "execute_contact_search=true 时必须同时设置 activate=true");
        }
        if (!activate)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "R10 CONTACT FORCE HOLD: SAFE IDLE: activate=false; "
                "未创建 JAKA 柔顺控制器，未清零、未配置、未启用、未运动");
            exit_code = 0;
        }
        else
        {
            if (csv_path.empty())
            {
                csv_path = default_csv_path();
            }
            csv.open(csv_path, std::ios::out | std::ios::trunc);
            if (!csv)
            {
                throw std::runtime_error("无法创建 R10 CSV: " + csv_path);
            }
            csv << "phase,elapsed_s,wrench_stamp_ns,fx_n,fy_n,fz_n,"
                   "tx_nm,ty_nm,tz_nm,normal_force_delta_n,tcp_x_m,tcp_y_m,"
                   "tcp_z_m,tcp_delta_x_m,tcp_delta_y_m,tcp_delta_z_m,"
                   "maximum_joint_delta_rad,motion_state,power_state,"
                   "servo_state,collision_state,compliance_status\n";
            csv << std::setprecision(17);

            massage_jaka::JakaComplianceConfig controller_config;
            controller_config.wrench_topic = wrench_topic;
            controller_config.wrench_frame = wrench_frame;
            controller_config.base_frame = base_frame;
            controller_config.tool_frame = tool_frame;
            controller_config.service_timeout = service_timeout;
            controller_config.feedback_timeout = feedback_timeout;
            controller_config.state_timeout = state_timeout;
            controller_config.force_control_frame = 1;
            controller_config.maximum_speed_wrench[2] = maximum_speed_wrench;
            controller_config.rebound_wrench[2] = rebound_wrench;
            controller = std::make_shared<massage_jaka::JakaComplianceController>(
                node, controller_config);
            auto zero_client = node->create_client<std_srvs::srv::Trigger>(
                "/jaka_driver/zero_ft_sensor");
            auto admittance_state_client =
                node->create_client<jaka_msgs::srv::GetAdmittanceState>(
                    "/jaka_driver/get_admittance_state");
            auto compliance_profile_client =
                node->create_client<jaka_msgs::srv::SetComplianceProfile>(
                    "/jaka_driver/set_compliance_profile");
            auto admittance_config_client =
                node->create_client<jaka_msgs::srv::SetAdmittanceConfig>(
                    "/jaka_driver/set_admittance_config");
            auto force_control_frame_client =
                node->create_client<jaka_msgs::srv::SetForceControlFrame>(
                    "/jaka_driver/set_force_control_frame");
            auto soft_limit_client =
                node->create_client<jaka_msgs::srv::SetTorqueSensorSoftLimit>(
                    "/jaka_driver/set_ft_soft_limit");
            tf2_ros::Buffer tf_buffer(node->get_clock());
            tf2_ros::TransformListener tf_listener(tf_buffer);
            std::mutex robot_state_mutex;
            RobotStateSnapshot robot_state;
            auto robot_state_subscription =
                node->create_subscription<jaka_msgs::msg::RobotMsg>(
                    "/jaka_driver/robot_states", rclcpp::SensorDataQoS(),
                    [&](const jaka_msgs::msg::RobotMsg::SharedPtr message)
                    {
                        std::lock_guard<std::mutex> lock(robot_state_mutex);
                        robot_state.state = *message;
                        robot_state.received_at = SteadyClock::now();
                        robot_state.received = true;
                    });
            const auto robot_snapshot = [&]()
                {
                    std::lock_guard<std::mutex> lock(robot_state_mutex);
                    return robot_state;
                };
            const auto lookup_tool = [&]()
                {
                    return tf_buffer.lookupTransform(
                        base_frame, tool_frame, tf2::TimePointZero);
                };

            const auto readiness_deadline = SteadyClock::now() +
                std::chrono::duration<double>(readiness_timeout);
            massage_motion::ComplianceFeedback feedback;
            geometry_msgs::msg::TransformStamped initial_tool;
            while (!interruption_requested.load() &&
                   SteadyClock::now() < readiness_deadline)
            {
                feedback = controller->feedback();
                try
                {
                    initial_tool = lookup_tool();
                }
                catch (const tf2::TransformException &)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
                if (!feedback.stale && feedback.joint_positions.size() == 6U &&
                    robot_ready(robot_snapshot(), state_timeout))
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (interruption_requested.load())
            {
                throw std::runtime_error("R10 在就绪阶段被中断");
            }
            feedback = controller->feedback();
            if (feedback.stale || feedback.joint_positions.size() != 6U ||
                !robot_ready(robot_snapshot(), state_timeout))
            {
                throw std::runtime_error(
                    "R10 就绪失败：关节、机器人状态、世界系 FT 或 TF 不满足要求");
            }
            initial_tool = lookup_tool();
            const double axis_error = tool_z_to_world_minus_z_error(
                initial_tool.transform.rotation);
            const double maximum_axis_error =
                maximum_tool_axis_error_degrees * kPi / 180.0;
            if (!std::isfinite(axis_error))
            {
                throw std::runtime_error(
                    "无法计算按摩头 Z 轴相对世界 -Z 的姿态误差");
            }
            if (execute_contact_search && axis_error > maximum_axis_error)
            {
                throw std::runtime_error(
                    "R10-C 禁止启动：按摩头 Z 轴未对准世界 -Z，axis_error_deg=" +
                    std::to_string(axis_error * 180.0 / kPi) +
                    ", limit_deg=" +
                    std::to_string(maximum_tool_axis_error_degrees));
            }
            const double expected_precontact_z = contact_z + precontact_clearance;
            const double precontact_dx =
                initial_tool.transform.translation.x - contact_x;
            const double precontact_dy =
                initial_tool.transform.translation.y - contact_y;
            const double precontact_dz =
                initial_tool.transform.translation.z - expected_precontact_z;
            const double precontact_position_error = norm3(
                precontact_dx, precontact_dy, precontact_dz);
            if (!std::isfinite(precontact_position_error))
            {
                throw std::runtime_error("无法计算 R10-C 预接触位置误差");
            }
            if (execute_contact_search &&
                precontact_position_error > maximum_precontact_position_error)
            {
                throw std::runtime_error(
                    "R10-C 禁止启动：当前按摩头不在 C0 预接触位，position_error=" +
                    std::to_string(precontact_position_error) +
                    " m, limit=" +
                    std::to_string(maximum_precontact_position_error) + " m");
            }
            RCLCPP_INFO(
                node->get_logger(),
                "R10 就绪通过: mode=%s, tool_axis_error=%.3f deg, "
                "orientation_gate=%s, precontact_error=%.6f m, "
                "position_gate=%s, target_wrench_z=%.3f N, "
                "measured_force_sign=%+.0f, contact_threshold=%.3f N, "
                "maximum_force=%.3f N",
                execute_contact_search ? "contact_search" : "configure_only",
                axis_error * 180.0 / kPi,
                execute_contact_search ? "enforced" : "not_required_no_motion",
                precontact_position_error,
                execute_contact_search ? "enforced" : "not_required_no_motion",
                target_wrench_z,
                expected_measured_force_sign_z, contact_threshold, maximum_force);

            const auto service_wait = std::chrono::duration<double>(service_timeout);
            const auto read_admittance_state = [&]()
                {
                    if (!admittance_state_client->wait_for_service(service_wait))
                    {
                        throw std::runtime_error(
                            "get_admittance_state 服务不可用");
                    }
                    auto future = admittance_state_client->async_send_request(
                        std::make_shared<
                            jaka_msgs::srv::GetAdmittanceState::Request>());
                    if (future.wait_for(service_wait) != std::future_status::ready)
                    {
                        throw std::runtime_error(
                            "get_admittance_state 服务超时");
                    }
                    auto response = future.get();
                    if (!response->success)
                    {
                        throw std::runtime_error(
                            "读取 JAKA 导纳配置失败: " + response->message);
                    }
                    return response;
                };
            const auto original_state = read_admittance_state();
            if (original_state->force_control_enabled ||
                original_state->control_owner != "idle")
            {
                throw std::runtime_error(
                    "R10 启动前力控未关闭或控制权非 idle: enabled=" +
                    std::string(original_state->force_control_enabled ?
                        "true" : "false") + ", owner=" +
                    original_state->control_owner);
            }

            restore_configuration = [
                original_state, admittance_state_client,
                compliance_profile_client, admittance_config_client,
                force_control_frame_client, soft_limit_client,
                service_timeout](std::string & message)
                {
                    const auto timeout =
                        std::chrono::duration<double>(service_timeout);
                    const auto failed = [&](const std::string & detail)
                        {
                            message = detail;
                            return false;
                        };

                    if (!admittance_state_client->wait_for_service(timeout))
                    {
                        return failed("恢复前 get_admittance_state 服务不可用");
                    }
                    auto idle_state_future =
                        admittance_state_client->async_send_request(
                            std::make_shared<
                                jaka_msgs::srv::GetAdmittanceState::Request>());
                    if (idle_state_future.wait_for(timeout) !=
                        std::future_status::ready)
                    {
                        return failed("恢复前导纳关闭状态读回超时");
                    }
                    const auto idle_state = idle_state_future.get();
                    if (!idle_state->success || idle_state->force_control_enabled ||
                        idle_state->control_owner != "idle")
                    {
                        return failed(
                            "恢复被拒绝：未确认 force_control=false 且 owner=idle");
                    }

                    if (!admittance_config_client->wait_for_service(timeout))
                    {
                        return failed("恢复导纳轴配置时服务不可用");
                    }
                    for (std::size_t axis = 0; axis < 6U; ++axis)
                    {
                        auto request = std::make_shared<
                            jaka_msgs::srv::SetAdmittanceConfig::Request>();
                        request->axis = static_cast<std::int32_t>(axis);
                        request->option = original_state->axis_options[axis];
                        request->maximum_speed_wrench =
                            original_state->maximum_speed_wrench[axis];
                        request->constant_wrench =
                            original_state->constant_wrench[axis];
                        request->normal_track =
                            original_state->normal_track[axis];
                        request->rebound_wrench =
                            original_state->rebound_wrench[axis];
                        auto future =
                            admittance_config_client->async_send_request(request);
                        if (future.wait_for(timeout) != std::future_status::ready ||
                            !future.get()->success)
                        {
                            return failed(
                                "恢复导纳轴配置失败，axis=" +
                                std::to_string(axis));
                        }
                    }

                    if (!force_control_frame_client->wait_for_service(timeout))
                    {
                        return failed("恢复力控坐标系时服务不可用");
                    }
                    auto frame_request = std::make_shared<
                        jaka_msgs::srv::SetForceControlFrame::Request>();
                    frame_request->frame = original_state->force_control_frame;
                    auto frame_future =
                        force_control_frame_client->async_send_request(frame_request);
                    if (frame_future.wait_for(timeout) != std::future_status::ready ||
                        !frame_future.get()->success)
                    {
                        return failed("恢复力控坐标系失败");
                    }

                    if (!soft_limit_client->wait_for_service(timeout))
                    {
                        return failed("恢复 FT 软限幅时服务不可用");
                    }
                    auto limit_request = std::make_shared<
                        jaka_msgs::srv::SetTorqueSensorSoftLimit::Request>();
                    limit_request->limits = original_state->soft_limits;
                    auto limit_future =
                        soft_limit_client->async_send_request(limit_request);
                    if (limit_future.wait_for(timeout) != std::future_status::ready ||
                        !limit_future.get()->success)
                    {
                        return failed("恢复 FT 软限幅失败");
                    }

                    if (!compliance_profile_client->wait_for_service(timeout))
                    {
                        return failed("恢复柔顺 profile 时服务不可用");
                    }
                    auto profile_request = std::make_shared<
                        jaka_msgs::srv::SetComplianceProfile::Request>();
                    profile_request->sensor_compensation =
                        original_state->sensor_compensation;
                    profile_request->compliance_type =
                        original_state->compliance_type;
                    profile_request->compliance_linear_speed_limit_mm_s =
                        original_state->compliance_linear_speed_limit_mm_s;
                    profile_request->compliance_angular_speed_limit_rad_s =
                        original_state->compliance_angular_speed_limit_rad_s;
                    profile_request->approach_linear_speed_limit_mm_s =
                        original_state->approach_linear_speed_limit_mm_s;
                    profile_request->approach_angular_speed_limit_rad_s =
                        original_state->approach_angular_speed_limit_rad_s;
                    auto profile_future =
                        compliance_profile_client->async_send_request(profile_request);
                    if (profile_future.wait_for(timeout) !=
                            std::future_status::ready ||
                        !profile_future.get()->success)
                    {
                        return failed("恢复柔顺 profile 失败");
                    }

                    auto state_future = admittance_state_client->async_send_request(
                        std::make_shared<
                            jaka_msgs::srv::GetAdmittanceState::Request>());
                    if (state_future.wait_for(timeout) != std::future_status::ready)
                    {
                        return failed("恢复后导纳配置读回超时");
                    }
                    const auto restored = state_future.get();
                    if (!restored->success || restored->force_control_enabled ||
                        restored->control_owner != "idle")
                    {
                        return failed(
                            "恢复后未确认 force_control=false 且 owner=idle");
                    }
                    constexpr double tolerance = 1.0e-6;
                    const auto equal = [](double left, double right)
                        {return std::abs(left - right) <= tolerance;};
                    bool matches =
                        restored->force_control_frame ==
                            original_state->force_control_frame &&
                        restored->sensor_compensation ==
                            original_state->sensor_compensation &&
                        restored->compliance_type ==
                            original_state->compliance_type &&
                        equal(restored->compliance_linear_speed_limit_mm_s,
                            original_state->compliance_linear_speed_limit_mm_s) &&
                        equal(restored->compliance_angular_speed_limit_rad_s,
                            original_state->compliance_angular_speed_limit_rad_s) &&
                        equal(restored->approach_linear_speed_limit_mm_s,
                            original_state->approach_linear_speed_limit_mm_s) &&
                        equal(restored->approach_angular_speed_limit_rad_s,
                            original_state->approach_angular_speed_limit_rad_s);
                    for (std::size_t axis = 0; axis < 6U && matches; ++axis)
                    {
                        matches =
                            restored->axis_options[axis] ==
                                original_state->axis_options[axis] &&
                            restored->normal_track[axis] ==
                                original_state->normal_track[axis] &&
                            equal(restored->soft_limits[axis],
                                original_state->soft_limits[axis]) &&
                            equal(restored->maximum_speed_wrench[axis],
                                original_state->maximum_speed_wrench[axis]) &&
                            equal(restored->constant_wrench[axis],
                                original_state->constant_wrench[axis]) &&
                            equal(restored->rebound_wrench[axis],
                                original_state->rebound_wrench[axis]);
                    }
                    if (!matches)
                    {
                        return failed("恢复后的 JAKA 导纳配置与启动快照不一致");
                    }
                    message = "原始 JAKA 导纳配置已恢复并读回一致";
                    return true;
                };

            restore_required = true;
            if (!compliance_profile_client->wait_for_service(service_wait))
            {
                throw std::runtime_error("set_compliance_profile 服务不可用");
            }
            auto profile_request = std::make_shared<
                jaka_msgs::srv::SetComplianceProfile::Request>();
            profile_request->sensor_compensation =
                static_cast<std::int32_t>(sensor_compensation);
            profile_request->compliance_type =
                static_cast<std::int32_t>(compliance_type);
            profile_request->compliance_linear_speed_limit_mm_s =
                compliance_linear_speed_limit_mm_s;
            profile_request->compliance_angular_speed_limit_rad_s =
                compliance_angular_speed_limit_rad_s;
            profile_request->approach_linear_speed_limit_mm_s =
                approach_linear_speed_limit_mm_s;
            profile_request->approach_angular_speed_limit_rad_s =
                approach_angular_speed_limit_rad_s;
            auto profile_future =
                compliance_profile_client->async_send_request(profile_request);
            if (profile_future.wait_for(service_wait) != std::future_status::ready)
            {
                throw std::runtime_error("set_compliance_profile 服务超时");
            }
            const auto profile_response = profile_future.get();
            constexpr double profile_tolerance = 1.0e-6;
            const auto profile_equal = [](double left, double right)
                {return std::abs(left - right) <= profile_tolerance;};
            if (!profile_response->success ||
                profile_response->actual_sensor_compensation !=
                    sensor_compensation ||
                profile_response->actual_compliance_type != compliance_type ||
                !profile_equal(
                    profile_response->actual_compliance_linear_speed_limit_mm_s,
                    compliance_linear_speed_limit_mm_s) ||
                !profile_equal(
                    profile_response->actual_compliance_angular_speed_limit_rad_s,
                    compliance_angular_speed_limit_rad_s) ||
                !profile_equal(
                    profile_response->actual_approach_linear_speed_limit_mm_s,
                    approach_linear_speed_limit_mm_s) ||
                !profile_equal(
                    profile_response->actual_approach_angular_speed_limit_rad_s,
                    approach_angular_speed_limit_rad_s))
            {
                throw std::runtime_error(
                    "R10 恒力模式或速度上限写入/读回不一致: " +
                    profile_response->message);
            }
            RCLCPP_INFO(
                node->get_logger(),
                "R10 恒力 profile 已读回: type=%ld, sensor_compensation=%ld, "
                "compliant_speed=[%.3f mm/s %.3f rad/s], "
                "approach_speed=[%.3f mm/s %.3f rad/s]",
                compliance_type, sensor_compensation,
                compliance_linear_speed_limit_mm_s,
                compliance_angular_speed_limit_rad_s,
                approach_linear_speed_limit_mm_s,
                approach_angular_speed_limit_rad_s);

            if (!zero_client->wait_for_service(
                    std::chrono::duration<double>(service_timeout)))
            {
                throw std::runtime_error("zero_ft_sensor 服务不可用");
            }
            auto zero_future = zero_client->async_send_request(
                std::make_shared<std_srvs::srv::Trigger::Request>());
            if (zero_future.wait_for(std::chrono::duration<double>(service_timeout)) !=
                    std::future_status::ready || !zero_future.get()->success)
            {
                throw std::runtime_error("R10 FT 清零失败或超时");
            }

            const auto baseline_started = SteadyClock::now();
            const auto baseline_deadline = baseline_started +
                std::chrono::duration<double>(baseline_duration);
            std::array<double, 6> baseline_sum{};
            std::size_t baseline_count = 0U;
            std::int64_t last_stamp = std::numeric_limits<std::int64_t>::min();
            while (!interruption_requested.load() &&
                   SteadyClock::now() < baseline_deadline)
            {
                feedback = controller->feedback();
                if (!feedback.stale &&
                    feedback.wrench_stamp_nanoseconds != last_stamp)
                {
                    last_stamp = feedback.wrench_stamp_nanoseconds;
                    for (std::size_t axis = 0; axis < baseline_sum.size(); ++axis)
                    {
                        baseline_sum[axis] += feedback.wrench[axis];
                    }
                    ++baseline_count;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (interruption_requested.load())
            {
                throw std::runtime_error("R10 在 FT 基线阶段被中断");
            }
            if (baseline_count < 10U)
            {
                throw std::runtime_error("R10 FT 清零后有效基线样本不足");
            }
            std::array<double, 6> baseline_mean{};
            for (std::size_t axis = 0; axis < baseline_mean.size(); ++axis)
            {
                baseline_mean[axis] =
                    baseline_sum[axis] / static_cast<double>(baseline_count);
                const double limit = axis < 3U ?
                    maximum_baseline_force_bias : maximum_baseline_torque_bias;
                if (std::abs(baseline_mean[axis]) > limit)
                {
                    throw std::runtime_error(
                        "R10 FT 清零后基线均值超限，axis=" +
                        std::to_string(axis) + ", mean=" +
                        std::to_string(baseline_mean[axis]));
                }
            }

            feedback = controller->feedback();
            const auto initial_joints = feedback.joint_positions;
            initial_tool = lookup_tool();
            const std::array<double, 3> initial_translation{
                initial_tool.transform.translation.x,
                initial_tool.transform.translation.y,
                initial_tool.transform.translation.z};

            massage_motion::ComplianceRequest request;
            request.request_id = "r10_world_z_contact_force_hold";
            request.enabled_axes[2] = true;
            request.target_wrench[2] = target_wrench_z;
            request.max_absolute_wrench = {
                maximum_force, maximum_force, maximum_force,
                maximum_torque, maximum_torque, maximum_torque};
            request.max_joint_displacement = maximum_joint_displacement;
            request.max_linear_displacement = maximum_linear_displacement;
            request.timeout = contact_timeout + hold_duration + 2.0;
            if (!execute_contact_search)
            {
                const auto configure_result = controller->configure(request);
                if (!configure_result.success)
                {
                    throw std::runtime_error(
                        "R10 配置/读回预检失败: " + configure_result.message);
                }
                std::string restore_message;
                if (!restore_configuration ||
                    !restore_configuration(restore_message))
                {
                    throw std::runtime_error(
                        "R10 配置预检完成但恢复失败: " + restore_message);
                }
                restore_required = false;
                RCLCPP_INFO(
                    node->get_logger(),
                    "R10 CONFIGURE-ONLY: PASS: 恒力模式、速度上限、世界系力控、"
                    "六轴参数和软限幅写入/读回通过；未启用导纳，未执行接触搜索；%s",
                    restore_message.c_str());
                exit_code = 0;
            }
            else
            {
                const auto start_result = controller->start(request);
                if (!start_result.success)
                {
                    throw std::runtime_error(
                        "R10 JAKA 导纳启动失败: " + start_result.message);
                }

                RCLCPP_WARN(
                node->get_logger(),
                "R10 ACTIVE: 只允许固定软测试块；禁止手持测试块或人体接触。"
                "未在 %.1f s 内接触或位移越界将自动关闭",
                contact_timeout);
            const auto active_started = SteadyClock::now();
            const auto contact_deadline = active_started +
                std::chrono::duration<double>(contact_timeout);
            SteadyClock::time_point hold_started{};
            bool contact_detected = false;
            bool target_reached = false;
            std::int64_t active_last_stamp =
                std::numeric_limits<std::int64_t>::min();
            std::size_t contact_consecutive = 0U;
            std::size_t target_consecutive = 0U;
            double peak_normal_force = 0.0;
            double peak_joint_delta = 0.0;
            double peak_linear_displacement = 0.0;
            double peak_transverse_displacement = 0.0;
            std::vector<massage_jaka::ForceHoldSample> hold_samples;

            while (!interruption_requested.load())
            {
                feedback = controller->feedback();
                if (feedback.stale || feedback.joint_positions.size() != 6U ||
                    feedback.status != massage_motion::ComplianceStatus::kActive)
                {
                    const auto stopped = controller->stop();
                    throw std::runtime_error(
                        "R10 导纳反馈失效或保护线程提前退出: " + stopped.message);
                }
                const auto state = robot_snapshot();
                if (!state.received || state.state.power_state != 1 ||
                    state.state.servo_state != 1 || state.state.collision_state != 0)
                {
                    throw std::runtime_error("R10 机器人上电、使能或碰撞状态异常");
                }
                if (feedback.wrench_stamp_nanoseconds == active_last_stamp)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                active_last_stamp = feedback.wrench_stamp_nanoseconds;
                const auto tool = lookup_tool();
                const double dx = tool.transform.translation.x - initial_translation[0];
                const double dy = tool.transform.translation.y - initial_translation[1];
                const double dz = tool.transform.translation.z - initial_translation[2];
                const double linear_displacement = norm3(dx, dy, dz);
                const double transverse_displacement = std::hypot(dx, dy);
                peak_linear_displacement = std::max(
                    peak_linear_displacement, linear_displacement);
                peak_transverse_displacement = std::max(
                    peak_transverse_displacement, transverse_displacement);
                double current_joint_delta = 0.0;
                for (std::size_t index = 0; index < initial_joints.size(); ++index)
                {
                    current_joint_delta = std::max(
                        current_joint_delta,
                        std::abs(feedback.joint_positions[index] - initial_joints[index]));
                }
                peak_joint_delta = std::max(peak_joint_delta, current_joint_delta);
                if (linear_displacement > maximum_linear_displacement)
                {
                    throw std::runtime_error("R10 TCP 总位移超过门限");
                }
                if (transverse_displacement > maximum_transverse_displacement)
                {
                    throw std::runtime_error("R10 TCP 横向漂移超过门限");
                }
                if (current_joint_delta > maximum_joint_displacement)
                {
                    throw std::runtime_error("R10 单关节位移超过门限");
                }
                if (expected_motion_direction_z * dz <
                    -maximum_wrong_direction_displacement)
                {
                    throw std::runtime_error("R10 TCP 沿预期接触方向的反方向移动");
                }

                const double signed_force_delta =
                    feedback.wrench[2] - baseline_mean[2];
                const auto force_projection = massage_jaka::project_normal_force(
                    signed_force_delta, expected_measured_force_sign_z,
                    contact_threshold);
                if (!force_projection.valid)
                {
                    throw std::runtime_error(
                        "R10 世界 Z 力方向检查失败: " +
                        force_projection.message);
                }
                const double normal_force = force_projection.normal_force;
                peak_normal_force = std::max(peak_normal_force, normal_force);
                if (normal_force >= maximum_force)
                {
                    throw std::runtime_error("R10 法向力达到测试硬上限");
                }
                contact_consecutive = normal_force >= contact_threshold ?
                    contact_consecutive + 1U : 0U;
                if (!contact_detected && contact_consecutive >=
                    static_cast<std::size_t>(contact_confirmation_samples))
                {
                    contact_detected = true;
                    RCLCPP_INFO(
                        node->get_logger(),
                        "R10 CONTACT CONFIRMED: elapsed=%.3f s, force=%.4f N, "
                        "tcp_dz=%.6f m",
                        std::chrono::duration<double>(
                            SteadyClock::now() - active_started).count(),
                        normal_force, dz);
                }
                const bool target_in_tolerance =
                    std::abs(normal_force - target_force) <= target_tolerance;
                target_consecutive = contact_detected && target_in_tolerance ?
                    target_consecutive + 1U : 0U;
                if (!target_reached && target_consecutive >=
                    static_cast<std::size_t>(target_confirmation_samples))
                {
                    target_reached = true;
                    hold_started = SteadyClock::now();
                    RCLCPP_INFO(
                        node->get_logger(),
                        "R10 TARGET FORCE REACHED: elapsed=%.3f s, force=%.4f N",
                        std::chrono::duration<double>(
                            hold_started - active_started).count(), normal_force);
                }
                const auto now = SteadyClock::now();
                const char * phase = target_reached ? "hold" :
                    (contact_detected ? "ramp" : "search");
                if (target_reached)
                {
                    hold_samples.push_back({
                        std::chrono::duration<double>(now - hold_started).count(),
                        normal_force});
                }
                csv << phase << ','
                    << std::chrono::duration<double>(now - active_started).count()
                    << ',' << feedback.wrench_stamp_nanoseconds;
                for (double value : feedback.wrench) csv << ',' << value;
                csv << ',' << normal_force
                    << ',' << tool.transform.translation.x
                    << ',' << tool.transform.translation.y
                    << ',' << tool.transform.translation.z
                    << ',' << dx << ',' << dy << ',' << dz
                    << ',' << peak_joint_delta
                    << ',' << state.state.motion_state
                    << ',' << state.state.power_state
                    << ',' << state.state.servo_state
                    << ',' << state.state.collision_state
                    << ',' << static_cast<std::int32_t>(feedback.status) << '\n';
                csv.flush();

                if (!target_reached && now >= contact_deadline)
                {
                    throw std::runtime_error(
                        contact_detected ?
                        "R10 已接触但未在时限内进入目标力容差带" :
                        "R10 在时限内未检测到软测试块");
                }
                if (target_reached &&
                    now - hold_started >= std::chrono::duration<double>(hold_duration))
                {
                    break;
                }
            }
            if (interruption_requested.load())
            {
                throw std::runtime_error("R10 接触保持测试被中断");
            }

            const auto stop_result = controller->stop();
            if (!stop_result.success)
            {
                throw std::runtime_error(
                    "R10 关闭导纳或恢复控制权失败: " + stop_result.message);
            }
            massage_jaka::ForceHoldAcceptance acceptance;
            acceptance.target_force = target_force;
            acceptance.tolerance = target_tolerance;
            acceptance.maximum_force = maximum_force;
            acceptance.minimum_in_tolerance_ratio =
                minimum_hold_in_tolerance_ratio;
            acceptance.maximum_standard_deviation =
                maximum_hold_standard_deviation;
            acceptance.minimum_duration = hold_duration * 0.9;
            acceptance.maximum_sample_gap = maximum_hold_sample_gap;
            acceptance.maximum_mean_absolute_error =
                maximum_hold_mean_absolute_error;
            acceptance.maximum_terminal_error = maximum_hold_terminal_error;
            acceptance.minimum_samples =
                static_cast<std::size_t>(minimum_hold_samples);
            const auto report = massage_jaka::analyze_force_hold(
                hold_samples, acceptance);
            if (!report.valid || !report.passed)
            {
                throw std::runtime_error(
                    "R10 恒力保持验收失败: " + report.message);
            }
            std::string restore_message;
            if (!restore_configuration ||
                !restore_configuration(restore_message))
            {
                throw std::runtime_error(
                    "R10 测试完成但原始导纳配置恢复失败: " + restore_message);
            }
            restore_required = false;
            RCLCPP_INFO(
                node->get_logger(), "R10 CLEANUP VERIFIED: %s",
                restore_message.c_str());
            RCLCPP_INFO(
                node->get_logger(),
                "R10 CONTACT FORCE HOLD: PASS: target=%.3f N, mean=%.4f N, "
                "std=%.4f N, mean_abs_error=%.4f N, terminal_error=%.4f N, "
                "peak=%.4f N, in_tolerance=%.2f%%, duration=%.3f s, "
                "max_sample_gap=%.3f s, "
                "tcp_linear=%.6f m, tcp_transverse=%.6f m, "
                "max_joint=%.6f rad, csv=%s",
                target_force, report.mean_force, report.standard_deviation,
                report.mean_absolute_error, report.terminal_absolute_error,
                peak_normal_force, report.in_tolerance_ratio * 100.0,
                report.duration, report.maximum_sample_gap,
                peak_linear_displacement, peak_transverse_displacement,
                peak_joint_delta, csv_path.c_str());
                exit_code = 0;
            }
            (void)robot_state_subscription;
            (void)tf_listener;
        }
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(node->get_logger(), "R10 CONTACT FORCE HOLD: FAIL: %s", exception.what());
        exit_code = interruption_requested.load() ? 130 : 4;
    }

    if (controller)
    {
        const auto cleanup = controller->stop();
        if (!cleanup.success)
        {
            RCLCPP_ERROR(
                node->get_logger(), "R10 退出时未确认导纳关闭: %s",
                cleanup.message.c_str());
            exit_code = 5;
        }
        if (!controller->reset())
        {
            RCLCPP_ERROR(node->get_logger(), "R10 柔顺适配器复位失败");
            exit_code = 5;
        }
    }
    if (restore_required)
    {
        std::string restore_message;
        if (!restore_configuration ||
            !restore_configuration(restore_message))
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "R10 退出时未能恢复原始 JAKA 导纳配置: %s",
                restore_message.c_str());
            exit_code = 5;
        }
        else
        {
            restore_required = false;
            RCLCPP_INFO(
                node->get_logger(), "R10 退出清理完成: %s",
                restore_message.c_str());
        }
    }
    if (csv) csv.flush();
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return exit_code;
}
