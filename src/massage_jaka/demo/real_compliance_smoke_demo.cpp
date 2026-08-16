#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "jaka_msgs/msg/robot_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "massage_jaka/jaka_compliance_controller.hpp"
#include "massage_motion/wrench_observation.hpp"

namespace
{

constexpr std::size_t kForceAxisCount = 3;
std::atomic<bool> interruption_requested{false};

void handle_signal(int)
{
    interruption_requested.store(true);
}

bool finite_positive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

std::string default_csv_path()
{
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return "/tmp/massage_compliance_" + std::to_string(milliseconds) + ".csv";
}

void log_feedback(
    const rclcpp::Logger & logger,
    const massage_motion::ComplianceFeedback & feedback)
{
    RCLCPP_INFO(
        logger,
        "FT=[%.4f, %.4f, %.4f N; %.4f, %.4f, %.4f N*m], age=%.4f s",
        feedback.wrench[0], feedback.wrench[1], feedback.wrench[2],
        feedback.wrench[3], feedback.wrench[4], feedback.wrench[5],
        feedback.age);
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
        "real_compliance_smoke_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    int exit_code = 1;
    std::shared_ptr<massage_jaka::JakaComplianceController> controller;
    std::ofstream csv;
    std::string csv_path;
    try
    {
        bool activate = false;
        bool parameters_confirmed = false;
        std::int64_t axis = 2;
        std::int64_t force_control_frame = 0;
        double target_wrench = 0.0;
        double maximum_force = 5.0;
        double maximum_torque = 1.0;
        double maximum_speed_wrench = 0.0;
        double rebound_wrench = 0.0;
        double run_duration = 2.0;
        double guard_timeout = 3.0;
        double readiness_timeout = 5.0;
        double baseline_duration = 2.0;
        double feedback_timeout = 0.5;
        double state_timeout = 0.5;
        double maximum_joint_displacement = 0.02;
        double maximum_linear_displacement = 0.01;
        std::string wrench_frame{"Link_06"};
        std::string base_frame{"world"};
        std::string tool_frame{"massage_tool_tip"};
        node->get_parameter_or("activate", activate, false);
        node->get_parameter_or("parameters_confirmed", parameters_confirmed, false);
        node->get_parameter_or("axis", axis, std::int64_t{2});
        node->get_parameter_or(
            "force_control_frame", force_control_frame, std::int64_t{0});
        node->get_parameter_or("target_wrench", target_wrench, 0.0);
        node->get_parameter_or("maximum_force", maximum_force, 5.0);
        node->get_parameter_or("maximum_torque", maximum_torque, 1.0);
        node->get_parameter_or(
            "maximum_speed_wrench", maximum_speed_wrench, 0.0);
        node->get_parameter_or("rebound_wrench", rebound_wrench, 0.0);
        node->get_parameter_or("run_duration", run_duration, 2.0);
        node->get_parameter_or("guard_timeout", guard_timeout, 3.0);
        node->get_parameter_or("readiness_timeout", readiness_timeout, 5.0);
        node->get_parameter_or("baseline_duration", baseline_duration, 2.0);
        node->get_parameter_or("feedback_timeout", feedback_timeout, 0.5);
        node->get_parameter_or("state_timeout", state_timeout, 0.5);
        node->get_parameter_or(
            "maximum_joint_displacement", maximum_joint_displacement, 0.02);
        node->get_parameter_or(
            "maximum_linear_displacement", maximum_linear_displacement, 0.01);
        node->get_parameter_or(
            "wrench_frame", wrench_frame, std::string{"Link_06"});
        node->get_parameter_or("base_frame", base_frame, std::string{"world"});
        node->get_parameter_or(
            "tool_frame", tool_frame, std::string{"massage_tool_tip"});
        node->get_parameter_or("csv_path", csv_path, std::string{});

        if (axis < 0 || axis >= 6 ||
            (force_control_frame != 0 && force_control_frame != 1) ||
            !std::isfinite(target_wrench) ||
            !finite_positive(maximum_force) ||
            !finite_positive(maximum_torque) ||
            !std::isfinite(maximum_speed_wrench) ||
            maximum_speed_wrench < 0.0 ||
            !std::isfinite(rebound_wrench) || rebound_wrench < 0.0 ||
            !finite_positive(run_duration) ||
            !finite_positive(guard_timeout) || guard_timeout <= run_duration ||
            !finite_positive(readiness_timeout) ||
            !finite_positive(baseline_duration) ||
            !finite_positive(feedback_timeout) ||
            !finite_positive(state_timeout) ||
            !finite_positive(maximum_joint_displacement) ||
            !finite_positive(maximum_linear_displacement) ||
            wrench_frame.empty() || base_frame.empty() || tool_frame.empty())
        {
            throw std::invalid_argument(
                "柔顺冒烟参数无效；guard_timeout 必须大于 run_duration");
        }
        const double selected_limit =
            static_cast<std::size_t>(axis) < kForceAxisCount ?
            maximum_force : maximum_torque;
        if (std::abs(target_wrench) > selected_limit)
        {
            throw std::invalid_argument("目标力/力矩超过对应项目限幅");
        }
        if (activate && !parameters_confirmed)
        {
            throw std::invalid_argument(
                "activate=true 时必须显式设置 parameters_confirmed=true");
        }
        if (activate && maximum_speed_wrench <= 0.0)
        {
            throw std::invalid_argument(
                "activate=true 时必须显式设置正的 maximum_speed_wrench");
        }

        if (csv_path.empty())
        {
            csv_path = default_csv_path();
        }
        csv.open(csv_path, std::ios::out | std::ios::trunc);
        if (!csv)
        {
            throw std::runtime_error("无法创建柔顺遥测 CSV: " + csv_path);
        }
        csv << "phase,receive_time_s,wrench_stamp_ns,fx_n,fy_n,fz_n,"
               "tx_nm,ty_nm,tz_nm,joint_1,joint_2,joint_3,joint_4,"
               "joint_5,joint_6,tcp_x_m,tcp_y_m,tcp_z_m,motion_state,"
               "power_state,servo_state,collision_state,compliance_status\n";
        csv << std::setprecision(17);

        massage_jaka::JakaComplianceConfig config;
        config.wrench_frame = wrench_frame;
        config.base_frame = base_frame;
        config.tool_frame = tool_frame;
        config.feedback_timeout = feedback_timeout;
        config.state_timeout = state_timeout;
        config.force_control_frame =
            static_cast<std::int32_t>(force_control_frame);
        config.maximum_speed_wrench[static_cast<std::size_t>(axis)] =
            maximum_speed_wrench;
        config.rebound_wrench[static_cast<std::size_t>(axis)] = rebound_wrench;
        controller = std::make_shared<massage_jaka::JakaComplianceController>(
            node, config);

        std::mutex robot_state_mutex;
        jaka_msgs::msg::RobotMsg robot_state;
        std::chrono::steady_clock::time_point robot_state_time{};
        bool robot_state_received = false;
        auto robot_state_subscription =
            node->create_subscription<jaka_msgs::msg::RobotMsg>(
                config.robot_state_topic,
                rclcpp::SensorDataQoS(),
                [&](const jaka_msgs::msg::RobotMsg::SharedPtr message)
                {
                    std::lock_guard<std::mutex> lock(robot_state_mutex);
                    robot_state = *message;
                    robot_state_time = std::chrono::steady_clock::now();
                    robot_state_received = true;
                });

        tf2_ros::Buffer tf_buffer(node->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);
        const auto telemetry_started_at = std::chrono::steady_clock::now();
        std::int64_t last_recorded_wrench_stamp =
            std::numeric_limits<std::int64_t>::min();
        std::vector<massage_motion::WrenchObservationSample> baseline_samples;

        const auto record_sample = [&](const char * phase)
            {
                const auto feedback = controller->feedback();
                if (feedback.stale || feedback.wrench_stamp_nanoseconds ==
                    last_recorded_wrench_stamp ||
                    feedback.joint_positions.size() != 6U)
                {
                    return false;
                }
                jaka_msgs::msg::RobotMsg current_robot_state;
                {
                    std::lock_guard<std::mutex> lock(robot_state_mutex);
                    if (!robot_state_received)
                    {
                        return false;
                    }
                    current_robot_state = robot_state;
                }
                geometry_msgs::msg::TransformStamped transform;
                try
                {
                    transform = tf_buffer.lookupTransform(
                        base_frame, tool_frame, tf2::TimePointZero);
                }
                catch (const tf2::TransformException &)
                {
                    return false;
                }
                last_recorded_wrench_stamp = feedback.wrench_stamp_nanoseconds;
                const double receive_time = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() -
                    telemetry_started_at).count();
                csv << phase << ',' << receive_time << ','
                    << feedback.wrench_stamp_nanoseconds;
                for (const double value : feedback.wrench)
                {
                    csv << ',' << value;
                }
                for (const double value : feedback.joint_positions)
                {
                    csv << ',' << value;
                }
                csv << ',' << transform.transform.translation.x
                    << ',' << transform.transform.translation.y
                    << ',' << transform.transform.translation.z
                    << ',' << current_robot_state.motion_state
                    << ',' << current_robot_state.power_state
                    << ',' << current_robot_state.servo_state
                    << ',' << current_robot_state.collision_state
                    << ',' << static_cast<std::int32_t>(feedback.status)
                    << '\n';
                csv.flush();

                if (std::string(phase) == "baseline")
                {
                    massage_motion::WrenchObservationSample sample;
                    sample.phase =
                        massage_motion::WrenchObservationPhase::kBaseline;
                    sample.stamp_nanoseconds =
                        feedback.wrench_stamp_nanoseconds;
                    sample.receive_time = receive_time;
                    sample.frame_id = wrench_frame;
                    sample.values = feedback.wrench;
                    baseline_samples.push_back(std::move(sample));
                }
                return true;
            };

        const auto readiness_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(readiness_timeout);
        massage_motion::ComplianceFeedback current_feedback;
        bool transform_ready = false;
        bool robot_ready = false;
        while (!interruption_requested.load() &&
            std::chrono::steady_clock::now() < readiness_deadline)
        {
            current_feedback = controller->feedback();
            transform_ready = tf_buffer._frameExists(base_frame) &&
                tf_buffer._frameExists(tool_frame) &&
                tf_buffer.canTransform(
                    base_frame, tool_frame, tf2::TimePointZero);
            {
                std::lock_guard<std::mutex> lock(robot_state_mutex);
                const double age = robot_state_received ?
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() -
                        robot_state_time).count() :
                    std::numeric_limits<double>::infinity();
                robot_ready = robot_state_received && age <= state_timeout &&
                    robot_state.power_state == 1 &&
                    robot_state.servo_state == 1 &&
                    robot_state.collision_state == 0 &&
                    robot_state.motion_state == 0;
            }
            if (!current_feedback.stale && transform_ready && robot_ready)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (interruption_requested.load())
        {
            throw std::runtime_error("柔顺冒烟在就绪阶段被中断");
        }
        if (current_feedback.stale || !transform_ready || !robot_ready)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "真机柔顺就绪检查失败: feedback_stale=%s, tf_ready=%s, "
                "robot_ready=%s",
                current_feedback.stale ? "true" : "false",
                transform_ready ? "true" : "false",
                robot_ready ? "true" : "false");
            exit_code = 2;
        }
        else
        {
            RCLCPP_INFO(
                node->get_logger(),
                "机器人状态、关节状态、FT 反馈和 %s -> %s TF 均已就绪",
                base_frame.c_str(), tool_frame.c_str());
            log_feedback(node->get_logger(), current_feedback);
            RCLCPP_INFO(
                node->get_logger(),
                "开始采集 %.2f 秒启用前 FT 基线", baseline_duration);
            const auto baseline_deadline = std::chrono::steady_clock::now() +
                std::chrono::duration<double>(baseline_duration);
            while (!interruption_requested.load() &&
                std::chrono::steady_clock::now() < baseline_deadline)
            {
                record_sample("baseline");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (interruption_requested.load())
            {
                throw std::runtime_error("柔顺冒烟在基线阶段被中断");
            }
            const auto baseline_report =
                massage_motion::analyze_wrench_phase(
                    baseline_samples, wrench_frame, 0.0, feedback_timeout);
            if (!baseline_report.valid)
            {
                throw std::runtime_error(
                    "启用前 FT 基线无效: " + baseline_report.message);
            }
            const auto & baseline = baseline_report.statistics;
            RCLCPP_INFO(
                node->get_logger(),
                "FT 基线 samples=%zu, mean=[%.4f %.4f %.4f; %.4f %.4f %.4f], "
                "std=[%.4f %.4f %.4f; %.4f %.4f %.4f]",
                baseline.sample_count,
                baseline.mean[0], baseline.mean[1], baseline.mean[2],
                baseline.mean[3], baseline.mean[4], baseline.mean[5],
                baseline.standard_deviation[0],
                baseline.standard_deviation[1],
                baseline.standard_deviation[2],
                baseline.standard_deviation[3],
                baseline.standard_deviation[4],
                baseline.standard_deviation[5]);
            exit_code = 0;
        }

        if (exit_code == 0 && activate)
        {
            massage_motion::ComplianceRequest request;
            request.request_id = "real_compliance_smoke";
            request.enabled_axes[static_cast<std::size_t>(axis)] = true;
            request.target_wrench[static_cast<std::size_t>(axis)] = target_wrench;
            request.max_absolute_wrench = {
                maximum_force, maximum_force, maximum_force,
                maximum_torque, maximum_torque, maximum_torque};
            request.max_joint_displacement = maximum_joint_displacement;
            request.max_linear_displacement = maximum_linear_displacement;
            request.timeout = guard_timeout;

            const auto start_result = controller->start(request);
            if (!start_result.success)
            {
                RCLCPP_ERROR(
                    node->get_logger(), "JAKA 柔顺启动失败: %s",
                    start_result.message.c_str());
                exit_code = 3;
            }
            else
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "JAKA 柔顺独立冒烟已启动，axis=%ld, duration=%.3f s",
                    static_cast<long>(axis), run_duration);
                const auto stop_at = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(run_duration);
                while (!interruption_requested.load() &&
                    std::chrono::steady_clock::now() < stop_at &&
                    controller->status() ==
                    massage_motion::ComplianceStatus::kActive)
                {
                    record_sample("active");
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }

                const auto stop_result = controller->stop();
                record_sample("stopped");
                log_feedback(node->get_logger(), controller->feedback());
                if (!stop_result.success)
                {
                    RCLCPP_ERROR(
                        node->get_logger(), "JAKA 柔顺冒烟结束异常: %s",
                        stop_result.message.c_str());
                    exit_code = 4;
                }
                else if (interruption_requested.load())
                {
                    RCLCPP_WARN(
                        node->get_logger(),
                        "收到中断，已完成导纳关闭与控制权读回确认");
                    exit_code = 130;
                }
                else
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "JAKA 柔顺冒烟完成并已关闭导纳，峰值 "
                        "[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                        stop_result.peak_absolute_wrench[0],
                        stop_result.peak_absolute_wrench[1],
                        stop_result.peak_absolute_wrench[2],
                        stop_result.peak_absolute_wrench[3],
                        stop_result.peak_absolute_wrench[4],
                        stop_result.peak_absolute_wrench[5]);
                    exit_code = 0;
                }
            }
        }
        (void)robot_state_subscription;
        (void)tf_listener;
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(node->get_logger(), "真机柔顺冒烟异常: %s", exception.what());
        exit_code = interruption_requested.load() ? 130 : 2;
    }

    if (controller)
    {
        if (controller->status() != massage_motion::ComplianceStatus::kIdle &&
            controller->status() != massage_motion::ComplianceStatus::kStopped)
        {
            const auto cleanup = controller->stop();
            if (!cleanup.success)
            {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "退出清理未能确认关闭柔顺: %s",
                    cleanup.message.c_str());
                exit_code = 5;
            }
        }
        if (!controller->reset())
        {
            RCLCPP_ERROR(node->get_logger(), "柔顺适配器复位失败");
            exit_code = 5;
        }
    }
    if (csv)
    {
        csv.flush();
        RCLCPP_INFO(node->get_logger(), "柔顺遥测 CSV: %s", csv_path.c_str());
    }

    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
