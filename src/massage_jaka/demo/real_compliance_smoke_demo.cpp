#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "massage_jaka/jaka_compliance_controller.hpp"

namespace
{

constexpr std::size_t kForceAxisCount = 3;

bool finite_positive(double value)
{
    return std::isfinite(value) && value > 0.0;
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
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "real_compliance_smoke_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    int exit_code = 1;
    std::shared_ptr<massage_jaka::JakaComplianceController> controller;
    try
    {
        bool activate = false;
        bool parameters_confirmed = false;
        std::int64_t axis = 2;
        double target_wrench = 0.0;
        double maximum_force = 5.0;
        double maximum_torque = 1.0;
        double constant = 0.0;
        double rebound = 0.0;
        double run_duration = 2.0;
        double guard_timeout = 3.0;
        double readiness_timeout = 5.0;
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
        node->get_parameter_or("target_wrench", target_wrench, 0.0);
        node->get_parameter_or("maximum_force", maximum_force, 5.0);
        node->get_parameter_or("maximum_torque", maximum_torque, 1.0);
        node->get_parameter_or("constant", constant, 0.0);
        node->get_parameter_or("rebound", rebound, 0.0);
        node->get_parameter_or("run_duration", run_duration, 2.0);
        node->get_parameter_or("guard_timeout", guard_timeout, 3.0);
        node->get_parameter_or("readiness_timeout", readiness_timeout, 5.0);
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

        if (axis < 0 || axis >= 6 ||
            !std::isfinite(target_wrench) ||
            !finite_positive(maximum_force) ||
            !finite_positive(maximum_torque) ||
            !std::isfinite(constant) ||
            !std::isfinite(rebound) ||
            !finite_positive(run_duration) ||
            !finite_positive(guard_timeout) ||
            guard_timeout <= run_duration ||
            !finite_positive(readiness_timeout) ||
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

        massage_jaka::JakaComplianceConfig config;
        config.wrench_frame = wrench_frame;
        config.base_frame = base_frame;
        config.tool_frame = tool_frame;
        config.feedback_timeout = feedback_timeout;
        config.state_timeout = state_timeout;
        config.constant[static_cast<std::size_t>(axis)] = constant;
        config.rebound[static_cast<std::size_t>(axis)] = rebound;
        controller = std::make_shared<massage_jaka::JakaComplianceController>(
            node, config);

        tf2_ros::Buffer tf_buffer(node->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);
        const auto readiness_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(readiness_timeout);
        massage_motion::ComplianceFeedback current_feedback;
        bool transform_ready = false;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < readiness_deadline)
        {
            current_feedback = controller->feedback();
            transform_ready = tf_buffer._frameExists(base_frame) &&
                tf_buffer._frameExists(tool_frame) &&
                tf_buffer.canTransform(
                    base_frame, tool_frame, tf2::TimePointZero);
            if (!current_feedback.stale && transform_ready)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (current_feedback.stale || !transform_ready)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "真机柔顺就绪检查失败: feedback_stale=%s, tf_ready=%s",
                current_feedback.stale ? "true" : "false",
                transform_ready ? "true" : "false");
            exit_code = 2;
        }
        else
        {
            RCLCPP_INFO(
                node->get_logger(),
                "关节状态、FT 反馈和 %s -> %s TF 均已就绪",
                base_frame.c_str(), tool_frame.c_str());
            log_feedback(node->get_logger(), current_feedback);
            RCLCPP_INFO(
                node->get_logger(),
                "柔顺 axis 参数使用 FT 坐标系 %s 的 X/Y/Z/RX/RY/RZ 顺序",
                wrench_frame.c_str());
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
                    node->get_logger(),
                    "JAKA 柔顺启动失败: %s",
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
                while (rclcpp::ok() &&
                    std::chrono::steady_clock::now() < stop_at &&
                    controller->status() ==
                    massage_motion::ComplianceStatus::kActive)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }

                const auto stop_result = controller->stop();
                log_feedback(node->get_logger(), controller->feedback());
                if (!stop_result.success)
                {
                    RCLCPP_ERROR(
                        node->get_logger(),
                        "JAKA 柔顺冒烟结束异常: %s",
                        stop_result.message.c_str());
                    exit_code = 4;
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
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(node->get_logger(), "真机柔顺冒烟异常: %s", exception.what());
        exit_code = 2;
    }

    if (controller)
    {
        if (controller->status() == massage_motion::ComplianceStatus::kActive)
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

    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
