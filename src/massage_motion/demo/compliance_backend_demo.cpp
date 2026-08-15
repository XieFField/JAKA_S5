#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "massage_motion/ros2_control_compliance_controller.hpp"

namespace
{

double maximum_joint_delta(
    const std::vector<double> & before,
    const std::vector<double> & after)
{
    if (before.size() != after.size())
    {
        return std::numeric_limits<double>::infinity();
    }

    double maximum = 0.0;
    for (std::size_t index = 0; index < before.size(); ++index)
    {
        maximum = std::max(maximum, std::abs(after[index] - before[index]));
    }
    return maximum;
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "compliance_backend_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    std::string test_mode{"normal"};
    node->get_parameter_or("test_mode", test_mode, std::string{"normal"});
    if (test_mode != "normal" &&
        test_mode != "timeout" &&
        test_mode != "force_limit")
    {
        RCLCPP_ERROR(node->get_logger(), "不支持的 test_mode: %s", test_mode.c_str());
        rclcpp::shutdown();
        return 1;
    }

    double run_duration = 1.0;
    double request_timeout = 3.0;
    double force_limit = 5.0;
    node->get_parameter_or("run_duration", run_duration, 1.0);
    node->get_parameter_or("request_timeout", request_timeout, 3.0);
    node->get_parameter_or("force_limit", force_limit, 5.0);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() {executor.spin();});

    int exit_code = 1;
    {
        massage_motion::Ros2ControlComplianceConfig config;
        config.joint_names = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"};

        massage_motion::Ros2ControlComplianceController controller(node, config);

        // 订阅建立后由 start() 等待第一批反馈。
        massage_motion::ComplianceRequest request;
        request.request_id = "backend_" + test_mode;
        request.enabled_axes[2] = true;
        request.max_absolute_wrench[2] = force_limit;
        request.max_joint_displacement = 0.02;
        request.max_linear_displacement = 0.01;
        request.timeout = request_timeout;

        const auto before = [&controller]() {
            for (int attempt = 0; attempt < 100; ++attempt)
            {
                const auto positions = controller.current_joint_positions();
                if (!positions.empty())
                {
                    return positions;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            return std::vector<double>{};
        }();

        const auto start_result = controller.start(request);
        if (!start_result.success)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "柔顺后端启动失败: %s",
                start_result.message.c_str());
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "%s", start_result.message.c_str());

            if (test_mode == "normal")
            {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(run_duration));
                const auto result = controller.stop();
                const auto after = controller.current_joint_positions();
                const double drift = maximum_joint_delta(before, after);

                RCLCPP_INFO(
                    node->get_logger(),
                    "正常停止结果: %s，最大关节漂移=%.9e rad",
                    result.message.c_str(),
                    drift);
                exit_code = result.success && drift <= 0.001 ? 0 : 1;
            }
            else
            {
                const auto wait_deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(request_timeout + 3.0);
                while (
                    rclcpp::ok() &&
                    controller.status() == massage_motion::ComplianceStatus::kActive &&
                    std::chrono::steady_clock::now() < wait_deadline)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }

                const auto result = controller.stop();
                RCLCPP_INFO(
                    node->get_logger(),
                    "保护停止结果: error=%d, %s",
                    static_cast<int>(result.error),
                    result.message.c_str());

                const auto expected_error = test_mode == "timeout" ?
                    massage_motion::ComplianceError::kTimeout :
                    massage_motion::ComplianceError::kLimitExceeded;
                exit_code =
                    result.status == massage_motion::ComplianceStatus::kStopped &&
                    result.error == expected_error ? 0 : 1;
            }
        }
    }

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return exit_code;
}
