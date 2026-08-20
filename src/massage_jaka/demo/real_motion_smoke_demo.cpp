#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/execution_timing.hpp"
#include "massage_motion/relative_joint_target.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"
#include "massage_motion/trajectory_execution_contract.hpp"

namespace
{

const std::vector<std::string> kJointNames{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};
constexpr double kRadiansToDegrees = 57.29577951308232;

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "real_motion_smoke_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    bool execute = false;
    std::string joint_name{"joint_1"};
    double joint_delta = 0.01;
    double maximum_joint_delta = 0.05;
    double velocity_scale = 0.02;
    double acceleration_scale = 0.02;
    double planning_timeout = 5.0;
    double execution_timeout_margin = 10.0;
    double joint_state_timeout = 3.0;
    double endpoint_tolerance = 0.002;
    node->get_parameter_or("execute", execute, false);
    node->get_parameter_or("joint_name", joint_name, std::string{"joint_1"});
    node->get_parameter_or("joint_delta", joint_delta, 0.01);
    node->get_parameter_or("maximum_joint_delta", maximum_joint_delta, 0.05);
    node->get_parameter_or("velocity_scale", velocity_scale, 0.02);
    node->get_parameter_or("acceleration_scale", acceleration_scale, 0.02);
    node->get_parameter_or("planning_timeout", planning_timeout, 5.0);
    node->get_parameter_or(
        "execution_timeout_margin", execution_timeout_margin, 10.0);
    node->get_parameter_or("joint_state_timeout", joint_state_timeout, 3.0);
    node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.002);

    if (joint_name.empty() ||
        !std::isfinite(joint_delta) || joint_delta == 0.0 ||
        !std::isfinite(maximum_joint_delta) || maximum_joint_delta <= 0.0 ||
        std::abs(joint_delta) > maximum_joint_delta ||
        !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
        velocity_scale > 1.0 ||
        !std::isfinite(acceleration_scale) || acceleration_scale <= 0.0 ||
        acceleration_scale > 1.0 ||
        !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
        !std::isfinite(execution_timeout_margin) || execution_timeout_margin < 0.0 ||
        !std::isfinite(joint_state_timeout) || joint_state_timeout <= 0.0 ||
        !std::isfinite(endpoint_tolerance) || endpoint_tolerance <= 0.0)
    {
        RCLCPP_ERROR(node->get_logger(), "真机运动冒烟参数无效");
        rclcpp::shutdown();
        return 2;
    }
    if (endpoint_tolerance >= std::abs(joint_delta))
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "endpoint_tolerance 必须严格小于 abs(joint_delta)，当前 "
            "tolerance=%.9f rad, delta=%.9f rad",
            endpoint_tolerance, joint_delta);
        rclcpp::shutdown();
        return 2;
    }

    std::mutex state_mutex;
    std::condition_variable state_condition;
    sensor_msgs::msg::JointState latest_state;
    std::uint64_t state_sequence = 0;
    auto state_subscription =
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [&](sensor_msgs::msg::JointState::SharedPtr message)
            {
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    latest_state = *message;
                    ++state_sequence;
                }
                state_condition.notify_all();
            });

    rclcpp::executors::SingleThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    std::thread spin_thread([&ros_executor]() {ros_executor.spin();});

    int exit_code = 1;
    try
    {
        {
            std::unique_lock<std::mutex> lock(state_mutex);
            if (!state_condition.wait_for(
                    lock,
                    std::chrono::duration<double>(joint_state_timeout),
                    [&state_sequence]() {return state_sequence > 0;}))
            {
                RCLCPP_ERROR(node->get_logger(), "等待 /joint_states 超时");
                exit_code = 2;
            }
        }

        if (exit_code != 2)
        {
            sensor_msgs::msg::JointState planning_state;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                planning_state = latest_state;
            }
            const auto target_result = massage_motion::make_relative_joint_target(
                planning_state,
                kJointNames,
                joint_name,
                joint_delta,
                maximum_joint_delta);
            if (!target_result.validation.valid)
            {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "构造相对关节目标失败: %s",
                    target_result.validation.message.c_str());
                exit_code = 2;
            }
            else
            {
                const auto selected_joint_iterator = std::find(
                    kJointNames.begin(), kJointNames.end(), joint_name);
                const auto selected_joint_index = static_cast<std::size_t>(
                    std::distance(kJointNames.begin(), selected_joint_iterator));
                const double target_position =
                    target_result.target.positions[selected_joint_index];
                const double initial_position = target_position - joint_delta;

                RCLCPP_INFO(
                    node->get_logger(),
                    "相对关节目标: joint=%s, initial=%.9f rad, "
                    "target=%.9f rad, commanded_delta=%.9f rad (%.6f deg)",
                    joint_name.c_str(), initial_position, target_position,
                    joint_delta, joint_delta * kRadiansToDegrees);

                if (execute)
                {
                    auto parameter_node = std::make_shared<rclcpp::Node>(
                        "motion_smoke_driver_parameter_client");
                    auto parameter_client =
                        std::make_shared<rclcpp::SyncParametersClient>(
                        parameter_node, "/jaka_driver");
                    if (!parameter_client->wait_for_service(
                            std::chrono::duration<double>(joint_state_timeout)))
                    {
                        throw std::runtime_error(
                            "等待 /jaka_driver 参数服务超时");
                    }
                    const auto driver_parameters =
                        parameter_client->get_parameters({
                            "trajectory_goal_tolerance",
                            "trajectory_goal_timeout"});
                    if (driver_parameters.size() != 2U)
                    {
                        throw std::runtime_error("驱动终点参数读取不完整");
                    }
                    const double driver_goal_tolerance =
                        driver_parameters[0].as_double();
                    const double driver_goal_timeout =
                        driver_parameters[1].as_double();
                    const auto execution_contract =
                        massage_motion::validate_trajectory_execution_contract(
                        driver_goal_tolerance, driver_goal_timeout,
                        endpoint_tolerance, execution_timeout_margin);
                    if (!execution_contract.valid)
                    {
                        std::ostringstream message;
                        message << "驱动终点参数与真机冒烟执行门槛不一致: "
                                << execution_contract.message
                                << "; tolerance=" << driver_goal_tolerance
                                << " rad, endpoint_tolerance="
                                << endpoint_tolerance << " rad, driver_margin="
                                << driver_goal_timeout
                                << " s, execution_margin="
                                << execution_timeout_margin << " s";
                        throw std::runtime_error(message.str());
                    }
                    RCLCPP_INFO(
                        node->get_logger(),
                        "驱动终点参数读回通过: tolerance=%.9f rad, "
                        "driver_margin=%.3f s, execution_margin=%.3f s",
                        driver_goal_tolerance, driver_goal_timeout,
                        execution_timeout_margin);
                }

                massage_motion::PlannerConfig planner_config;
                planner_config.planning_group = "jaka_s5";
                planner_config.end_effector_link = "massage_tool_tip";
                planner_config.reference_frame = "world";
                planner_config.planning_pipeline =
                    "pilz_industrial_motion_planner";
                auto planner = std::make_shared<massage_motion::MotionPlanningSdk>(
                    node, planner_config);

                massage_motion::MotionRequest request;
                request.request_id = "real_motion_smoke_ptp";
                request.motion_type = massage_motion::MotionType::kPtp;
                request.target = target_result.target;
                request.velocity_scale = velocity_scale;
                request.acceleration_scale = acceleration_scale;
                request.planning_timeout = planning_timeout;

                const auto plan = planner->plan(request);
                if (!plan.success)
                {
                    RCLCPP_ERROR(
                        node->get_logger(),
                        "真机冒烟轨迹规划失败: %s",
                        plan.message.c_str());
                    exit_code = 3;
                }
                else if (!execute)
                {
                    RCLCPP_INFO(
                        node->get_logger(),
                        "只规划验证通过: joint=%s, delta=%.6f rad, points=%zu",
                        joint_name.c_str(),
                        joint_delta,
                        plan.trajectory.joint_trajectory.points.size());
                    exit_code = 0;
                }
                else
                {
                    massage_motion::ExecutionTimingPolicy timing_policy;
                    timing_policy.margin = execution_timeout_margin;
                    const auto timing = massage_motion::calculate_execution_timing(
                        plan.trajectory, timing_policy);
                    if (!timing.valid)
                    {
                        RCLCPP_ERROR(
                            node->get_logger(),
                            "无法确定真机冒烟轨迹执行超时: %s",
                            timing.message.c_str());
                        exit_code = 4;
                    }
                    else
                    {
                        RCLCPP_INFO(node->get_logger(), "%s", timing.message.c_str());
                        auto trajectory_executor = std::make_shared<
                            massage_motion::MoveItTrajectoryExecutor>(node);
                        massage_motion::ExecutionRequest execution_request;
                        execution_request.request_id = "real_motion_smoke_execution";
                        execution_request.robot_trajectory = plan.trajectory;
                        execution_request.timeout = timing.timeout;
                        const auto execution =
                            trajectory_executor->execute(execution_request);
                        if (!execution.success)
                        {
                            RCLCPP_ERROR(
                                node->get_logger(),
                                "真机冒烟轨迹执行失败: %s",
                                execution.message.c_str());
                            exit_code = 4;
                        }
                        else
                        {
                            sensor_msgs::msg::JointState final_state;
                            {
                                std::unique_lock<std::mutex> lock(state_mutex);
                                const auto sequence_after_execution = state_sequence;
                                if (!state_condition.wait_for(
                                        lock,
                                        std::chrono::duration<double>(joint_state_timeout),
                                        [&state_sequence, sequence_after_execution]()
                                        {
                                            return state_sequence > sequence_after_execution;
                                        }))
                                {
                                    RCLCPP_ERROR(
                                        node->get_logger(),
                                        "等待执行后的 /joint_states 更新超时");
                                    exit_code = 5;
                                }
                                else
                                {
                                    final_state = latest_state;
                                }
                            }
                            if (exit_code != 5)
                            {
                                const auto endpoint =
                                    massage_motion::calculate_trajectory_endpoint_error(
                                        plan.trajectory, final_state);
                                const auto selected_error = std::find_if(
                                    endpoint.joint_errors.begin(),
                                    endpoint.joint_errors.end(),
                                    [&joint_name](const auto & joint_error)
                                    {
                                        return joint_error.joint_name == joint_name;
                                    });
                                const bool selected_joint_valid =
                                    selected_error != endpoint.joint_errors.end();
                                double achieved_delta = 0.0;
                                double completion_ratio = 0.0;
                                if (selected_joint_valid)
                                {
                                    achieved_delta =
                                        selected_error->actual_position - initial_position;
                                    completion_ratio = achieved_delta / joint_delta;
                                    RCLCPP_INFO(
                                        node->get_logger(),
                                        "%s 终点数据: initial=%.9f rad, "
                                        "target=%.9f rad, actual=%.9f rad, "
                                        "commanded_delta=%.9f rad (%.6f deg), "
                                        "achieved_delta=%.9f rad (%.6f deg), "
                                        "completion=%.2f%%, target_error=%.9f rad, "
                                        "tolerance=%.9f rad",
                                        joint_name.c_str(), initial_position,
                                        selected_error->target_position,
                                        selected_error->actual_position,
                                        joint_delta,
                                        joint_delta * kRadiansToDegrees,
                                        achieved_delta,
                                        achieved_delta * kRadiansToDegrees,
                                        completion_ratio * 100.0,
                                        selected_error->absolute_error,
                                        endpoint_tolerance);
                                }

                                if (!endpoint.valid || !selected_joint_valid ||
                                    endpoint.max_absolute_error > endpoint_tolerance)
                                {
                                    RCLCPP_ERROR(
                                        node->get_logger(),
                                        "终点校验失败: valid=%s, selected_joint=%s, "
                                        "max_error=%.9f rad, tolerance=%.9f rad, "
                                        "message=%s",
                                        endpoint.valid ? "true" : "false",
                                        selected_joint_valid ? "found" : "missing",
                                        endpoint.max_absolute_error,
                                        endpoint_tolerance,
                                        endpoint.message.c_str());
                                    exit_code = 5;
                                }
                                else
                                {
                                    RCLCPP_INFO(
                                        node->get_logger(),
                                        "真机运动冒烟验证完成，最大终点误差 %.9f rad",
                                        endpoint.max_absolute_error);
                                    exit_code = 0;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(node->get_logger(), "真机运动冒烟异常: %s", exception.what());
        exit_code = 6;
    }

    (void)state_subscription;
    ros_executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
