#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/relative_joint_target.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"

namespace
{

const std::vector<std::string> kJointNames{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};

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
    double execution_timeout = 20.0;
    double joint_state_timeout = 3.0;
    double endpoint_tolerance = 0.01;
    node->get_parameter_or("execute", execute, false);
    node->get_parameter_or("joint_name", joint_name, std::string{"joint_1"});
    node->get_parameter_or("joint_delta", joint_delta, 0.01);
    node->get_parameter_or("maximum_joint_delta", maximum_joint_delta, 0.05);
    node->get_parameter_or("velocity_scale", velocity_scale, 0.02);
    node->get_parameter_or("acceleration_scale", acceleration_scale, 0.02);
    node->get_parameter_or("planning_timeout", planning_timeout, 5.0);
    node->get_parameter_or("execution_timeout", execution_timeout, 20.0);
    node->get_parameter_or("joint_state_timeout", joint_state_timeout, 3.0);
    node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.01);

    if (joint_name.empty() ||
        !std::isfinite(joint_delta) || joint_delta == 0.0 ||
        !std::isfinite(maximum_joint_delta) || maximum_joint_delta <= 0.0 ||
        std::abs(joint_delta) > maximum_joint_delta ||
        !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
        velocity_scale > 1.0 ||
        !std::isfinite(acceleration_scale) || acceleration_scale <= 0.0 ||
        acceleration_scale > 1.0 ||
        !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
        !std::isfinite(execution_timeout) || execution_timeout <= 0.0 ||
        !std::isfinite(joint_state_timeout) || joint_state_timeout <= 0.0 ||
        !std::isfinite(endpoint_tolerance) || endpoint_tolerance <= 0.0)
    {
        RCLCPP_ERROR(node->get_logger(), "真机运动冒烟参数无效");
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
                    auto trajectory_executor = std::make_shared<
                        massage_motion::MoveItTrajectoryExecutor>(node);
                    massage_motion::ExecutionRequest execution_request;
                    execution_request.request_id = "real_motion_smoke_execution";
                    execution_request.robot_trajectory = plan.trajectory;
                    execution_request.timeout = execution_timeout;
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
                            if (!endpoint.valid ||
                                endpoint.max_absolute_error > endpoint_tolerance)
                            {
                                RCLCPP_ERROR(
                                    node->get_logger(),
                                    "终点校验失败: valid=%s, max_error=%.9f rad",
                                    endpoint.valid ? "true" : "false",
                                    endpoint.max_absolute_error);
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
