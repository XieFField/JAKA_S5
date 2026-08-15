#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "massage_motion/execution_types.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/ros2_control_compliance_controller.hpp"

namespace
{

using massage_motion::MotionRequest;
using massage_motion::MotionType;
using massage_motion::PoseTarget;

PoseTarget make_work_pose(double y_position)
{
    PoseTarget target;
    target.pose.header.frame_id = "world";
    // 给机械臂保留横向接近时的 IK 裕量，避免在最大伸展边界规划。
    target.pose.pose.position.x = 0.75;
    target.pose.pose.position.y = y_position;
    target.pose.pose.position.z = 0.106382015840;
    target.pose.pose.orientation.x = 0.707108079859;
    target.pose.pose.orientation.y = -0.000000008619;
    target.pose.pose.orientation.z = 0.000000008627;
    target.pose.pose.orientation.w = 0.707105482511;
    return target;
}

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
    return static_cast<double>(duration.sec) +
        static_cast<double>(duration.nanosec) * 1e-9;
}

std::vector<double> reorder_positions(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const trajectory_msgs::msg::JointTrajectoryPoint & point,
    const std::vector<std::string> & expected_names)
{
    if (trajectory.joint_names.size() != point.positions.size())
    {
        return {};
    }

    std::vector<double> ordered;
    ordered.reserve(expected_names.size());
    for (const auto & expected_name : expected_names)
    {
        const auto iterator = std::find(
            trajectory.joint_names.begin(),
            trajectory.joint_names.end(),
            expected_name);
        if (iterator == trajectory.joint_names.end())
        {
            return {};
        }
        ordered.push_back(point.positions[static_cast<std::size_t>(
            std::distance(trajectory.joint_names.begin(), iterator))]);
    }
    return ordered;
}

massage_motion::ExecutionResult execute_plan(
    massage_motion::MoveItTrajectoryExecutor & executor,
    const massage_motion::PlanResult & plan,
    const std::string & request_id,
    double timeout)
{
    massage_motion::ExecutionRequest request;
    request.request_id = request_id;
    request.robot_trajectory = plan.trajectory;
    request.timeout = timeout;
    return executor.execute(request);
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "compliance_contact_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    double force_limit = 5.0;
    bool expect_limit_exceeded = false;
    node->get_parameter_or("force_limit", force_limit, 5.0);
    node->get_parameter_or(
        "expect_limit_exceeded",
        expect_limit_exceeded,
        false);

    rclcpp::executors::MultiThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    std::thread ros_thread([&ros_executor]() {ros_executor.spin();});

    std::atomic<double> latest_force_z{0.0};
    std::atomic<double> maximum_force_z{0.0};
    auto wrench_subscription =
        node->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/massage_ft_broadcaster/wrench",
            rclcpp::SensorDataQoS(),
            [&latest_force_z, &maximum_force_z](
                const geometry_msgs::msg::WrenchStamped::SharedPtr message)
            {
                const double absolute_force = std::abs(message->wrench.force.z);
                latest_force_z.store(message->wrench.force.z);

                double observed = maximum_force_z.load();
                while (
                    absolute_force > observed &&
                    !maximum_force_z.compare_exchange_weak(observed, absolute_force))
                {
                }
            });

    int exit_code = 1;
    try
    {
        massage_motion::PlannerConfig planner_config;
        planner_config.planning_group = "jaka_s5";
        planner_config.end_effector_link = "Link_06";
        planner_config.reference_frame = "world";
        planner_config.planning_pipeline = "pilz_industrial_motion_planner";

        massage_motion::MotionPlanningSdk planner(node, planner_config);
        massage_motion::MoveItTrajectoryExecutor trajectory_executor(node);

        MotionRequest precontact_request;
        precontact_request.request_id = "contact_preparation";
        precontact_request.motion_type = MotionType::kPtp;
        // 端面距离测试块约 5 mm，同时避开 joint_5≈0 的腕部奇异构型。
        precontact_request.target = make_work_pose(-0.015);
        precontact_request.velocity_scale = 0.15;
        precontact_request.acceleration_scale = 0.15;

        const auto precontact_plan = planner.plan(precontact_request);
        if (!precontact_plan.success)
        {
            throw std::runtime_error(
                "预备位规划失败: " + precontact_plan.message);
        }

        const auto precontact_execution = execute_plan(
            trajectory_executor,
            precontact_plan,
            "contact_preparation_execution",
            12.0);
        if (!precontact_execution.success)
        {
            throw std::runtime_error(
                "预备位执行失败: " + precontact_execution.message);
        }
        RCLCPP_INFO(node->get_logger(), "已到达无接触预备位");

        MotionRequest contact_request;
        contact_request.request_id = "compliant_contact_reference";
        contact_request.motion_type = MotionType::kLin;
        // 正常模式最多压入约 0.5 mm；保护实验使用 2 mm 受限压入，
        // 让接触载荷越过高于自由空间惯性峰值的测试阈值。
        const double contact_y = expect_limit_exceeded ? -0.022 : -0.0205;
        contact_request.target = make_work_pose(contact_y);
        contact_request.velocity_scale = 0.005;
        contact_request.acceleration_scale = 0.005;
        contact_request.planning_timeout = 5.0;

        const auto contact_plan = planner.plan(contact_request);
        if (!contact_plan.success)
        {
            throw std::runtime_error(
                "接触参考轨迹规划失败: " + contact_plan.message);
        }

        const auto & reference_trajectory =
            contact_plan.trajectory.joint_trajectory;
        if (reference_trajectory.points.empty())
        {
            throw std::runtime_error("接触参考轨迹为空");
        }

        const std::vector<std::string> joint_names = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"};

        massage_motion::Ros2ControlComplianceConfig compliance_config;
        compliance_config.joint_names = joint_names;
        massage_motion::Ros2ControlComplianceController compliance_controller(
            node,
            compliance_config);

        massage_motion::ComplianceRequest compliance_request;
        compliance_request.request_id = "bounded_contact";
        compliance_request.enabled_axes[2] = true;
        compliance_request.max_absolute_wrench[2] = force_limit;
        compliance_request.max_joint_displacement = 0.20;
        compliance_request.max_linear_displacement = 0.03;
        const double reference_duration = duration_seconds(
            reference_trajectory.points.back().time_from_start);
        compliance_request.timeout = reference_duration + 5.0;

        RCLCPP_INFO(
            node->get_logger(),
            "接触参考轨迹: 时长=%.3f s, 点数=%zu",
            reference_duration,
            reference_trajectory.points.size());
        RCLCPP_INFO(
            node->get_logger(),
            "验收模式: force_limit=%.3f N, expect_limit_exceeded=%s",
            force_limit,
            expect_limit_exceeded ? "true" : "false");

        const auto compliance_start = compliance_controller.start(compliance_request);
        if (!compliance_start.success)
        {
            throw std::runtime_error(
                "导纳后端启动失败: " + compliance_start.message);
        }

        maximum_force_z.store(0.0);
        const auto reference_started_at = std::chrono::steady_clock::now();
        bool contact_detected = false;
        for (const auto & point : reference_trajectory.points)
        {
            const auto publish_time = reference_started_at +
                std::chrono::duration<double>(duration_seconds(point.time_from_start));
            std::this_thread::sleep_until(publish_time);

            if (compliance_controller.status() !=
                massage_motion::ComplianceStatus::kActive)
            {
                break;
            }

            // 一旦建立接触便停止继续压入，保持上一条参考并交给导纳响应。
            if (!expect_limit_exceeded &&
                std::abs(latest_force_z.load()) >= 0.1)
            {
                contact_detected = true;
                RCLCPP_INFO(
                    node->get_logger(),
                    "检测到接触，停止参考进给: Fz=%.6f N",
                    latest_force_z.load());
                break;
            }

            const auto ordered = reorder_positions(
                reference_trajectory,
                point,
                joint_names);
            if (ordered.empty() ||
                !compliance_controller.publish_joint_reference(ordered))
            {
                compliance_controller.stop();
                throw std::runtime_error("发布接触参考轨迹失败");
            }
        }

        // 最后一个参考点发布后，给传感器一个短窗口确认接触。
        const auto contact_deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(1);
        while (
            !contact_detected &&
            compliance_controller.status() ==
                massage_motion::ComplianceStatus::kActive &&
            std::chrono::steady_clock::now() < contact_deadline)
        {
            if (std::abs(latest_force_z.load()) >= 0.1)
            {
                contact_detected = true;
                RCLCPP_INFO(
                    node->get_logger(),
                    "检测到接触: Fz=%.6f N",
                    latest_force_z.load());
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (compliance_controller.status() ==
            massage_motion::ComplianceStatus::kActive)
        {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        const auto compliance_result = compliance_controller.stop();
        const double observed_force = maximum_force_z.load();

        RCLCPP_INFO(
            node->get_logger(),
            "接触阶段结束: max|Fz|=%.6f N, final Fz=%.6f N, result=%s",
            observed_force,
            latest_force_z.load(),
            compliance_result.message.c_str());

        if (expect_limit_exceeded)
        {
            if (compliance_result.error !=
                massage_motion::ComplianceError::kLimitExceeded)
            {
                throw std::runtime_error(
                    "预期触发力上限保护，但实际结果为: " +
                    compliance_result.message);
            }
            if (observed_force >= 5.0)
            {
                throw std::runtime_error("保护触发前的接触力峰值超过总体验收上限");
            }
            if (!compliance_controller.reset())
            {
                throw std::runtime_error("力上限保护触发后复位失败");
            }
            RCLCPP_INFO(
                node->get_logger(),
                "真实接触下的力上限保护、自动回切和复位通过");
        }
        else
        {
            if (!compliance_result.success)
            {
                throw std::runtime_error(
                    "接触阶段触发保护: " + compliance_result.message);
            }
            if (!contact_detected)
            {
                throw std::runtime_error("接近轨迹完成，但未检测到真实接触");
            }
            if (observed_force < 0.02 || observed_force >= 5.0)
            {
                throw std::runtime_error("未获得处于安全范围内的真实接触力");
            }
        }

        // 回切轨迹控制器后重新规划退出，避免复用接触前的起点状态。
        const auto retreat_plan = planner.plan(precontact_request);
        if (!retreat_plan.success)
        {
            throw std::runtime_error("退出规划失败: " + retreat_plan.message);
        }
        const auto retreat_execution = execute_plan(
            trajectory_executor,
            retreat_plan,
            "contact_retreat_execution",
            12.0);
        if (!retreat_execution.success)
        {
            throw std::runtime_error("退出执行失败: " + retreat_execution.message);
        }

        RCLCPP_INFO(node->get_logger(), "受控接触、柔顺响应和退出全部通过");
        exit_code = 0;
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
    }

    // 保证订阅对象持续到测试结束。
    (void)wrench_subscription;
    ros_executor.cancel();
    ros_thread.join();
    rclcpp::shutdown();
    return exit_code;
}
