#include <exception>
#include <memory>
#include <string>
#include <thread>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>

#include "sensor_msgs/msg/joint_state.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"

#include "rclcpp/rclcpp.hpp"

#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_task/minimal_task_state_machine.hpp"

using namespace massage_motion;
using namespace massage_task;

bool is_expected_task_result(
    const std::string & test_mode,
    const massage_task::TaskResult & result)
{
    if (test_mode == "normal")
    {
        return
            result.success &&
            result.final_state == massage_task::TaskState::kCompleted &&
            result.error == massage_task::TaskError::kNone &&
            result.execution_result.status ==
                massage_motion::ExecutionStatus::kSucceeded;
    }

    if (test_mode == "cancel")
    {
        return
            !result.success &&
            result.final_state == massage_task::TaskState::kCanceled &&
            result.error == massage_task::TaskError::kCanceled &&
            result.execution_result.status ==
                massage_motion::ExecutionStatus::kCanceled;
    }

    return
        !result.success &&
        result.final_state == massage_task::TaskState::kFault &&
        result.error == massage_task::TaskError::kTimeout &&
        result.execution_result.status ==
            massage_motion::ExecutionStatus::kTimedOut;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "minimal_task_demo",
        rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true)
    );

    std::string test_mode{"normal"};
    node->get_parameter_or(
        "test_mode",
        test_mode,
        std::string{"normal"}
    );

    if (test_mode != "normal" &&
        test_mode != "cancel" &&
        test_mode != "timeout")
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "未知 test_mode: %s，可选 normal/cancel/timeout",
            test_mode.c_str());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);

    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    int error_code = 0;

    std::mutex joint_state_mutex;
    std::condition_variable joint_state_condition;
    sensor_msgs::msg::JointState latest_joint_state;
    std::uint64_t joint_state_sequence = 0;

    auto joint_state_subscription =
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [&](const sensor_msgs::msg::JointState::SharedPtr message)
            {
                {
                    std::lock_guard<std::mutex> lock(joint_state_mutex);
                    latest_joint_state = *message;
                    ++joint_state_sequence;
                }

                joint_state_condition.notify_all();
            });


    double joint_error_tolerance{0.01};

    node->get_parameter_or(
        "joint_error_tolerance",
        joint_error_tolerance,
        0.01);

    if (!std::isfinite(joint_error_tolerance) 
        || joint_error_tolerance <= 0.0)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "joint_error_tolerance 必须是有限正数，当前值: %.9e",
            joint_error_tolerance);
        executor.cancel();
        executor_thread.join();
        rclcpp::shutdown();
        return 1;
    }
try
{
    PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "Link_06";
    planner_config.reference_frame = "world";
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";

    MotionRequest ptp_request;
    ptp_request.motion_type = MotionType::kPtp;
    ptp_request.request_id = "ptp_demo_001";

    PoseTarget pose_target;
    pose_target.pose.header.frame_id = "world";
    pose_target.pose.pose.position.x = 0.798761369855;
    pose_target.pose.pose.position.y = -0.003724931204;
    pose_target.pose.pose.position.z = 0.106382015840;

    pose_target.pose.pose.orientation.x = 0.707108079859;
    pose_target.pose.pose.orientation.y = -0.000000008619;
    pose_target.pose.pose.orientation.z = 0.000000008627;
    pose_target.pose.pose.orientation.w = 0.707105482511;

    const PoseTarget normal_pose_target = pose_target;

    if (test_mode == "cancel" || test_mode == "timeout")
    {
        // 故障实验需要一条有足够执行时间的轨迹。该位姿已在 LIN demo 中验证可达，
        // 避免机器人已位于普通目标点时轨迹瞬间完成，使取消和超时可重复触发。
        pose_target.pose.pose.position.x = 0.748761369855;
        pose_target.pose.pose.position.z = 0.21;
    }

    ptp_request.target = pose_target;


    auto planner =
    std::make_shared<massage_motion::MotionPlanningSdk>(
        node, planner_config);

    auto trajectory_executor =
        std::make_shared<massage_motion::MoveItTrajectoryExecutor>(
            node, "/execute_trajectory");

    massage_task::MinimalTaskStateMachine machine(
        planner, trajectory_executor);

    TaskRequest task_request;

    task_request.task_id = "minimal_task_" + test_mode + "_001";
    task_request.motion_request = ptp_request;

    if(test_mode == "timeout")
    {
        // 该超时包含 Action Server 等待、Goal 响应和轨迹执行的总时间。
        task_request.execution_timeout = 0.05;
    }
    else
    {
        task_request.execution_timeout = 10.0;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "开始状态机实验，模式: %s，执行超时: %.3f 秒",
        test_mode.c_str(),
        task_request.execution_timeout
    );

    // 等待第一条反馈
    
    {
        std::unique_lock<std::mutex> lock(joint_state_mutex);

        const bool received_initial_state =
            joint_state_condition.wait_for(
                lock,
                std::chrono::seconds(3),
                [&joint_state_sequence]()
                {
                    return joint_state_sequence > 0;
                });

        if (!received_initial_state)
        {
            throw std::runtime_error(
                "等待 /joint_states 初始反馈超时");
        }
    }

    if (test_mode == "cancel" || test_mode == "timeout")
    {
        // 每次故障实验先回到同一个起点，保证待执行轨迹具有稳定的运动距离。
        MotionRequest preparation_motion_request = ptp_request;
        preparation_motion_request.request_id =
            test_mode + "_preparation_ptp";
        preparation_motion_request.target = normal_pose_target;

        TaskRequest preparation_task_request;
        preparation_task_request.task_id =
            test_mode + "_preparation_task";
        preparation_task_request.motion_request = preparation_motion_request;
        preparation_task_request.execution_timeout = 10.0;

        RCLCPP_INFO(
            node->get_logger(),
            "%s 模式准备阶段：先回到固定实验起点",
            test_mode.c_str());

        const TaskResult preparation_result =
            machine.run(preparation_task_request);

        if (!preparation_result.success ||
            preparation_result.final_state != TaskState::kCompleted ||
            !machine.reset())
        {
            throw std::runtime_error(
                test_mode + " 模式准备阶段失败，无法开始故障实验");
        }
    }

    std::thread cancel_thread;

    if(test_mode == "cancel")
    {
        cancel_thread = std::thread(
            [&machine, logger = node->get_logger()]()
            {
                bool cancel_sent = false;
                bool execution_observed = false;

                while (rclcpp::ok())
                {
                    const auto current_state = machine.state();

                    if (current_state == TaskState::kExecuting)
                    {
                        if (!execution_observed)
                        {
                            execution_observed = true;
                            // 等待 MoveIt 将 Goal 交给控制器，验证真正的执行中取消。
                            std::this_thread::sleep_for(
                                std::chrono::milliseconds(300));
                        }

                        if (machine.cancel())
                        {
                            cancel_sent = true;
                            break;
                        }
                    }
                    else if (current_state == TaskState::kCompleted ||
                        current_state == TaskState::kCanceled ||
                        current_state == TaskState::kFault)
                    {
                        break;
                    }

                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(5));
                }

                RCLCPP_INFO(
                    logger,
                    "取消请求发送结果: %s",
                    cancel_sent ? "成功" : "失败");
            });
    }

    TaskResult result;

    try
    {
        result = machine.run(task_request);
    }
    catch (...)
    {
        if (cancel_thread.joinable())
        {
            cancel_thread.join();
        }
        throw;
    }

    if (cancel_thread.joinable())
    {
        cancel_thread.join();
    }

    const bool experiment_passed =
        is_expected_task_result(test_mode, result);

    RCLCPP_INFO(
        node->get_logger(),
        "模式=%s，实验结果=%s，任务终态=%d，任务错误码=%d，执行终态=%d，执行错误码=%d",
        test_mode.c_str(),
        experiment_passed ? "符合预期" : "不符合预期",
        static_cast<int>(result.final_state),
        static_cast<int>(result.error),
        static_cast<int>(result.execution_result.status),
        static_cast<int>(result.execution_result.error));

    if (!experiment_passed)
    {
        error_code = 1;
    }

    if(test_mode == "normal" && experiment_passed)
    {
        sensor_msgs::msg::JointState actual_joint_state;

        {
            std::unique_lock<std::mutex> lock(joint_state_mutex);

            const std::uint64_t sequence_after_execution =
                joint_state_sequence;

            const bool received_updated_state =
                joint_state_condition.wait_for(
                    lock,
                    std::chrono::seconds(2),
                    [&joint_state_sequence, sequence_after_execution]()
                    {
                        return joint_state_sequence >
                            sequence_after_execution;
                    });

            if (!received_updated_state)
            {
                throw std::runtime_error(
                    "等待执行结束后的 /joint_states 更新超时");
            }

            actual_joint_state = latest_joint_state;
        }

        const auto endpoint_result =
            massage_motion::calculate_trajectory_endpoint_error(
                result.plan_result.trajectory,
                actual_joint_state);

        if (!endpoint_result.valid)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "轨迹终点误差计算失败: %s",
                endpoint_result.message.c_str());
            error_code = 1;
        }
        else
        {
            for (const auto & joint_error : endpoint_result.joint_errors)
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "%s: target=%.6f, actual=%.6f, error=%.6f rad",
                    joint_error.joint_name.c_str(),
                    joint_error.target_position,
                    joint_error.actual_position,
                    joint_error.absolute_error);
            }

            RCLCPP_INFO(
                node->get_logger(),
                "最大关节误差: %.6f rad",
                endpoint_result.max_absolute_error);

            if (endpoint_result.max_absolute_error > joint_error_tolerance)
            {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "终点误差验收失败: 最大误差 %.9e rad 超过阈值 %.9e rad",
                    endpoint_result.max_absolute_error,
                    joint_error_tolerance);
                error_code = 1;
            }
            else
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "终点误差验收通过: 最大误差 %.9e rad，阈值 %.9e rad",
                    endpoint_result.max_absolute_error,
                    joint_error_tolerance);
            }
        }

        RCLCPP_INFO(
            node->get_logger(),
            "任务执行成功，终态=%d，消息=%s",
            static_cast<int>(result.final_state),
            result.message.c_str());
    }
}   
catch (const std::exception & exception)
{
    RCLCPP_ERROR(
        node->get_logger(),
        "规划执行 Demo 发生异常: %s",
        exception.what());
    error_code = 1;
}
    executor.cancel();
    if(executor_thread.joinable())
    {
        executor_thread.join();
    }
    rclcpp::shutdown();
    return error_code;
}
