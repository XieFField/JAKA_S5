#include "rclcpp/rclcpp.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/motion_types.hpp"
#include "massage_motion/execution_types.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <thread>

using namespace massage_motion;

bool report_result(
    const rclcpp::Logger & logger,
    const std::string & stage_name,
    const PlanResult & result)
{
    if(!result.success)
    {
        RCLCPP_ERROR(
            logger,
            "%s规划失败，错误码: %d, moveit错误码: %d, 消息: %s",
            stage_name.c_str(),
            static_cast<int>(result.error),
            result.moveit_error_code,
            result.message.c_str()
        );
        return false;
    }
    else
    {
        RCLCPP_INFO(
            logger,
            "%s规划成功,规划时间: %.3f 秒，轨迹点数: %zu",
            stage_name.c_str(),
            result.planning_time,
            result.trajectory.joint_trajectory.points.size()
        );
        return true;
    }
}

bool is_expected_execution_result(
    const std::string & test_mode,
    const ExecutionResult & result)
{
    if (test_mode == "normal")
    {
        return result.success &&
            result.error == ExecutionError::kNone &&
            result.status == ExecutionStatus::kSucceeded;
    }

    if (test_mode == "cancel")
    {
        return !result.success &&
            result.error == ExecutionError::kCanceled &&
            result.status == ExecutionStatus::kCanceled;
    }

    return !result.success &&
        result.error == ExecutionError::kTimeout &&
        result.status == ExecutionStatus::kTimedOut;
}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "planning_execution_demo",
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

    // 在单独的线程中运行执行器
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    int error_code = 0;

    // MoveIt 相关对象必须在 rclcpp::shutdown() 之前析构。
    try
    {
        PlannerConfig planner_config;
        planner_config.planning_group = "jaka_s5";
        planner_config.end_effector_link = "Link_06";
        planner_config.reference_frame = "world";
        planner_config.planning_pipeline = "pilz_industrial_motion_planner";

        MotionPlanningSdk sdk(node, planner_config);

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

        ptp_request.target = pose_target;

        const auto plan_result = sdk.plan(ptp_request);

        if (!report_result(node->get_logger(), "PTP", plan_result))
        {
            error_code = 1;
        }
        else if (plan_result.trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "PTP 规划结果中没有轨迹点，无法进入执行阶段");
            error_code = 1;
        }
        else
        {
            const auto & final_time =
                plan_result.trajectory.joint_trajectory.points.back().time_from_start;

            const double trajectory_duration =
                static_cast<double>(final_time.sec) +
                static_cast<double>(final_time.nanosec) * 1e-9;

            ExecutionRequest execution_request;
            execution_request.robot_trajectory = plan_result.trajectory;
            execution_request.request_id = "execution_demo_001";

            if (test_mode == "timeout")
            {
                // 明确短于预计轨迹时间，使 execute() 进入超时取消流程。
                execution_request.timeout = std::max(
                    0.1,
                    std::min(0.5, trajectory_duration * 0.3));
            }
            else
            {
                execution_request.timeout = trajectory_duration + 5.0;
            }

            RCLCPP_INFO(
                node->get_logger(),
                "开始执行实验，模式: %s，预计轨迹时间: %.3f 秒，执行超时: %.3f 秒",
                test_mode.c_str(),
                trajectory_duration,
                execution_request.timeout);

            MoveItTrajectoryExecutor jaka_executor(
                node,
                "/execute_trajectory");

            std::thread cancel_thread;

            if (test_mode == "cancel")
            {
                cancel_thread = std::thread(
                    [&jaka_executor, logger = node->get_logger()]()
                    {
                        // 等待 execute() 获得 GoalHandle 并进入执行状态。
                        while (rclcpp::ok())
                        {
                            const auto current_status = jaka_executor.status();

                            if (current_status == ExecutionStatus::kExecuting)
                            {
                                break;
                            }

                            if (current_status == ExecutionStatus::kFailed ||
                                current_status == ExecutionStatus::kTimedOut ||
                                current_status == ExecutionStatus::kSucceeded ||
                                current_status == ExecutionStatus::kCanceled)
                            {
                                return;
                            }

                            std::this_thread::sleep_for(
                                std::chrono::milliseconds(10));
                        }

                        if (!rclcpp::ok())
                        {
                            return;
                        }

                        // 留出一小段时间，便于观察机械臂已经开始运动。
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(300));

                        const bool cancel_sent = jaka_executor.cancel();

                        RCLCPP_INFO(
                            logger,
                            "取消请求发送结果: %s",
                            cancel_sent ? "已发送" : "未发送");
                    });
            }

            ExecutionResult execution_result;

            try
            {
                execution_result = jaka_executor.execute(execution_request);
            }
            catch (...)
            {
                // 防止异常展开时析构仍为 joinable 的线程而触发 terminate。
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
                is_expected_execution_result(test_mode, execution_result);

            RCLCPP_INFO(
                node->get_logger(),
                "模式=%s，实验结果=%s，终态=%d，错误码=%d，后端错误码=%d，消息=%s",
                test_mode.c_str(),
                experiment_passed ? "符合预期" : "不符合预期",
                static_cast<int>(execution_result.status),
                static_cast<int>(execution_result.error),
                execution_result.backend_error_code,
                execution_result.message.c_str());

            error_code = experiment_passed ? 0 : 1;
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

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return error_code;
}
