#include "massage_motion/moveit_trajectory_executor.hpp"

#include <chrono>
#include <future>

namespace massage_motion
{

MoveItTrajectoryExecutor::MoveItTrajectoryExecutor(
    const rclcpp::Node::SharedPtr & node,
    const std::string & action_name)
    :logger_ (node->get_logger())
{
    
    action_client_ =
        rclcpp_action::create_client<ExecuteTrajectory>(
            node,
            action_name);

    execution_event_publisher_ =
        node->create_publisher<std_msgs::msg::String>(
            "/trajectory_execution_event",
            rclcpp::QoS(1));
}

ExecutionStatus MoveItTrajectoryExecutor::status() const
{
    return status_.load();
}

ExecutionResult MoveItTrajectoryExecutor::execute(
    const ExecutionRequest & request)
{
    std::unique_lock<std::mutex> execution_lock(
        execution_mutex_,
        std::try_to_lock
    );

    if(!execution_lock.owns_lock())
    {
        return {
            false,
            ExecutionError::kRejected,
            0,
            "当前执行器正在执行其他轨迹",
            status_.load()
        };
    }

    // 上一次超时后如果没有确认终态，保留目标句柄并拒绝新轨迹，
    // 避免上一条轨迹可能仍在运行时启动下一条轨迹。
    {
        std::lock_guard<std::mutex> goal_lock(goal_mutex_);
        if (active_goal_)
        {
            return {
                false,
                ExecutionError::kRejected,
                0,
                "上一条轨迹尚未确认终态",
                status_.load()
            };
        }
    }

    const auto validation_result =
        validate_execution_request(request);

    if (!validation_result.valid)
    {
        status_.store(ExecutionStatus::kFailed);

        return {
            false,
            validation_result.error,
            0,
            validation_result.message,
            ExecutionStatus::kFailed
        };
    }

    using SteadyClock = std::chrono::steady_clock;
    const auto timeout_duration =
        std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::duration<double>(request.timeout));
    const auto deadline = SteadyClock::now() + timeout_duration;

    const auto server_wait_time = deadline - SteadyClock::now();
    if (server_wait_time <= SteadyClock::duration::zero() ||
        !action_client_->wait_for_action_server(server_wait_time))
    {
        status_.store(ExecutionStatus::kFailed);

        return {
            false,
            ExecutionError::kBackendUnavailable,
            0,
            "在执行期限内未发现 ExecuteTrajectory Action Server",
            ExecutionStatus::kFailed
        };
    }

    ExecuteTrajectory::Goal goal;
    goal.trajectory = request.robot_trajectory;

    // 如果 Goal 响应晚于总截止时间，回调会取消这个迟到但已被接受的 Goal。
    auto abandon_goal = std::make_shared<std::atomic_bool>(false);
    auto late_cancel_sent = std::make_shared<std::atomic_bool>(false);
    auto client = action_client_;

    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions goal_options;
    goal_options.goal_response_callback =
        [client, abandon_goal, late_cancel_sent](GoalHandle::SharedPtr goal_handle)
        {
            if (goal_handle && abandon_goal->load() &&
                !late_cancel_sent->exchange(true))
            {
                client->async_cancel_goal(goal_handle);
            }
        };

    auto goal_future = action_client_->async_send_goal(goal, goal_options);

    if (goal_future.wait_until(deadline) != std::future_status::ready)
    {
        abandon_goal->store(true);

        // 处理响应与截止时间同时发生的竞争，避免遗漏已经到达的 GoalHandle。
        if (goal_future.wait_for(std::chrono::seconds(0)) ==
            std::future_status::ready)
        {
            const auto late_goal_handle = goal_future.get();
            if (late_goal_handle && !late_cancel_sent->exchange(true))
            {
                action_client_->async_cancel_goal(late_goal_handle);
            }
        }

        status_.store(ExecutionStatus::kTimedOut);

        return {
            false,
            ExecutionError::kTimeout,
            0,
            "等待 ExecuteTrajectory Goal 响应超时",
            ExecutionStatus::kTimedOut
        };
    }

    const auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
        status_.store(ExecutionStatus::kFailed);

        return {
            false,
            ExecutionError::kRejected,
            0,
            "MoveIt 拒绝了轨迹执行请求",
            ExecutionStatus::kFailed
        };
    }

    {
        std::lock_guard<std::mutex> goal_lock(goal_mutex_);
        active_goal_ = goal_handle;
    }

    status_.store(ExecutionStatus::kExecuting);

    auto result_future = action_client_->async_get_result(goal_handle);

    if (result_future.wait_until(deadline) != std::future_status::ready)
    {
        // 到达总执行期限后先请求取消，再给服务端短暂时间确认终态。
        cancel();

        constexpr auto cancel_grace_period = std::chrono::seconds(2);
        if (result_future.wait_for(cancel_grace_period) !=
            std::future_status::ready)
        {
            // 未确认终态时保留 active_goal_，后续 execute() 会拒绝新轨迹。
            status_.store(ExecutionStatus::kTimedOut);

            return {
                false,
                ExecutionError::kTimeout,
                0,
                "轨迹执行超时，已发送取消请求但尚未确认终态",
                ExecutionStatus::kTimedOut
            };
        }

        const auto timeout_result = result_future.get();
        const auto backend_error_code = timeout_result.result ?
            timeout_result.result->error_code.val : 0;

        {
            std::lock_guard<std::mutex> goal_lock(goal_mutex_);
            active_goal_.reset();
        }

        status_.store(ExecutionStatus::kTimedOut);

        return {
            false,
            ExecutionError::kTimeout,
            backend_error_code,
            "轨迹执行超时，取消流程已结束",
            ExecutionStatus::kTimedOut
        };
    }

    const auto wrapped_result = result_future.get();

    {
        std::lock_guard<std::mutex> goal_lock(goal_mutex_);
        active_goal_.reset();
    }

    if (!wrapped_result.result)
    {
        status_.store(ExecutionStatus::kFailed);

        return{
            false,
            ExecutionError::kExecutionFailed,
            0,
            "服务端没有返回 ExecuteTrajectory::Result",
            ExecutionStatus::kFailed
        };
    }

    const auto backend_error_code = wrapped_result.result->error_code.val;

    if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED ||
        backend_error_code == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
    {
        status_.store(ExecutionStatus::kCanceled);

        return {
            false,
            ExecutionError::kCanceled,
            backend_error_code,
            "轨迹执行已取消",
            ExecutionStatus::kCanceled
        };
    }

    if (backend_error_code == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT)
    {
        status_.store(ExecutionStatus::kTimedOut);

        return {
            false,
            ExecutionError::kTimeout,
            backend_error_code,
            "MoveIt 报告轨迹执行超时",
            ExecutionStatus::kTimedOut
        };
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        backend_error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        status_.store(ExecutionStatus::kFailed);

        return{
            false,
            ExecutionError::kExecutionFailed,
            backend_error_code,
            "轨迹执行失败",
            ExecutionStatus::kFailed
        };
    }

    status_.store(ExecutionStatus::kSucceeded);

    return {
        true,
        ExecutionError::kNone,
        backend_error_code,
        "MoveIt 执行轨迹成功",
        ExecutionStatus::kSucceeded
    };
}

bool MoveItTrajectoryExecutor::cancel()
{
    GoalHandle::SharedPtr goal_handle;

    {
        std::lock_guard<std::mutex> goal_lock(goal_mutex_);
        goal_handle = active_goal_;
    }

    if (!goal_handle)
    {
        return false;
    }

    try
    {
        // MoveIt 的轨迹执行管理器订阅该事件并立即停止当前控制器。
        // 同时提交标准 Action cancel，使 Goal 的终态也能转换为 CANCELED。
        std_msgs::msg::String stop_event;
        stop_event.data = "stop";
        execution_event_publisher_->publish(stop_event);
        action_client_->async_cancel_goal(goal_handle);
    }
    catch (const std::exception & exception)
    {
        RCLCPP_WARN(
            logger_,
            "发送轨迹取消请求失败: %s",
            exception.what());
        return false;
    }

    return true;
}


}// namespace massage_motion
