#include "massage_task/minimal_task_state_machine.hpp"

#include <string>

namespace massage_task
{
using namespace massage_motion;

MinimalTaskStateMachine::MinimalTaskStateMachine(
        std::shared_ptr<massage_motion::IMotionPlanner> planner,
        std::shared_ptr<massage_motion::ITrajectoryExecutor> executor)
        : planner_(planner),
          executor_(executor)
{

}

TaskResult MinimalTaskStateMachine::run(const TaskRequest & request)
{
    const PlanResult empty_plan_result;
    const ExecutionResult empty_execution_result;

    std::unique_lock<std::mutex> run_lock(
        run_mutex_,
        std::try_to_lock
    );

    if(!run_lock.owns_lock())
    {
        return {
            false,
            state_.load(),
            TaskError::kBusy,
            "当前状态机正在运行其他任务",
            empty_plan_result,
            empty_execution_result
        };
    }

    if(state_.load() != TaskState::kIdle)
    {
        return {
            false,
            state_.load(),
            TaskError::kBusy,
            "当前状态机不处于空闲状态",
            empty_plan_result,
            empty_execution_result
        };
    }

    state_.store(TaskState::kPlanning);

    auto plan_result = planner_->plan(request.motion_request);

    // 规划失败时直接返回，不进入执行阶段。
    if(!plan_result.success)
    {
        state_.store(TaskState::kFault);

        return {
            false,
            TaskState::kFault,
            TaskError::kPlanningFailed,
            plan_result.message,
            plan_result,
            empty_execution_result
        };
    }

    if (!request.execute)
    {
        state_.store(TaskState::kCompleted);
        return {
            true,
            TaskState::kCompleted,
            TaskError::kNone,
            "只规划任务完成，未调用轨迹执行器",
            plan_result,
            empty_execution_result
        };
    }

    const auto timing = calculate_execution_timing(
        plan_result.trajectory, request.execution_timing);
    if (!timing.valid)
    {
        state_.store(TaskState::kFault);
        return {
            false,
            TaskState::kFault,
            TaskError::kInvalidRequest,
            "无法确定轨迹执行超时: " + timing.message,
            plan_result,
            empty_execution_result
        };
    }

    ExecutionRequest execution_request;
    execution_request.request_id = request.task_id;
    execution_request.robot_trajectory = plan_result.trajectory;
    execution_request.timeout = timing.timeout;
    execution_request.has_motion_semantics = true;
    execution_request.motion_type = request.motion_request.motion_type;
    execution_request.velocity_scale = request.motion_request.velocity_scale;
    execution_request.acceleration_scale =
        request.motion_request.acceleration_scale;
    execution_request.planner_id = plan_result.planner_id;
    execution_request.motion_target = request.motion_request.target;

    state_.store(TaskState::kExecuting);

    ExecutionResult execution_result = executor_->execute(execution_request);
    const std::string task_message = execution_result.message + "; " + timing.message;

    if(execution_result.success)
    {
        state_.store(TaskState::kCompleted);

        return {
            true,
            TaskState::kCompleted,
            TaskError::kNone,
            task_message,
            plan_result,
            execution_result
        };
    }
    else if(execution_result.status == ExecutionStatus::kCanceled)
    {
        state_.store(TaskState::kCanceled);

        return {
            false,
            TaskState::kCanceled,
            TaskError::kCanceled,
            task_message,
            plan_result,
            execution_result
        };
    }
    else if (execution_result.status == ExecutionStatus::kTimedOut)
    {
        state_.store(TaskState::kFault);

        return {
            false,
            TaskState::kFault,
            TaskError::kTimeout,
            task_message,
            plan_result,
            execution_result
        };
    }

    state_.store(TaskState::kFault);
    return{
        false,
        TaskState::kFault,
        TaskError::kExecutionFailed,
        task_message,
        plan_result,
        execution_result
    };
}

bool MinimalTaskStateMachine::cancel()
{
    if(state_.load() == TaskState::kExecuting)
    {
        return executor_->cancel();
    }

    return false;
}

bool MinimalTaskStateMachine::reset()
{
    if(state_.load()== TaskState::kPlanning
       || state_.load() == TaskState::kExecuting)
    {
        return false;
    }

    state_.store(TaskState::kIdle);
    return true;

}

TaskState MinimalTaskStateMachine::state() const
{
    return state_.load();
}

}
