#include <limits>

#include "gtest/gtest.h"

#include "massage_task/task_types.hpp"
#include "massage_task/minimal_task_state_machine.hpp"

class FakePlanner final : public massage_motion::IMotionPlanner
{
public:
    massage_motion::PlanResult next_result;
    int call_count{0};

    massage_motion::PlanResult plan(
        const massage_motion::MotionRequest &) override
    {
        ++call_count;
        return next_result;
    }
};

class FakeExecutor final : public massage_motion::ITrajectoryExecutor
{
public:
    massage_motion::ExecutionResult next_result;
    int execute_call_count{0};
    int cancel_call_count{0};

    massage_motion::ExecutionResult execute(
        const massage_motion::ExecutionRequest & request) override
    {
        ++execute_call_count;
        last_request = request;
        return next_result;
    }

    bool cancel() override
    {
        ++cancel_call_count;
        return true;
    }
    massage_motion::ExecutionRequest last_request;
    massage_motion::ExecutionStatus status() const override
    {
        return next_result.status;
    }
};

TEST(MinimalTaskStateMachineTest, PlanningAndExecutionSuccess)
{
    auto planner = std::make_shared<FakePlanner>();
    auto executor = std::make_shared<FakeExecutor>();

    planner->next_result.success = true;

    executor->next_result.success = true;
    executor->next_result.status =
        massage_motion::ExecutionStatus::kSucceeded;

    massage_task::MinimalTaskStateMachine machine(planner, executor);

    massage_task::TaskRequest request;
    request.task_id = "success_test";
    request.execution_timeout = 5.0;

    const auto result = machine.run(request);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.error, massage_task::TaskError::kNone);
    EXPECT_EQ(result.final_state, massage_task::TaskState::kCompleted);
    EXPECT_EQ(machine.state(), massage_task::TaskState::kCompleted);

    EXPECT_EQ(planner->call_count, 1);
    EXPECT_EQ(executor->execute_call_count, 1);
    EXPECT_EQ(executor->last_request.request_id, request.task_id);
    EXPECT_DOUBLE_EQ(executor->last_request.timeout, request.execution_timeout);
}

TEST(MinimalTaskStateMachineTest, PlanningFailure)
{
    auto planner = std::make_shared<FakePlanner>();
    auto executor = std::make_shared<FakeExecutor>();

    planner->next_result.success = false;
    planner->next_result.message = "Planning failed";

    massage_task::MinimalTaskStateMachine machine(planner, executor);

    massage_task::TaskRequest request;
    request.task_id = "planning_failure_test";
    request.execution_timeout = 5.0;

    const auto result = machine.run(request);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error, massage_task::TaskError::kPlanningFailed);
    EXPECT_EQ(result.final_state, massage_task::TaskState::kFault);
    EXPECT_EQ(machine.state(), massage_task::TaskState::kFault);

    EXPECT_EQ(planner->call_count, 1);
    EXPECT_EQ(executor->execute_call_count, 0);
}

TEST(MinimalTaskStateMachineTest, ExecutionFailure)
{
    auto planner = std::make_shared<FakePlanner>();
    auto executor = std::make_shared<FakeExecutor>();

    planner->next_result.success = true;

    executor->next_result.success = false;
    executor->next_result.status =
        massage_motion::ExecutionStatus::kFailed;
    executor->next_result.message = "Execution failed";

    massage_task::MinimalTaskStateMachine machine(planner, executor);

    massage_task::TaskRequest request;
    request.task_id = "execution_failure_test";
    request.execution_timeout = 5.0;

    const auto result = machine.run(request);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error, massage_task::TaskError::kExecutionFailed);
    EXPECT_EQ(result.final_state, massage_task::TaskState::kFault);
    EXPECT_EQ(machine.state(), massage_task::TaskState::kFault);

    EXPECT_EQ(planner->call_count, 1);
    EXPECT_EQ(executor->execute_call_count, 1);
}

TEST(MinimalTaskStateMachineTest, ExecutionCanceled)
{
    auto planner = std::make_shared<FakePlanner>();
    auto executor = std::make_shared<FakeExecutor>();

    planner->next_result.success = true;

    executor->next_result.success = false;
    executor->next_result.status =
        massage_motion::ExecutionStatus::kCanceled;
    executor->next_result.message = "Execution canceled";

    massage_task::MinimalTaskStateMachine machine(planner, executor);

    massage_task::TaskRequest request;
    request.task_id = "execution_canceled_test";
    request.execution_timeout = 5.0;

    const auto result = machine.run(request);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error, massage_task::TaskError::kCanceled);
    EXPECT_EQ(result.final_state, massage_task::TaskState::kCanceled);
    EXPECT_EQ(machine.state(), massage_task::TaskState::kCanceled);

    EXPECT_EQ(planner->call_count, 1);
    EXPECT_EQ(executor->execute_call_count, 1);
}

TEST(MinimalTaskStateMachineTest, ExecutionTimedOut)
{
    auto planner = std::make_shared<FakePlanner>();
    auto executor = std::make_shared<FakeExecutor>();

    planner->next_result.success = true;

    executor->next_result.success = false;
    executor->next_result.status =
        massage_motion::ExecutionStatus::kTimedOut;
    executor->next_result.message = "Execution timed out";

    massage_task::MinimalTaskStateMachine machine(planner, executor);

    massage_task::TaskRequest request;
    request.task_id = "execution_timeout_test";
    request.execution_timeout = 5.0;

    const auto result = machine.run(request);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error, massage_task::TaskError::kTimeout);
    EXPECT_EQ(result.final_state, massage_task::TaskState::kFault);
    EXPECT_EQ(machine.state(), massage_task::TaskState::kFault);

    EXPECT_EQ(planner->call_count, 1);
    EXPECT_EQ(executor->execute_call_count, 1);
}

//非idle 再运行
TEST(MinimalTaskStateMachineTest, RunWhileBusy)
{
    auto planner = std::make_shared<FakePlanner>();
    auto executor = std::make_shared<FakeExecutor>();

    planner->next_result.success = true;

    executor->next_result.success = true;
    executor->next_result.status =
        massage_motion::ExecutionStatus::kSucceeded;

    massage_task::MinimalTaskStateMachine machine(planner, executor);

    massage_task::TaskRequest request;
    request.task_id = "run_while_busy_test";
    request.execution_timeout = 5.0;

    // 第一次运行
    const auto result1 = machine.run(request);
    EXPECT_TRUE(result1.success);
    EXPECT_EQ(result1.final_state, massage_task::TaskState::kCompleted);

    // 第二次运行，应该返回忙碌错误
    const auto result2 = machine.run(request);
    EXPECT_FALSE(result2.success);
    EXPECT_EQ(result2.error, massage_task::TaskError::kBusy);
    EXPECT_EQ(executor->execute_call_count, 1); // 确保执行器只被调用了一次
    EXPECT_EQ(planner->call_count, 1);
    EXPECT_EQ(executor->execute_call_count, 1);
    EXPECT_EQ(machine.state(), massage_task::TaskState::kCompleted);
    EXPECT_EQ(result2.final_state, massage_task::TaskState::kCompleted);
}

// reset 测试
TEST(MinimalTaskStateMachineTest, ResetAfterCompletion)
{
    auto planner = std::make_shared<FakePlanner>();
    auto executor = std::make_shared<FakeExecutor>();
    planner->next_result.success = true;
    executor->next_result.success = true;
    executor->next_result.status = massage_motion::ExecutionStatus::kSucceeded;

    massage_task::MinimalTaskStateMachine machine(planner, executor);

    massage_task::TaskRequest request;
    request.task_id = "reset_after_completion_test";
    request.execution_timeout = 5.0;

    const auto result1 = machine.run(request);
    EXPECT_TRUE(result1.success);
    EXPECT_EQ(machine.state(), massage_task::TaskState::kCompleted);
    EXPECT_TRUE(machine.reset());
    EXPECT_EQ(machine.state(), massage_task::TaskState::kIdle);

    const auto result2 = machine.run(request);
    EXPECT_TRUE(result2.success);
    EXPECT_EQ(planner->call_count, 2);
    EXPECT_EQ(executor->execute_call_count, 2);
}
