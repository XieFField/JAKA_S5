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
        const massage_motion::ExecutionRequest &) override
    {
        ++execute_call_count;
        return next_result;
    }

    bool cancel() override
    {
        ++cancel_call_count;
        return true;
    }

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
}