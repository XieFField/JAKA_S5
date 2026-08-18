#include <memory>

#include "gtest/gtest.h"

#include "massage_motion/guarded_trajectory_executor.hpp"

namespace massage_motion
{

namespace
{

class FakeExecutor : public ITrajectoryExecutor
{
public:
  ExecutionResult execute(const ExecutionRequest &) override
  {
    ++execute_calls;
    return {true, ExecutionError::kNone, 0, "done", ExecutionStatus::kSucceeded};
  }

  bool cancel() override
  {
    ++cancel_calls;
    return true;
  }

  ExecutionStatus status() const override
  {
    return ExecutionStatus::kIdle;
  }

  int execute_calls{0};
  int cancel_calls{0};
};

}  // namespace

TEST(GuardedTrajectoryExecutorTest, DelegatesWhenPreconditionPasses)
{
  auto backend = std::make_shared<FakeExecutor>();
  GuardedTrajectoryExecutor executor(
    backend,
    []() {return ExecutionValidationResult{true, ExecutionError::kNone, "ready"};});

  ExecutionRequest request;
  const auto result = executor.execute(request);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(backend->execute_calls, 1);
  EXPECT_TRUE(executor.cancel());
  EXPECT_EQ(backend->cancel_calls, 1);
}

TEST(GuardedTrajectoryExecutorTest, RejectsBeforeBackendExecution)
{
  auto backend = std::make_shared<FakeExecutor>();
  GuardedTrajectoryExecutor executor(
    backend,
    []()
    {
      return ExecutionValidationResult{
        false, ExecutionError::kRejected, "robot state changed"};
    });

  ExecutionRequest request;
  const auto result = executor.execute(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, ExecutionError::kRejected);
  EXPECT_EQ(backend->execute_calls, 0);
}

}  // namespace massage_motion
