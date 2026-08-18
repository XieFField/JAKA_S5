#include "massage_motion/guarded_trajectory_executor.hpp"

#include <utility>

namespace massage_motion
{

GuardedTrajectoryExecutor::GuardedTrajectoryExecutor(
  std::shared_ptr<ITrajectoryExecutor> executor,
  ExecutionPrecondition precondition)
: executor_(std::move(executor)), precondition_(std::move(precondition))
{
}

ExecutionResult GuardedTrajectoryExecutor::execute(
  const ExecutionRequest & request)
{
  if (!executor_ || !precondition_)
  {
    return {
      false, ExecutionError::kBackendUnavailable, 0,
      "轨迹执行器或执行前置条件未配置", ExecutionStatus::kFailed};
  }

  const auto validation = precondition_();
  if (!validation.valid)
  {
    return {
      false, ExecutionError::kRejected, 0,
      "执行前置条件未通过: " + validation.message,
      ExecutionStatus::kFailed};
  }

  return executor_->execute(request);
}

bool GuardedTrajectoryExecutor::cancel()
{
  return executor_ && executor_->cancel();
}

ExecutionStatus GuardedTrajectoryExecutor::status() const
{
  return executor_ ? executor_->status() : ExecutionStatus::kFailed;
}

}  // namespace massage_motion
