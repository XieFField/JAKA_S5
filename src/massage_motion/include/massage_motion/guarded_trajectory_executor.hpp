#ifndef MASSAGE_MOTION__GUARDED_TRAJECTORY_EXECUTOR_HPP_
#define MASSAGE_MOTION__GUARDED_TRAJECTORY_EXECUTOR_HPP_

#include <functional>
#include <memory>

#include "massage_motion/execution_types.hpp"
#include "massage_motion/trajectory_executor.hpp"

namespace massage_motion
{

using ExecutionPrecondition = std::function<ExecutionValidationResult()>;

class GuardedTrajectoryExecutor final : public ITrajectoryExecutor
{
public:
  GuardedTrajectoryExecutor(
    std::shared_ptr<ITrajectoryExecutor> executor,
    ExecutionPrecondition precondition);

  ExecutionResult execute(const ExecutionRequest & request) override;
  bool cancel() override;
  ExecutionStatus status() const override;

private:
  std::shared_ptr<ITrajectoryExecutor> executor_;
  ExecutionPrecondition precondition_;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__GUARDED_TRAJECTORY_EXECUTOR_HPP_
