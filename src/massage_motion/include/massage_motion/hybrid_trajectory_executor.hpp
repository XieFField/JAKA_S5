#ifndef MASSAGE_MOTION__HYBRID_TRAJECTORY_EXECUTOR_HPP_
#define MASSAGE_MOTION__HYBRID_TRAJECTORY_EXECUTOR_HPP_

#include <atomic>
#include <memory>
#include <mutex>

#include "massage_motion/trajectory_executor.hpp"

namespace massage_motion
{

// Routes semantically identified PTP commands to the native backend. All
// other requests retain the existing MoveIt execution path during phase one.
class HybridTrajectoryExecutor final : public ITrajectoryExecutor
{
public:
  HybridTrajectoryExecutor(
    std::shared_ptr<ITrajectoryExecutor> native_ptp,
    std::shared_ptr<ITrajectoryExecutor> moveit_fallback);
  HybridTrajectoryExecutor(
    std::shared_ptr<ITrajectoryExecutor> native_ptp,
    std::shared_ptr<ITrajectoryExecutor> native_cartesian,
    std::shared_ptr<ITrajectoryExecutor> moveit_fallback);

  ExecutionResult execute(const ExecutionRequest & request) override;
  bool cancel() override;
  ExecutionStatus status() const override;

private:
  std::shared_ptr<ITrajectoryExecutor> native_ptp_;
  std::shared_ptr<ITrajectoryExecutor> native_cartesian_;
  std::shared_ptr<ITrajectoryExecutor> moveit_fallback_;
  mutable std::mutex active_mutex_;
  std::shared_ptr<ITrajectoryExecutor> active_;
  std::atomic<ExecutionStatus> status_{ExecutionStatus::kIdle};
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__HYBRID_TRAJECTORY_EXECUTOR_HPP_
