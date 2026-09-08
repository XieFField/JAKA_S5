#include "massage_motion/hybrid_trajectory_executor.hpp"

#include <stdexcept>
#include <utility>

namespace massage_motion
{

HybridTrajectoryExecutor::HybridTrajectoryExecutor(
  std::shared_ptr<ITrajectoryExecutor> native_ptp,
  std::shared_ptr<ITrajectoryExecutor> moveit_fallback)
: native_ptp_(std::move(native_ptp)),
  moveit_fallback_(std::move(moveit_fallback))
{
  if (!native_ptp_ || !moveit_fallback_)
  {
    throw std::invalid_argument("混合轨迹执行器后端不能为空");
  }
}

HybridTrajectoryExecutor::HybridTrajectoryExecutor(
  std::shared_ptr<ITrajectoryExecutor> native_ptp,
  std::shared_ptr<ITrajectoryExecutor> native_cartesian,
  std::shared_ptr<ITrajectoryExecutor> moveit_fallback)
: native_ptp_(std::move(native_ptp)),
  native_cartesian_(std::move(native_cartesian)),
  moveit_fallback_(std::move(moveit_fallback))
{
  if (!native_ptp_ || !native_cartesian_ || !moveit_fallback_)
  {
    throw std::invalid_argument("混合轨迹执行器后端不能为空");
  }
}

ExecutionResult HybridTrajectoryExecutor::execute(
  const ExecutionRequest & request)
{
  auto selected = moveit_fallback_;
  if (request.has_motion_semantics && request.motion_type == MotionType::kPtp)
  {
    selected = native_ptp_;
  }
  else if (request.has_motion_semantics && native_cartesian_ &&
    (request.motion_type == MotionType::kLin ||
    request.motion_type == MotionType::kCirc))
  {
    selected = native_cartesian_;
  }
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (active_)
    {
      return {
        false, ExecutionError::kRejected, 0,
        "混合轨迹执行器已有活动请求", status_.load()};
    }
    active_ = selected;
  }
  status_.store(ExecutionStatus::kExecuting);
  auto result = selected->execute(request);
  status_.store(result.status);
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_.reset();
  }
  return result;
}

bool HybridTrajectoryExecutor::cancel()
{
  std::shared_ptr<ITrajectoryExecutor> active;
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    active = active_;
  }
  return active && active->cancel();
}

ExecutionStatus HybridTrajectoryExecutor::status() const
{
  return status_.load();
}

}  // namespace massage_motion
