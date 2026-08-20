#include "massage_motion/execution_timing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"

namespace massage_motion
{

namespace
{

bool duration_seconds(
  const builtin_interfaces::msg::Duration & duration,
  double & seconds)
{
  if (duration.sec < 0 || duration.nanosec >= 1000000000U)
  {
    return false;
  }
  seconds = static_cast<double>(duration.sec) +
    static_cast<double>(duration.nanosec) * 1e-9;
  return std::isfinite(seconds) && seconds >= 0.0;
}

template<typename PointT>
bool final_duration(
  const std::vector<PointT> & points,
  double & duration,
  std::string & message)
{
  if (points.empty())
  {
    return true;
  }

  double previous = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index)
  {
    double current = 0.0;
    if (!duration_seconds(points[index].time_from_start, current))
    {
      message = "轨迹包含无效或负的 time_from_start";
      return false;
    }
    if (index > 0 && current < previous)
    {
      message = "轨迹的 time_from_start 不是单调递增";
      return false;
    }
    previous = current;
  }
  duration = previous;
  return true;
}

}  // namespace

ExecutionTimingResult calculate_execution_timing(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const ExecutionTimingPolicy & policy)
{
  ExecutionTimingResult result;
  if (!std::isfinite(policy.margin) || policy.margin < 0.0)
  {
    result.message = "执行超时余量必须为有限非负数";
    return result;
  }

  const auto & joint_points = trajectory.joint_trajectory.points;
  const auto & multi_dof_points = trajectory.multi_dof_joint_trajectory.points;
  if (joint_points.empty() && multi_dof_points.empty())
  {
    result.message = "无法从空轨迹计算预期执行时间";
    return result;
  }

  double joint_duration = 0.0;
  double multi_dof_duration = 0.0;
  if (!final_duration(joint_points, joint_duration, result.message) ||
    !final_duration(multi_dof_points, multi_dof_duration, result.message))
  {
    return result;
  }
  result.expected_duration = std::max(joint_duration, multi_dof_duration);

  if (policy.timeout_override.has_value())
  {
    const double override = *policy.timeout_override;
    if (!std::isfinite(override) || override <= 0.0)
    {
      result.message = "执行超时覆盖值必须为有限正数";
      return result;
    }
    if (override < result.expected_duration &&
      !policy.allow_shorter_timeout_for_testing)
    {
      std::ostringstream stream;
      stream << "执行超时覆盖值 " << override
             << " s 小于轨迹预期时长 " << result.expected_duration
             << " s";
      result.message = stream.str();
      return result;
    }
    result.timeout = override;
    result.used_override = true;
  }
  else
  {
    result.timeout = result.expected_duration + policy.margin;
  }

  if (!std::isfinite(result.timeout) || result.timeout <= 0.0)
  {
    result.message = "计算得到的执行超时必须为有限正数";
    return result;
  }

  result.valid = true;
  std::ostringstream stream;
  stream << "轨迹预期时长=" << result.expected_duration
         << " s, 超时余量=" << policy.margin
         << " s, 执行超时判定上限=" << result.timeout << " s";
  if (result.used_override)
  {
    stream << " (使用显式覆盖值)";
  }
  result.message = stream.str();
  return result;
}

}  // namespace massage_motion
