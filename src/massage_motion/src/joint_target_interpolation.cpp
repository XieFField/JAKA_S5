#include "massage_motion/joint_target_interpolation.hpp"

#include <algorithm>
#include <cmath>

namespace massage_motion
{

JointTargetInterpolationResult interpolate_joint_target(
  const std::vector<double> & start_positions,
  const std::vector<double> & goal_positions,
  double ratio,
  double maximum_joint_travel)
{
  JointTargetInterpolationResult result;
  if (start_positions.empty() ||
    start_positions.size() != goal_positions.size())
  {
    result.message = "起点和终点关节数组必须非空且长度一致";
    return result;
  }
  if (!std::isfinite(ratio) || ratio <= 0.0 || ratio > 1.0)
  {
    result.message = "分段比例必须是 (0, 1] 内的有限数值";
    return result;
  }
  if (!std::isfinite(maximum_joint_travel) || maximum_joint_travel <= 0.0)
  {
    result.message = "最大单关节行程必须是有限正数";
    return result;
  }

  result.target_positions.reserve(start_positions.size());
  result.joint_travels.reserve(start_positions.size());
  for (std::size_t joint = 0; joint < start_positions.size(); ++joint)
  {
    if (!std::isfinite(start_positions[joint]) ||
      !std::isfinite(goal_positions[joint]))
    {
      result.message = "起点或终点包含非有限关节位置";
      result.target_positions.clear();
      result.joint_travels.clear();
      return result;
    }

    const double full_delta = goal_positions[joint] - start_positions[joint];
    const double travel = ratio * full_delta;
    const double target = start_positions[joint] + travel;
    if (!std::isfinite(full_delta) || !std::isfinite(travel) ||
      !std::isfinite(target))
    {
      result.message = "分段目标计算产生非有限数值";
      result.target_positions.clear();
      result.joint_travels.clear();
      return result;
    }
    result.target_positions.push_back(target);
    result.joint_travels.push_back(travel);
    result.maximum_joint_travel = std::max(
      result.maximum_joint_travel, std::abs(travel));
  }

  if (result.maximum_joint_travel > maximum_joint_travel)
  {
    result.message = "分段目标超过最大单关节行程";
    result.target_positions.clear();
    result.joint_travels.clear();
    return result;
  }

  result.valid = true;
  result.message = "分段关节目标计算成功";
  return result;
}

}  // namespace massage_motion
