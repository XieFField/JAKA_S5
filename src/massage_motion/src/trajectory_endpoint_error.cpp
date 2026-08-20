#include "massage_motion/trajectory_endpoint_error.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace massage_motion
{

JointTargetErrorResult calculate_joint_target_error(
  const std::vector<std::string> & target_joint_names,
  const std::vector<double> & target_positions,
  const sensor_msgs::msg::JointState & actual_state)
{
  if (target_joint_names.empty())
  {
    return {false, "目标关节名称不能为空", {}, 0.0};
  }
  if (target_joint_names.size() != target_positions.size())
  {
    return {false, "目标关节名称数量与位置数量不匹配", {}, 0.0};
  }
  if (actual_state.name.size() != actual_state.position.size())
  {
    return {false, "实际关节状态名称数量与位置数量不匹配", {}, 0.0};
  }

  std::unordered_map<std::string, double> actual_joint_positions;
  actual_joint_positions.reserve(actual_state.name.size());
  for (std::size_t i = 0; i < actual_state.name.size(); ++i)
  {
    const auto inserted = actual_joint_positions.emplace(
      actual_state.name[i], actual_state.position[i]);
    if (!inserted.second)
    {
      return {
        false,
        "实际关节状态中存在重复关节名称: " + actual_state.name[i],
        {},
        0.0};
    }
  }

  std::unordered_set<std::string> target_joint_set;
  target_joint_set.reserve(target_joint_names.size());
  std::vector<JointPositionError> joint_errors;
  joint_errors.reserve(target_joint_names.size());
  double max_absolute_error = 0.0;

  for (std::size_t i = 0; i < target_joint_names.size(); ++i)
  {
    const auto & joint_name = target_joint_names[i];
    if (joint_name.empty())
    {
      return {false, "目标关节名称不能为空字符串", {}, 0.0};
    }
    if (!target_joint_set.emplace(joint_name).second)
    {
      return {
        false,
        "目标关节中存在重复关节名称: " + joint_name,
        {},
        0.0};
    }

    const auto actual_iterator = actual_joint_positions.find(joint_name);
    if (actual_iterator == actual_joint_positions.end())
    {
      return {
        false,
        "实际关节状态缺少目标关节: " + joint_name,
        {},
        0.0};
    }

    const double target_position = target_positions[i];
    const double actual_position = actual_iterator->second;
    if (!std::isfinite(target_position) || !std::isfinite(actual_position))
    {
      return {
        false,
        "关节位置包含非有限数值: " + joint_name,
        {},
        0.0};
    }

    const double absolute_error = std::abs(target_position - actual_position);
    joint_errors.push_back({
      joint_name, target_position, actual_position, absolute_error});
    max_absolute_error = std::max(max_absolute_error, absolute_error);
  }

  return {
    true, "计算成功", std::move(joint_errors), max_absolute_error};
}

bool joint_target_reached(
  const JointTargetErrorResult & result,
  double tolerance)
{
  if (!result.valid || !std::isfinite(tolerance) || tolerance < 0.0)
  {
    return false;
  }

  const double scale = std::max(
    {1.0, std::abs(result.max_absolute_error), std::abs(tolerance)});
  const double rounding_margin =
    8.0 * std::numeric_limits<double>::epsilon() * scale;
  return result.max_absolute_error <= tolerance + rounding_margin;
}

TrajectoryEndpointErrorResult calculate_trajectory_endpoint_error(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const sensor_msgs::msg::JointState & actual_state)
{
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.points.empty())
  {
    return {false, "规划轨迹没有轨迹点", {}, 0.0};
  }

  const auto result = calculate_joint_target_error(
    joint_trajectory.joint_names,
    joint_trajectory.points.back().positions,
    actual_state);
  if (!result.valid && result.message == "目标关节名称不能为空")
  {
    return {false, "规划轨迹没有关节名称", {}, 0.0};
  }
  if (!result.valid && result.message == "目标关节名称数量与位置数量不匹配")
  {
    return {
      false, "规划关节名称数量与轨迹末点位置数量不匹配", {}, 0.0};
  }
  return result;
}

}  // namespace massage_motion
