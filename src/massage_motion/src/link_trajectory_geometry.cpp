#include "massage_motion/link_trajectory_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "moveit/robot_state/robot_state.h"

namespace massage_motion
{

namespace
{

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

}  // namespace

LinkHeightMetrics calculate_link_height_metrics(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & link_name)
{
  LinkHeightMetrics metrics;
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (!robot_model)
  {
    metrics.message = "RobotModel 为空";
    return metrics;
  }
  if (link_name.empty() || !robot_model->hasLinkModel(link_name))
  {
    metrics.message = "RobotModel 中不存在诊断连杆: " + link_name;
    return metrics;
  }
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty())
  {
    metrics.message = "轨迹关节名或轨迹点为空";
    return metrics;
  }

  std::unordered_set<std::string> unique_joint_names;
  const std::unordered_set<std::string> model_variable_names(
    robot_model->getVariableNames().begin(),
    robot_model->getVariableNames().end());
  for (const auto & joint_name : joint_trajectory.joint_names)
  {
    if (joint_name.empty() || !unique_joint_names.insert(joint_name).second)
    {
      metrics.message = "轨迹包含空关节名或重复关节名";
      return metrics;
    }
    if (model_variable_names.count(joint_name) == 0U)
    {
      metrics.message = "RobotModel 中不存在轨迹关节变量: " + joint_name;
      return metrics;
    }
  }

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  metrics.minimum_z = std::numeric_limits<double>::infinity();
  metrics.maximum_z = -std::numeric_limits<double>::infinity();
  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t point_index = 0;
    point_index < joint_trajectory.points.size(); ++point_index)
  {
    const auto & point = joint_trajectory.points[point_index];
    if (point.positions.size() != joint_trajectory.joint_names.size())
    {
      metrics.message = "轨迹点关节位置数量不一致";
      return metrics;
    }
    if (!std::all_of(
        point.positions.begin(), point.positions.end(),
        [](double value) {return std::isfinite(value);}))
    {
      metrics.message = "轨迹点包含非有限关节位置";
      return metrics;
    }
    const double point_time = duration_seconds(point.time_from_start);
    if (!std::isfinite(point_time) || point_time < 0.0 ||
      point_time < previous_time)
    {
      metrics.message = "轨迹点时间无效或不是单调非递减";
      return metrics;
    }
    previous_time = point_time;

    state.setVariablePositions(joint_trajectory.joint_names, point.positions);
    state.updateLinkTransforms();
    const double z = state.getGlobalLinkTransform(link_name).translation().z();
    if (!std::isfinite(z))
    {
      metrics.message = "连杆 FK 高度不是有限数值";
      return metrics;
    }
    if (point_index == 0U)
    {
      metrics.start_z = z;
    }
    metrics.end_z = z;
    metrics.maximum_z = std::max(metrics.maximum_z, z);
    if (z < metrics.minimum_z)
    {
      metrics.minimum_z = z;
      metrics.minimum_point_index = point_index;
      metrics.minimum_point_time = point_time;
    }
  }
  metrics.maximum_drop_below_start = metrics.start_z - metrics.minimum_z;
  metrics.valid = true;
  metrics.message = "连杆全轨迹 FK 诊断成功";
  return metrics;
}

LinkHeightAtJointTargetResult calculate_link_height_at_joint_target(
  const moveit::core::RobotModelConstPtr & robot_model,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  const std::string & link_name)
{
  LinkHeightAtJointTargetResult result;
  if (!robot_model)
  {
    result.message = "RobotModel 为空";
    return result;
  }
  if (link_name.empty() || !robot_model->hasLinkModel(link_name))
  {
    result.message = "RobotModel 中不存在诊断连杆: " + link_name;
    return result;
  }
  if (joint_names.empty() || joint_names.size() != joint_positions.size())
  {
    result.message = "关节目标名称为空或名称与位置数量不一致";
    return result;
  }

  std::unordered_map<std::string, double> target;
  target.reserve(joint_names.size());
  for (std::size_t index = 0; index < joint_names.size(); ++index)
  {
    if (joint_names[index].empty() || !std::isfinite(joint_positions[index]) ||
      !target.emplace(joint_names[index], joint_positions[index]).second)
    {
      result.message = "关节目标包含空名称、重复名称或非有限位置";
      return result;
    }
  }

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  for (const auto & variable_name : robot_model->getVariableNames())
  {
    const auto target_position = target.find(variable_name);
    if (target_position == target.end())
    {
      result.message = "关节目标缺少 RobotModel 变量: " + variable_name;
      return result;
    }
    state.setVariablePosition(variable_name, target_position->second);
  }
  state.updateLinkTransforms();
  result.z = state.getGlobalLinkTransform(link_name).translation().z();
  if (!std::isfinite(result.z))
  {
    result.message = "连杆 FK 高度不是有限数值";
    return result;
  }
  result.valid = true;
  result.message = "关节目标连杆 FK 高度计算成功";
  return result;
}

LinkHeightGateResult evaluate_link_height_gate(
  const LinkHeightMetrics & metrics,
  const LinkHeightGateConfig & config)
{
  LinkHeightGateResult result;
  if (!config.enabled)
  {
    result.valid = true;
    result.accepted = true;
    result.message = "连杆高度门禁已关闭";
    return result;
  }
  if (!std::isfinite(config.minimum_z) ||
    !std::isfinite(config.reference_z) ||
    !std::isfinite(config.maximum_drop_below_reference) ||
    config.maximum_drop_below_reference < 0.0)
  {
    result.message = "连杆高度门禁参数无效";
    return result;
  }
  if (!metrics.valid)
  {
    result.message = "缺少有效的连杆全轨迹 FK 证据: " + metrics.message;
    return result;
  }
  if (!std::isfinite(metrics.start_z) || !std::isfinite(metrics.end_z) ||
    !std::isfinite(metrics.minimum_z) || !std::isfinite(metrics.maximum_z) ||
    !std::isfinite(metrics.maximum_drop_below_start))
  {
    result.message = "连杆高度指标包含非有限数值";
    return result;
  }

  result.valid = true;
  result.minimum_allowed_z = std::max(
    config.minimum_z,
    config.reference_z - config.maximum_drop_below_reference);
  result.drop_below_reference = config.reference_z - metrics.minimum_z;
  constexpr double tolerance = 1.0e-9;
  const bool below_minimum = metrics.minimum_z + tolerance < config.minimum_z;
  const bool excessive_drop =
    result.drop_below_reference >
    config.maximum_drop_below_reference + tolerance;
  if (!below_minimum && !excessive_drop)
  {
    result.accepted = true;
    result.message = "连杆绝对高度和相对参考待机位下降量均通过门禁";
    return result;
  }

  std::ostringstream stream;
  if (below_minimum)
  {
    stream << "minimum_z=" << metrics.minimum_z << " m 低于阈值 "
           << config.minimum_z << " m";
  }
  if (excessive_drop)
  {
    if (below_minimum)
    {
      stream << "; ";
    }
    stream << "drop_below_reference=" << result.drop_below_reference
           << " m 超过阈值 "
           << config.maximum_drop_below_reference << " m";
  }
  result.message = stream.str();
  return result;
}

}  // namespace massage_motion
