#include "massage_motion/elbow_posture_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "Eigen/Geometry"
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

bool finite_normal(const std::array<double, 3> & normal)
{
  return std::all_of(
    normal.begin(), normal.end(),
    [](double value) {return std::isfinite(value);});
}

}  // namespace

ElbowPostureMetrics calculate_elbow_posture_metrics(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const ElbowPostureGeometryConfig & config)
{
  ElbowPostureMetrics metrics;
  if (!robot_model)
  {
    metrics.message = "RobotModel 为空";
    return metrics;
  }
  if (config.shoulder_link.empty() || config.elbow_link.empty() ||
    config.wrist_link.empty() ||
    !robot_model->hasLinkModel(config.shoulder_link) ||
    !robot_model->hasLinkModel(config.elbow_link) ||
    !robot_model->hasLinkModel(config.wrist_link))
  {
    metrics.message = "肩、肘或腕诊断连杆为空或不存在于 RobotModel";
    return metrics;
  }
  if (!finite_normal(config.surface_normal) ||
    !std::isfinite(config.side_tolerance) || config.side_tolerance < 0.0 ||
    !std::isfinite(config.observability_tolerance) ||
    config.observability_tolerance < 0.0 ||
    !std::isfinite(config.minimum_shoulder_wrist_distance) ||
    config.minimum_shoulder_wrist_distance <= 0.0)
  {
    metrics.message = "肘部构型诊断参数无效";
    return metrics;
  }
  Eigen::Vector3d surface_normal(
    config.surface_normal[0], config.surface_normal[1],
    config.surface_normal[2]);
  const double surface_normal_length = surface_normal.norm();
  if (!std::isfinite(surface_normal_length) ||
    surface_normal_length <= std::numeric_limits<double>::epsilon())
  {
    metrics.message = "表面法向量长度为零或不是有限数值";
    return metrics;
  }
  surface_normal /= surface_normal_length;

  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty())
  {
    metrics.message = "轨迹关节名或轨迹点为空";
    return metrics;
  }
  std::unordered_set<std::string> trajectory_variables;
  trajectory_variables.reserve(joint_trajectory.joint_names.size());
  const std::unordered_set<std::string> model_variables(
    robot_model->getVariableNames().begin(),
    robot_model->getVariableNames().end());
  for (const auto & joint_name : joint_trajectory.joint_names)
  {
    if (joint_name.empty() ||
      !trajectory_variables.emplace(joint_name).second ||
      model_variables.count(joint_name) == 0U)
    {
      metrics.message = "轨迹包含空名称、重复名称或未知关节变量";
      return metrics;
    }
  }
  for (const auto & variable_name : robot_model->getVariableNames())
  {
    if (trajectory_variables.count(variable_name) == 0U)
    {
      metrics.message = "轨迹缺少 RobotModel 变量: " + variable_name;
      return metrics;
    }
  }

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  metrics.minimum_signed_offset = std::numeric_limits<double>::infinity();
  metrics.maximum_signed_offset = -std::numeric_limits<double>::infinity();
  metrics.minimum_bend_distance = std::numeric_limits<double>::infinity();
  metrics.maximum_bend_distance = 0.0;
  metrics.minimum_direction_observability =
    std::numeric_limits<double>::infinity();
  metrics.minimum_shoulder_wrist_distance =
    std::numeric_limits<double>::infinity();
  double previous_time = -std::numeric_limits<double>::infinity();
  int previous_unambiguous_side = 0;

  for (std::size_t point_index = 0;
    point_index < joint_trajectory.points.size(); ++point_index)
  {
    const auto & point = joint_trajectory.points[point_index];
    if (point.positions.size() != joint_trajectory.joint_names.size() ||
      !std::all_of(
        point.positions.begin(), point.positions.end(),
        [](double value) {return std::isfinite(value);}))
    {
      metrics.message = "轨迹点关节位置数量不一致或包含非有限数值";
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
    const Eigen::Vector3d shoulder = state.getGlobalLinkTransform(
      config.shoulder_link).translation();
    const Eigen::Vector3d elbow = state.getGlobalLinkTransform(
      config.elbow_link).translation();
    const Eigen::Vector3d wrist = state.getGlobalLinkTransform(
      config.wrist_link).translation();
    if (!shoulder.allFinite() || !elbow.allFinite() || !wrist.allFinite())
    {
      metrics.message = "肩、肘或腕 FK 位置不是有限数值";
      return metrics;
    }

    const Eigen::Vector3d shoulder_to_wrist = wrist - shoulder;
    const double shoulder_wrist_distance = shoulder_to_wrist.norm();
    if (!std::isfinite(shoulder_wrist_distance) ||
      shoulder_wrist_distance < config.minimum_shoulder_wrist_distance)
    {
      metrics.message = "肩腕距离过小，无法定义肩腕连线";
      return metrics;
    }
    const Eigen::Vector3d arm_axis =
      shoulder_to_wrist / shoulder_wrist_distance;
    const Eigen::Vector3d shoulder_to_elbow = elbow - shoulder;
    const Eigen::Vector3d elbow_offset = shoulder_to_elbow -
      shoulder_to_elbow.dot(arm_axis) * arm_axis;
    const double signed_offset = elbow_offset.dot(surface_normal);
    const double bend_distance = elbow_offset.norm();
    const double direction_observability =
      (surface_normal - surface_normal.dot(arm_axis) * arm_axis).norm();
    if (!std::isfinite(signed_offset) || !std::isfinite(bend_distance) ||
      !std::isfinite(direction_observability))
    {
      metrics.message = "肘部构型指标不是有限数值";
      return metrics;
    }

    if (point_index == 0U)
    {
      metrics.start_signed_offset = signed_offset;
    }
    metrics.end_signed_offset = signed_offset;
    if (signed_offset < metrics.minimum_signed_offset)
    {
      metrics.minimum_signed_offset = signed_offset;
      metrics.minimum_signed_offset_point_index = point_index;
      metrics.minimum_signed_offset_point_time = point_time;
    }
    metrics.maximum_signed_offset = std::max(
      metrics.maximum_signed_offset, signed_offset);
    if (bend_distance < metrics.minimum_bend_distance)
    {
      metrics.minimum_bend_distance = bend_distance;
      metrics.minimum_bend_point_index = point_index;
      metrics.minimum_bend_point_time = point_time;
    }
    metrics.maximum_bend_distance = std::max(
      metrics.maximum_bend_distance, bend_distance);
    if (direction_observability < metrics.minimum_direction_observability)
    {
      metrics.minimum_direction_observability = direction_observability;
      metrics.minimum_observability_point_index = point_index;
      metrics.minimum_observability_point_time = point_time;
    }
    metrics.minimum_shoulder_wrist_distance = std::min(
      metrics.minimum_shoulder_wrist_distance, shoulder_wrist_distance);

    int current_side = 0;
    if (signed_offset > config.side_tolerance)
    {
      current_side = 1;
    }
    else if (signed_offset < -config.side_tolerance)
    {
      current_side = -1;
    }
    else
    {
      ++metrics.ambiguous_side_sample_count;
    }
    if (direction_observability <= config.observability_tolerance)
    {
      ++metrics.direction_degenerate_sample_count;
    }
    if (current_side != 0)
    {
      if (previous_unambiguous_side != 0 &&
        current_side != previous_unambiguous_side)
      {
        ++metrics.side_change_count;
      }
      previous_unambiguous_side = current_side;
    }
  }

  metrics.sample_count = joint_trajectory.points.size();
  metrics.valid = true;
  metrics.message = "肩肘腕全轨迹构型诊断成功；当前仅记录，不执行肘上门禁";
  return metrics;
}

}  // namespace massage_motion
