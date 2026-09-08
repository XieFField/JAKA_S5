#include "massage_motion/tool_orientation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>

#include "Eigen/Geometry"
#include "moveit/robot_state/robot_state.h"

namespace massage_motion
{
namespace
{

bool finite_vector(const std::array<double, 3> & value)
{
  return std::all_of(
    value.begin(), value.end(),
    [](double component) {return std::isfinite(component);});
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z) && std::isfinite(value.w);
}

Eigen::Vector3d vector(const std::array<double, 3> & value)
{
  return {value[0], value[1], value[2]};
}

std::array<double, 3> array(const Eigen::Vector3d & value)
{
  return {value.x(), value.y(), value.z()};
}

Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion & value)
{
  return {value.w, value.x, value.y, value.z};
}

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

double orientation_distance(
  const Eigen::Quaterniond & expected, const Eigen::Quaterniond & actual)
{
  return Eigen::AngleAxisd(expected.conjugate() * actual).angle();
}

bool validate_gate_config(
  const ToolAlignmentGateConfig & config, Eigen::Vector3d & desired,
  Eigen::Quaterniond & expected, std::string & message)
{
  if (!finite_vector(config.desired_tool_z_world) ||
    !finite_quaternion(config.expected_orientation) ||
    !std::isfinite(config.maximum_axis_error) ||
    config.maximum_axis_error < 0.0 || config.maximum_axis_error > M_PI)
  {
    message = "工具姿态门禁参数包含非有限数值或无效角度";
    return false;
  }
  desired = vector(config.desired_tool_z_world);
  const double desired_norm = desired.norm();
  expected = quaternion(config.expected_orientation);
  if (desired_norm <= 1.0e-12 || expected.squaredNorm() <= 1.0e-12)
  {
    message = "工具姿态门禁的期望轴或四元数长度为零";
    return false;
  }
  desired /= desired_norm;
  expected.normalize();
  return true;
}

void update_metrics(
  ToolAlignmentMetrics & metrics, const Eigen::Quaterniond & orientation,
  const Eigen::Vector3d & desired, const Eigen::Quaterniond & expected,
  std::size_t point_index, double point_time)
{
  const Eigen::Vector3d tool_z = orientation * Eigen::Vector3d::UnitZ();
  const double alignment = std::clamp(tool_z.dot(desired), -1.0, 1.0);
  const double axis_error = std::acos(alignment);
  const double drift = orientation_distance(expected, orientation);
  metrics.final_tool_z_world = array(tool_z);
  metrics.minimum_signed_alignment = std::min(
    metrics.minimum_signed_alignment, alignment);
  metrics.maximum_orientation_drift = std::max(
    metrics.maximum_orientation_drift, drift);
  if (axis_error >= metrics.maximum_axis_error)
  {
    metrics.maximum_axis_error = axis_error;
    metrics.worst_point_index = point_index;
    metrics.worst_point_time = point_time;
  }
  ++metrics.sample_count;
}

}  // namespace

ToolOrientationResult make_surface_aligned_tool_orientation(
  const ToolOrientationRequest & request)
{
  ToolOrientationResult result;
  if (!finite_vector(request.surface_normal_world) ||
    !finite_vector(request.tangent_direction_world) ||
    !std::isfinite(request.minimum_projected_tangent_norm) ||
    request.minimum_projected_tangent_norm <= 0.0)
  {
    result.message = "表面法向、切向或投影阈值无效";
    return result;
  }

  Eigen::Vector3d normal = vector(request.surface_normal_world);
  const double normal_norm = normal.norm();
  if (normal_norm <= 1.0e-12)
  {
    result.message = "表面法向量长度为零";
    return result;
  }
  normal /= normal_norm;
  const Eigen::Vector3d tool_z = -normal;
  const Eigen::Vector3d tangent = vector(request.tangent_direction_world);
  Eigen::Vector3d tool_x = tangent - tangent.dot(tool_z) * tool_z;
  const double projected_norm = tool_x.norm();
  if (!std::isfinite(projected_norm) ||
    projected_norm < request.minimum_projected_tangent_norm)
  {
    result.message = "切向方向与表面法向平行或投影长度过小";
    return result;
  }
  tool_x /= projected_norm;
  Eigen::Vector3d tool_y = tool_z.cross(tool_x);
  tool_y.normalize();

  Eigen::Matrix3d rotation;
  rotation.col(0) = tool_x;
  rotation.col(1) = tool_y;
  rotation.col(2) = tool_z;
  result.determinant = rotation.determinant();
  if (!rotation.allFinite() || std::abs(result.determinant - 1.0) > 1.0e-9)
  {
    result.message = "构造的工具坐标系不是有限右手正交坐标系";
    return result;
  }

  Eigen::Quaterniond orientation(rotation);
  orientation.normalize();
  result.orientation.x = orientation.x();
  result.orientation.y = orientation.y();
  result.orientation.z = orientation.z();
  result.orientation.w = orientation.w();
  result.surface_normal_world = array(normal);
  result.tool_x_world = array(tool_x);
  result.tool_y_world = array(tool_y);
  result.tool_z_world = array(tool_z);
  result.valid = true;
  result.message = "表面法向和切向已构造确定性工具姿态";
  return result;
}

ToolAlignmentMetrics evaluate_tool_alignment(
  const geometry_msgs::msg::Quaternion & orientation,
  const ToolAlignmentGateConfig & config)
{
  ToolAlignmentMetrics metrics;
  Eigen::Vector3d desired;
  Eigen::Quaterniond expected;
  if (!validate_gate_config(config, desired, expected, metrics.message) ||
    !finite_quaternion(orientation))
  {
    if (metrics.message.empty())
    {
      metrics.message = "待校验工具四元数包含非有限数值";
    }
    return metrics;
  }
  Eigen::Quaterniond actual = quaternion(orientation);
  if (actual.squaredNorm() <= 1.0e-12)
  {
    metrics.message = "待校验工具四元数长度为零";
    return metrics;
  }
  actual.normalize();
  metrics.minimum_signed_alignment = 1.0;
  metrics.desired_tool_z_world = array(desired);
  update_metrics(metrics, actual, desired, expected, 0U, 0.0);
  metrics.valid = true;
  metrics.accepted = metrics.maximum_axis_error <= config.maximum_axis_error;
  metrics.message = metrics.accepted ?
    "工具 Z 轴方向通过姿态门禁" : "工具 Z 轴方向超过姿态门禁";
  return metrics;
}

ToolAlignmentMetrics evaluate_tool_alignment_trajectory(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & tool_link,
  const ToolAlignmentGateConfig & config)
{
  ToolAlignmentMetrics metrics;
  Eigen::Vector3d desired;
  Eigen::Quaterniond expected;
  if (!validate_gate_config(config, desired, expected, metrics.message))
  {
    return metrics;
  }
  if (!robot_model)
  {
    metrics.message = "RobotModel 为空";
    return metrics;
  }
  if (tool_link.empty() || !robot_model->hasLinkModel(tool_link))
  {
    metrics.message = "RobotModel 中不存在工具连杆: " + tool_link;
    return metrics;
  }
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty())
  {
    metrics.message = "轨迹关节名或轨迹点为空";
    return metrics;
  }
  std::unordered_set<std::string> names;
  const std::unordered_set<std::string> model_names(
    robot_model->getVariableNames().begin(), robot_model->getVariableNames().end());
  for (const auto & name : joint_trajectory.joint_names)
  {
    if (name.empty() || !names.insert(name).second || model_names.count(name) == 0U)
    {
      metrics.message = "轨迹包含空名称、重复名称或未知关节变量";
      return metrics;
    }
  }
  for (const auto & name : robot_model->getVariableNames())
  {
    if (names.count(name) == 0U)
    {
      metrics.message = "轨迹缺少 RobotModel 变量: " + name;
      return metrics;
    }
  }

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  metrics.minimum_signed_alignment = 1.0;
  metrics.desired_tool_z_world = array(desired);
  const std::size_t first_index = config.inspect_all_samples ?
    0U : joint_trajectory.points.size() - 1U;
  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < joint_trajectory.points.size(); ++index)
  {
    const auto & point = joint_trajectory.points[index];
    const double point_time = duration_seconds(point.time_from_start);
    if (point.positions.size() != joint_trajectory.joint_names.size() ||
      !std::all_of(
        point.positions.begin(), point.positions.end(),
        [](double value) {return std::isfinite(value);}) ||
      !std::isfinite(point_time) || point_time < 0.0 || point_time < previous_time)
    {
      metrics.message = "轨迹点关节位置或时间无效";
      return metrics;
    }
    previous_time = point_time;
    if (index < first_index)
    {
      continue;
    }
    state.setVariablePositions(joint_trajectory.joint_names, point.positions);
    state.updateLinkTransforms();
    Eigen::Quaterniond actual(state.getGlobalLinkTransform(tool_link).rotation());
    if (!actual.coeffs().allFinite() || actual.squaredNorm() <= 1.0e-12)
    {
      metrics.message = "轨迹工具 FK 姿态不是有限四元数";
      return metrics;
    }
    actual.normalize();
    update_metrics(metrics, actual, desired, expected, index, point_time);
  }
  metrics.valid = metrics.sample_count > 0U;
  metrics.accepted = metrics.valid &&
    metrics.maximum_axis_error <= config.maximum_axis_error;
  if (!metrics.valid)
  {
    metrics.message = "轨迹中没有可校验的工具姿态样本";
  }
  else if (metrics.accepted)
  {
    metrics.message = config.inspect_all_samples ?
      "整条轨迹工具 Z 轴方向通过姿态门禁" :
      "轨迹终点工具 Z 轴方向通过姿态门禁";
  }
  else
  {
    std::ostringstream message;
    message << "工具 Z 轴最大角误差 " << metrics.maximum_axis_error
            << " rad 超过阈值 " << config.maximum_axis_error << " rad";
    metrics.message = message.str();
  }
  return metrics;
}

}  // namespace massage_motion
