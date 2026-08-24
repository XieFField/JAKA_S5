#include "massage_motion/cartesian_path_verification.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>

#include "Eigen/Geometry"
#include "moveit/robot_state/robot_state.h"

namespace massage_motion
{

namespace
{

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const double values[] = {
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w};
  if (!std::all_of(
      std::begin(values), std::end(values),
      [](double value) {return std::isfinite(value);}))
  {
    return false;
  }
  const Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  return orientation.squaredNorm() > 1.0e-12;
}

Eigen::Quaterniond normalized_orientation(
  const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  orientation.normalize();
  return orientation;
}

Eigen::Vector3d position(const geometry_msgs::msg::Pose & pose)
{
  return {pose.position.x, pose.position.y, pose.position.z};
}

double orientation_distance(
  const Eigen::Quaterniond & expected,
  const Eigen::Quaterniond & actual)
{
  return Eigen::AngleAxisd(expected.conjugate() * actual).angle();
}

bool valid_trace_config(const CartesianLineTraceConfig & config)
{
  const double values[] = {
    config.maximum_endpoint_position_error,
    config.maximum_endpoint_orientation_error,
    config.maximum_transverse_error,
    config.maximum_height_error,
    config.maximum_orientation_error,
    config.maximum_longitudinal_overshoot};
  return std::all_of(
    std::begin(values), std::end(values),
    [](double value) {return std::isfinite(value) && value >= 0.0;});
}

}  // namespace

LinkPoseResult calculate_link_pose(
  const moveit::core::RobotModelConstPtr & robot_model,
  const sensor_msgs::msg::JointState & joint_state,
  const std::string & link_name)
{
  LinkPoseResult result;
  if (!robot_model)
  {
    result.message = "RobotModel 为空";
    return result;
  }
  if (link_name.empty() || !robot_model->hasLinkModel(link_name))
  {
    result.message = "RobotModel 中不存在目标连杆: " + link_name;
    return result;
  }
  if (joint_state.name.empty() ||
    joint_state.name.size() != joint_state.position.size())
  {
    result.message = "关节状态名称为空或名称与位置数量不一致";
    return result;
  }

  const auto & model_names = robot_model->getVariableNames();
  std::unordered_map<std::string, double> positions;
  positions.reserve(joint_state.name.size());
  for (std::size_t index = 0; index < joint_state.name.size(); ++index)
  {
    if (joint_state.name[index].empty() ||
      !std::isfinite(joint_state.position[index]) ||
      !positions.emplace(
        joint_state.name[index], joint_state.position[index]).second)
    {
      result.message = "关节状态包含空名称、重复名称或非有限位置";
      return result;
    }
  }

  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  for (const auto & name : model_names)
  {
    const auto iterator = positions.find(name);
    if (iterator == positions.end())
    {
      result.message = "关节状态缺少 RobotModel 变量: " + name;
      return result;
    }
    state.setVariablePosition(name, iterator->second);
  }
  state.updateLinkTransforms();
  const auto & transform = state.getGlobalLinkTransform(link_name);
  if (!transform.matrix().allFinite())
  {
    result.message = "连杆 FK 位姿包含非有限数值";
    return result;
  }
  result.pose.position.x = transform.translation().x();
  result.pose.position.y = transform.translation().y();
  result.pose.position.z = transform.translation().z();
  Eigen::Quaterniond orientation(transform.rotation());
  orientation.normalize();
  result.pose.orientation.x = orientation.x();
  result.pose.orientation.y = orientation.y();
  result.pose.orientation.z = orientation.z();
  result.pose.orientation.w = orientation.w();
  result.valid = true;
  result.message = "连杆 FK 位姿计算成功";
  return result;
}

PoseErrorMetrics calculate_pose_error(
  const geometry_msgs::msg::Pose & expected,
  const geometry_msgs::msg::Pose & actual)
{
  PoseErrorMetrics result;
  if (!finite_pose(expected) || !finite_pose(actual))
  {
    result.message = "期望或实际位姿包含非有限数值或零四元数";
    return result;
  }
  result.translation = (position(actual) - position(expected)).norm();
  result.rotation = orientation_distance(
    normalized_orientation(expected), normalized_orientation(actual));
  result.valid = std::isfinite(result.translation) &&
    std::isfinite(result.rotation);
  result.message = result.valid ? "位姿误差计算成功" : "位姿误差不是有限数值";
  return result;
}

CartesianLineTraceMetrics evaluate_cartesian_line_trace(
  const std::vector<geometry_msgs::msg::Pose> & samples,
  const geometry_msgs::msg::Pose & expected_start,
  const geometry_msgs::msg::Pose & expected_end,
  const CartesianLineTraceConfig & config)
{
  CartesianLineTraceMetrics result;
  result.sample_count = samples.size();
  if (samples.size() < 2U || !finite_pose(expected_start) ||
    !finite_pose(expected_end) || !valid_trace_config(config))
  {
    result.message = "笛卡尔轨迹样本、期望位姿或门限参数无效";
    return result;
  }
  if (!std::all_of(samples.begin(), samples.end(), finite_pose))
  {
    result.message = "笛卡尔轨迹样本包含非有限位姿或零四元数";
    return result;
  }

  const Eigen::Vector3d start = position(expected_start);
  const Eigen::Vector3d end = position(expected_end);
  const Eigen::Vector3d delta = end - start;
  result.expected_length = delta.norm();
  if (!std::isfinite(result.expected_length) || result.expected_length <= 1.0e-9)
  {
    result.message = "期望直线长度必须为有限正数";
    return result;
  }
  const Eigen::Vector3d direction = delta / result.expected_length;
  const Eigen::Quaterniond expected_orientation =
    normalized_orientation(expected_start);
  result.minimum_directed_progress = std::numeric_limits<double>::infinity();
  result.maximum_directed_progress = -std::numeric_limits<double>::infinity();

  for (const auto & sample : samples)
  {
    const Eigen::Vector3d offset = position(sample) - start;
    const double progress = offset.dot(direction);
    const Eigen::Vector3d transverse = offset - progress * direction;
    const double clamped_fraction = std::clamp(
      progress / result.expected_length, 0.0, 1.0);
    const double expected_z = start.z() + clamped_fraction * delta.z();
    result.minimum_directed_progress = std::min(
      result.minimum_directed_progress, progress);
    result.maximum_directed_progress = std::max(
      result.maximum_directed_progress, progress);
    result.maximum_transverse_error = std::max(
      result.maximum_transverse_error, transverse.norm());
    result.maximum_height_error = std::max(
      result.maximum_height_error,
      std::abs(sample.position.z - expected_z));
    result.maximum_orientation_error = std::max(
      result.maximum_orientation_error,
      orientation_distance(
        expected_orientation, normalized_orientation(sample)));
  }

  const auto endpoint_error = calculate_pose_error(expected_end, samples.back());
  if (!endpoint_error.valid)
  {
    result.message = "无法计算笛卡尔轨迹终点误差: " + endpoint_error.message;
    return result;
  }
  result.final_directed_progress =
    (position(samples.back()) - start).dot(direction);
  result.endpoint_position_error = endpoint_error.translation;
  result.endpoint_orientation_error = endpoint_error.rotation;
  result.valid = true;

  const bool accepted =
    result.endpoint_position_error <= config.maximum_endpoint_position_error &&
    result.endpoint_orientation_error <=
    config.maximum_endpoint_orientation_error &&
    result.maximum_transverse_error <= config.maximum_transverse_error &&
    result.maximum_height_error <= config.maximum_height_error &&
    result.maximum_orientation_error <= config.maximum_orientation_error &&
    result.minimum_directed_progress >=
    -config.maximum_longitudinal_overshoot &&
    result.maximum_directed_progress <=
    result.expected_length + config.maximum_longitudinal_overshoot;
  result.accepted = accepted;
  if (accepted)
  {
    result.message = "世界系直线方向、长度、高度、姿态和终点均通过";
    return result;
  }

  std::ostringstream message;
  message << "笛卡尔直线验收失败: endpoint_position="
          << result.endpoint_position_error
          << ", endpoint_orientation=" << result.endpoint_orientation_error
          << ", transverse=" << result.maximum_transverse_error
          << ", height=" << result.maximum_height_error
          << ", orientation=" << result.maximum_orientation_error
          << ", progress=[" << result.minimum_directed_progress << ", "
          << result.maximum_directed_progress << "]";
  result.message = message.str();
  return result;
}

}  // namespace massage_motion
