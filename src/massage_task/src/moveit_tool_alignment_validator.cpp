#include "massage_task/moveit_tool_alignment_validator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include "massage_motion/link_trajectory_geometry.hpp"
#include "moveit/robot_state/robot_state.h"
#include "tf2/exceptions.h"

namespace massage_task
{

namespace
{

constexpr double kRadiansToDegrees = 180.0 / M_PI;

}  // namespace

MoveItToolAlignmentValidator::MoveItToolAlignmentValidator(
  rclcpp::Node::SharedPtr node,
  moveit::core::RobotModelConstPtr robot_model,
  std::string tool_link,
  double current_pose_timeout)
: logger_(node ? node->get_logger() : rclcpp::get_logger("tool_alignment")),
  clock_(node ? node->get_clock() : nullptr),
  robot_model_(std::move(robot_model)), tool_link_(std::move(tool_link)),
  current_pose_timeout_(current_pose_timeout),
  tf_buffer_(node ? std::make_unique<tf2_ros::Buffer>(node->get_clock()) : nullptr),
  tf_listener_(node ? std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_, node, false) : nullptr)
{
  if (!node || !robot_model_ || tool_link_.empty() ||
    !robot_model_->hasLinkModel(tool_link_) ||
    !std::isfinite(current_pose_timeout_) || current_pose_timeout_ <= 0.0)
  {
    throw std::invalid_argument("MoveIt 工具姿态校验器依赖或配置无效");
  }
}

massage_motion::ToolAlignmentMetrics
MoveItToolAlignmentValidator::validate_trajectory(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const massage_motion::ToolAlignmentGateConfig & config)
{
  auto metrics = massage_motion::evaluate_tool_alignment_trajectory(
    robot_model_, trajectory, tool_link_, config);
  log_metrics(
    config.inspect_all_samples ? "trajectory" : "endpoint", metrics);
  return metrics;
}

massage_motion::ToolAlignmentMetrics
MoveItToolAlignmentValidator::validate_current(
  const std::string & reference_frame,
  const massage_motion::ToolAlignmentGateConfig & config)
{
  massage_motion::ToolAlignmentMetrics metrics;
  if (reference_frame.empty())
  {
    metrics.message = "当前工具姿态校验缺少参考坐标系";
    log_metrics("current", metrics);
    return metrics;
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(current_pose_timeout_);
  std::string last_error = "TF 尚未可用";
  do
  {
    try
    {
      const auto transform = tf_buffer_->lookupTransform(
        reference_frame, tool_link_, tf2::TimePointZero);
      metrics = massage_motion::evaluate_tool_alignment(
        transform.transform.rotation, config);
      log_metrics("current", metrics);
      return metrics;
    }
    catch (const tf2::TransformException & exception)
    {
      last_error = exception.what();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);

  metrics.message = "等待当前工具 TF 超时: " + last_error;
  log_metrics("current", metrics);
  return metrics;
}

ReturnTrajectorySafetyResult MoveItToolAlignmentValidator::validate(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const ReturnTrajectorySafetyConfig & config)
{
  ReturnTrajectorySafetyResult result;
  const auto finite_vector = [](const std::array<double, 3> & values) {
      return std::all_of(
        values.begin(), values.end(),
        [](double value) {return std::isfinite(value);});
    };
  const double normal_norm = std::sqrt(
    config.surface_normal[0] * config.surface_normal[0] +
    config.surface_normal[1] * config.surface_normal[1] +
    config.surface_normal[2] * config.surface_normal[2]);
  const double tangent_norm = std::sqrt(
    config.surface_tangent[0] * config.surface_tangent[0] +
    config.surface_tangent[1] * config.surface_tangent[1] +
    config.surface_tangent[2] * config.surface_tangent[2]);
  const double normal_tangent_dot =
    config.surface_normal[0] * config.surface_tangent[0] +
    config.surface_normal[1] * config.surface_tangent[1] +
    config.surface_normal[2] * config.surface_tangent[2];
  if (!std::isfinite(config.minimum_tool_z) ||
    !std::isfinite(config.minimum_diagnostic_link_z) ||
    !finite_vector(config.surface_origin) ||
    !finite_vector(config.surface_normal) ||
    !finite_vector(config.surface_tangent) ||
    !std::isfinite(normal_norm) || normal_norm <= 1.0e-9 ||
    !std::isfinite(tangent_norm) || tangent_norm <= 1.0e-9 ||
    !std::isfinite(normal_tangent_dot) ||
    std::abs(normal_tangent_dot / (normal_norm * tangent_norm)) > 1.0e-6 ||
    !std::isfinite(config.protected_tangent_min) ||
    !std::isfinite(config.protected_tangent_max) ||
    config.protected_tangent_min > config.protected_tangent_max ||
    !std::isfinite(config.protected_lateral_half_width) ||
    config.protected_lateral_half_width <= 0.0 ||
    !std::isfinite(config.minimum_surface_clearance) ||
    config.minimum_surface_clearance < 0.0 ||
    config.tool_link.empty() || config.diagnostic_link.empty())
  {
    result.message = "回待机轨迹高度门禁配置无效";
    return result;
  }

  const auto tool_metrics = massage_motion::calculate_link_height_metrics(
    robot_model_, trajectory, config.tool_link);
  const auto diagnostic_metrics = massage_motion::calculate_link_height_metrics(
    robot_model_, trajectory, config.diagnostic_link);
  if (!tool_metrics.valid || !diagnostic_metrics.valid)
  {
    result.message = "回待机轨迹 FK 诊断失败: tool=" + tool_metrics.message +
      "; diagnostic=" + diagnostic_metrics.message;
    return result;
  }

  result.valid = true;
  result.minimum_tool_z = tool_metrics.minimum_z;
  result.minimum_diagnostic_link_z = diagnostic_metrics.minimum_z;
  result.minimum_surface_clearance = std::numeric_limits<double>::infinity();
  const std::array<double, 3> unit_normal{{
      config.surface_normal[0] / normal_norm,
      config.surface_normal[1] / normal_norm,
      config.surface_normal[2] / normal_norm}};
  const std::array<double, 3> unit_tangent{{
      config.surface_tangent[0] / tangent_norm,
      config.surface_tangent[1] / tangent_norm,
      config.surface_tangent[2] / tangent_norm}};
  const std::array<double, 3> unit_lateral{{
      unit_normal[1] * unit_tangent[2] - unit_normal[2] * unit_tangent[1],
      unit_normal[2] * unit_tangent[0] - unit_normal[0] * unit_tangent[2],
      unit_normal[0] * unit_tangent[1] - unit_normal[1] * unit_tangent[0]}};

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  const auto & joint_trajectory = trajectory.joint_trajectory;
  for (const auto & point : joint_trajectory.points)
  {
    state.setVariablePositions(joint_trajectory.joint_names, point.positions);
    state.updateLinkTransforms();
    const auto position = state.getGlobalLinkTransform(config.tool_link).translation();
    const std::array<double, 3> offset{{
        position.x() - config.surface_origin[0],
        position.y() - config.surface_origin[1],
        position.z() - config.surface_origin[2]}};
    const double tangent_coordinate =
      offset[0] * unit_tangent[0] + offset[1] * unit_tangent[1] +
      offset[2] * unit_tangent[2];
    const double lateral_coordinate =
      offset[0] * unit_lateral[0] + offset[1] * unit_lateral[1] +
      offset[2] * unit_lateral[2];
    if (tangent_coordinate >= config.protected_tangent_min - 1.0e-9 &&
      tangent_coordinate <= config.protected_tangent_max + 1.0e-9 &&
      std::abs(lateral_coordinate) <=
      config.protected_lateral_half_width + 1.0e-9)
    {
      const double clearance =
        offset[0] * unit_normal[0] + offset[1] * unit_normal[1] +
        offset[2] * unit_normal[2];
      result.minimum_surface_clearance = std::min(
        result.minimum_surface_clearance, clearance);
      ++result.protected_sample_count;
    }
  }

  constexpr double tolerance = 1.0e-9;
  result.accepted =
    result.minimum_tool_z + tolerance >= config.minimum_tool_z &&
    result.minimum_diagnostic_link_z + tolerance >=
    config.minimum_diagnostic_link_z &&
    result.minimum_surface_clearance + tolerance >=
    config.minimum_surface_clearance;
  result.message = result.accepted ?
    "TCP 在患者区域内保持于表面上方，且全局高度门禁通过" :
    "TCP 患者区域净空或全局高度未通过门禁";

  if (result.protected_sample_count == 0U)
  {
    RCLCPP_INFO(
      logger_,
      "RETURN HEIGHT GATE: %s: tool=%s min_z=%.6f m limit=%.6f m, "
      "diagnostic=%s min_z=%.6f m limit=%.6f m, "
      "patient_clearance=n/a limit=%.6f m protected_samples=0/%zu",
      result.accepted ? "PASS" : "REJECTED", config.tool_link.c_str(),
      result.minimum_tool_z, config.minimum_tool_z,
      config.diagnostic_link.c_str(), result.minimum_diagnostic_link_z,
      config.minimum_diagnostic_link_z, config.minimum_surface_clearance,
      trajectory.joint_trajectory.points.size());
  }
  else
  {
    RCLCPP_INFO(
      logger_,
      "RETURN HEIGHT GATE: %s: tool=%s min_z=%.6f m limit=%.6f m, "
      "diagnostic=%s min_z=%.6f m limit=%.6f m, "
      "patient_clearance=%.6f m limit=%.6f m protected_samples=%zu/%zu",
      result.accepted ? "PASS" : "REJECTED", config.tool_link.c_str(),
      result.minimum_tool_z, config.minimum_tool_z,
      config.diagnostic_link.c_str(), result.minimum_diagnostic_link_z,
      config.minimum_diagnostic_link_z, result.minimum_surface_clearance,
      config.minimum_surface_clearance, result.protected_sample_count,
      trajectory.joint_trajectory.points.size());
  }
  return result;
}

void MoveItToolAlignmentValidator::log_metrics(
  const std::string & scope,
  const massage_motion::ToolAlignmentMetrics & metrics) const
{
  if (!metrics.valid)
  {
    RCLCPP_ERROR(
      logger_, "TOOL ALIGNMENT GATE [%s]: INVALID: %s",
      scope.c_str(), metrics.message.c_str());
    return;
  }
  const auto & tool = metrics.final_tool_z_world;
  const auto & desired = metrics.desired_tool_z_world;
  if (metrics.accepted)
  {
    RCLCPP_INFO(
      logger_, "TOOL ALIGNMENT GATE [%s]: PASS: tool_z_world=[%.6f %.6f %.6f], "
      "desired_tool_z_world=[%.6f %.6f %.6f], signed_alignment=%.9f, "
      "maximum_axis_error_deg=%.6f, orientation_drift_deg=%.6f, "
      "samples=%zu, worst_point=%zu, worst_time=%.6f s",
      scope.c_str(), tool[0], tool[1], tool[2], desired[0], desired[1],
      desired[2], metrics.minimum_signed_alignment,
      metrics.maximum_axis_error * kRadiansToDegrees,
      metrics.maximum_orientation_drift * kRadiansToDegrees,
      metrics.sample_count, metrics.worst_point_index, metrics.worst_point_time);
  }
  else
  {
    RCLCPP_ERROR(
      logger_, "TOOL ALIGNMENT GATE [%s]: FAIL: tool_z_world=[%.6f %.6f %.6f], "
      "desired_tool_z_world=[%.6f %.6f %.6f], signed_alignment=%.9f, "
      "maximum_axis_error_deg=%.6f, orientation_drift_deg=%.6f, "
      "samples=%zu, worst_point=%zu, worst_time=%.6f s, message=%s",
      scope.c_str(), tool[0], tool[1], tool[2], desired[0], desired[1],
      desired[2], metrics.minimum_signed_alignment,
      metrics.maximum_axis_error * kRadiansToDegrees,
      metrics.maximum_orientation_drift * kRadiansToDegrees,
      metrics.sample_count, metrics.worst_point_index, metrics.worst_point_time,
      metrics.message.c_str());
  }
}

}  // namespace massage_task
