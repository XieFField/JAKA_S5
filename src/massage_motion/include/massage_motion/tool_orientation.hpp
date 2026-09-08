#ifndef MASSAGE_MOTION__TOOL_ORIENTATION_HPP_
#define MASSAGE_MOTION__TOOL_ORIENTATION_HPP_

#include <array>
#include <cstddef>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/robot_model/robot_model.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace massage_motion
{

struct ToolOrientationRequest
{
  // The surface normal points from the patient toward free space.
  std::array<double, 3> surface_normal_world{0.0, 0.0, 1.0};
  // This only fixes rotation about the symmetric massage-head axis.
  std::array<double, 3> tangent_direction_world{0.0, 1.0, 0.0};
  double minimum_projected_tangent_norm{1.0e-6};
};

struct ToolOrientationResult
{
  bool valid{false};
  std::string message;
  geometry_msgs::msg::Quaternion orientation;
  std::array<double, 3> surface_normal_world{};
  std::array<double, 3> tool_x_world{};
  std::array<double, 3> tool_y_world{};
  std::array<double, 3> tool_z_world{};
  double determinant{0.0};
};

struct ToolAlignmentGateConfig
{
  std::array<double, 3> desired_tool_z_world{0.0, 0.0, -1.0};
  geometry_msgs::msg::Quaternion expected_orientation;
  double maximum_axis_error{0.0523598775598299};
  bool inspect_all_samples{true};
};

struct ToolAlignmentMetrics
{
  bool valid{false};
  bool accepted{false};
  std::string message;
  std::size_t sample_count{0U};
  std::array<double, 3> final_tool_z_world{};
  std::array<double, 3> desired_tool_z_world{};
  double minimum_signed_alignment{-1.0};
  double maximum_axis_error{0.0};
  double maximum_orientation_drift{0.0};
  std::size_t worst_point_index{0U};
  double worst_point_time{0.0};
};

ToolOrientationResult make_surface_aligned_tool_orientation(
  const ToolOrientationRequest & request);

ToolAlignmentMetrics evaluate_tool_alignment(
  const geometry_msgs::msg::Quaternion & orientation,
  const ToolAlignmentGateConfig & config);

ToolAlignmentMetrics evaluate_tool_alignment_trajectory(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & tool_link,
  const ToolAlignmentGateConfig & config);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__TOOL_ORIENTATION_HPP_
