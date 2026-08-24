#ifndef MASSAGE_MOTION__LINK_TRAJECTORY_GEOMETRY_HPP_
#define MASSAGE_MOTION__LINK_TRAJECTORY_GEOMETRY_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "moveit/robot_model/robot_model.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace massage_motion
{

struct LinkHeightMetrics
{
  bool valid{false};
  std::string message;
  double start_z{0.0};
  double end_z{0.0};
  double minimum_z{0.0};
  double maximum_z{0.0};
  double maximum_drop_below_start{0.0};
  std::size_t minimum_point_index{0U};
  double minimum_point_time{0.0};
};

struct LinkHeightGateConfig
{
  bool enabled{true};
  double minimum_z{0.0};
  double reference_z{std::numeric_limits<double>::quiet_NaN()};
  double maximum_drop_below_reference{0.20};
};

struct LinkHeightGateResult
{
  bool valid{false};
  bool accepted{false};
  std::string message;
  double minimum_allowed_z{0.0};
  double drop_below_reference{0.0};
};

struct LinkHeightAtJointTargetResult
{
  bool valid{false};
  std::string message;
  double z{0.0};
};

LinkHeightMetrics calculate_link_height_metrics(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & link_name);

LinkHeightAtJointTargetResult calculate_link_height_at_joint_target(
  const moveit::core::RobotModelConstPtr & robot_model,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  const std::string & link_name);

LinkHeightGateResult evaluate_link_height_gate(
  const LinkHeightMetrics & metrics,
  const LinkHeightGateConfig & config);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__LINK_TRAJECTORY_GEOMETRY_HPP_
