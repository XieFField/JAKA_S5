#ifndef MASSAGE_MOTION__ELBOW_POSTURE_GEOMETRY_HPP_
#define MASSAGE_MOTION__ELBOW_POSTURE_GEOMETRY_HPP_

#include <array>
#include <cstddef>
#include <string>

#include "moveit/robot_model/robot_model.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace massage_motion
{

struct ElbowPostureGeometryConfig
{
  std::string shoulder_link{"Link_02"};
  std::string elbow_link{"Link_03"};
  std::string wrist_link{"Link_04"};
  std::array<double, 3> surface_normal{0.0, 0.0, 1.0};
  double side_tolerance{1.0e-6};
  double observability_tolerance{1.0e-6};
  double minimum_shoulder_wrist_distance{1.0e-9};
};

struct ElbowPostureMetrics
{
  bool valid{false};
  std::string message;
  double start_signed_offset{0.0};
  double end_signed_offset{0.0};
  double minimum_signed_offset{0.0};
  double maximum_signed_offset{0.0};
  double minimum_bend_distance{0.0};
  double maximum_bend_distance{0.0};
  double minimum_direction_observability{0.0};
  double minimum_shoulder_wrist_distance{0.0};
  std::size_t minimum_signed_offset_point_index{0U};
  double minimum_signed_offset_point_time{0.0};
  std::size_t minimum_bend_point_index{0U};
  double minimum_bend_point_time{0.0};
  std::size_t minimum_observability_point_index{0U};
  double minimum_observability_point_time{0.0};
  std::size_t side_change_count{0U};
  std::size_t ambiguous_side_sample_count{0U};
  std::size_t direction_degenerate_sample_count{0U};
  std::size_t sample_count{0U};
};

ElbowPostureMetrics calculate_elbow_posture_metrics(
  const moveit::core::RobotModelConstPtr & robot_model,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const ElbowPostureGeometryConfig & config = {});

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__ELBOW_POSTURE_GEOMETRY_HPP_
