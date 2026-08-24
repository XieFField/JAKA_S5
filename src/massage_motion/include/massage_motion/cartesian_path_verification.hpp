#ifndef MASSAGE_MOTION__CARTESIAN_PATH_VERIFICATION_HPP_
#define MASSAGE_MOTION__CARTESIAN_PATH_VERIFICATION_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/robot_model/robot_model.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace massage_motion
{

struct LinkPoseResult
{
  bool valid{false};
  std::string message;
  geometry_msgs::msg::Pose pose;
};

struct PoseErrorMetrics
{
  bool valid{false};
  std::string message;
  double translation{0.0};
  double rotation{0.0};
};

struct CartesianLineTraceConfig
{
  double maximum_endpoint_position_error{0.005};
  double maximum_endpoint_orientation_error{0.03};
  double maximum_transverse_error{0.005};
  double maximum_height_error{0.003};
  double maximum_orientation_error{0.03};
  double maximum_longitudinal_overshoot{0.005};
};

struct CartesianLineTraceMetrics
{
  bool valid{false};
  bool accepted{false};
  std::string message;
  std::size_t sample_count{0U};
  double expected_length{0.0};
  double final_directed_progress{0.0};
  double minimum_directed_progress{0.0};
  double maximum_directed_progress{0.0};
  double endpoint_position_error{0.0};
  double endpoint_orientation_error{0.0};
  double maximum_transverse_error{0.0};
  double maximum_height_error{0.0};
  double maximum_orientation_error{0.0};
};

LinkPoseResult calculate_link_pose(
  const moveit::core::RobotModelConstPtr & robot_model,
  const sensor_msgs::msg::JointState & joint_state,
  const std::string & link_name);

PoseErrorMetrics calculate_pose_error(
  const geometry_msgs::msg::Pose & expected,
  const geometry_msgs::msg::Pose & actual);

CartesianLineTraceMetrics evaluate_cartesian_line_trace(
  const std::vector<geometry_msgs::msg::Pose> & samples,
  const geometry_msgs::msg::Pose & expected_start,
  const geometry_msgs::msg::Pose & expected_end,
  const CartesianLineTraceConfig & config = {});

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__CARTESIAN_PATH_VERIFICATION_HPP_
