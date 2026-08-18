#ifndef MASSAGE_MOTION__EXECUTION_TIMING_HPP_
#define MASSAGE_MOTION__EXECUTION_TIMING_HPP_

#include <optional>
#include <string>

#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace massage_motion
{

struct ExecutionTimingPolicy
{
  double margin{10.0};
  std::optional<double> timeout_override;
  bool allow_shorter_timeout_for_testing{false};
};

struct ExecutionTimingResult
{
  bool valid{false};
  std::string message;
  double expected_duration{0.0};
  double timeout{0.0};
  bool used_override{false};
};

ExecutionTimingResult calculate_execution_timing(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const ExecutionTimingPolicy & policy);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__EXECUTION_TIMING_HPP_
