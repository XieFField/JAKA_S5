#ifndef MASSAGE_MOTION__PROGRESSIVE_JOINT_CHECKPOINTS_HPP_
#define MASSAGE_MOTION__PROGRESSIVE_JOINT_CHECKPOINTS_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace massage_motion
{

struct ProgressiveJointCheckpointConfig
{
  std::vector<double> required_fractions{0.10, 0.25, 0.50, 1.00};
  double maximum_joint_step{0.30};
  std::size_t maximum_checkpoint_count{64U};
};

struct ProgressiveJointCheckpoint
{
  double fraction{0.0};
  bool required{false};
  std::vector<double> positions;
  std::size_t source_segment_index{0U};
  double source_time{0.0};
  double maximum_joint_step_from_previous{0.0};
};

struct ProgressiveJointCheckpointResult
{
  bool valid{false};
  std::string message;
  std::vector<std::string> joint_names;
  std::vector<double> start_positions;
  std::vector<ProgressiveJointCheckpoint> checkpoints;
  double joint_path_length{0.0};
  double maximum_generated_joint_step{0.0};
};

ProgressiveJointCheckpointResult generate_progressive_joint_checkpoints(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const ProgressiveJointCheckpointConfig & config = {});

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__PROGRESSIVE_JOINT_CHECKPOINTS_HPP_
