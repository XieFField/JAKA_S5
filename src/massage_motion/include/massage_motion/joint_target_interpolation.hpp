#ifndef MASSAGE_MOTION__JOINT_TARGET_INTERPOLATION_HPP_
#define MASSAGE_MOTION__JOINT_TARGET_INTERPOLATION_HPP_

#include <string>
#include <vector>

namespace massage_motion
{

struct JointTargetInterpolationResult
{
  bool valid{false};
  std::string message;
  std::vector<double> target_positions;
  std::vector<double> joint_travels;
  double maximum_joint_travel{0.0};
};

JointTargetInterpolationResult interpolate_joint_target(
  const std::vector<double> & start_positions,
  const std::vector<double> & goal_positions,
  double ratio,
  double maximum_joint_travel);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__JOINT_TARGET_INTERPOLATION_HPP_
