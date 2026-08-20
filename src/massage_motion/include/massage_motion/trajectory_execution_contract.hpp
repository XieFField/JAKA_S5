#ifndef MASSAGE_MOTION__TRAJECTORY_EXECUTION_CONTRACT_HPP_
#define MASSAGE_MOTION__TRAJECTORY_EXECUTION_CONTRACT_HPP_

#include <string>

namespace massage_motion
{

struct TrajectoryExecutionContractResult
{
  bool valid{false};
  std::string message;
};

TrajectoryExecutionContractResult validate_trajectory_execution_contract(
  double controller_goal_tolerance,
  double controller_endpoint_margin,
  double endpoint_tolerance,
  double execution_timeout_margin);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__TRAJECTORY_EXECUTION_CONTRACT_HPP_
