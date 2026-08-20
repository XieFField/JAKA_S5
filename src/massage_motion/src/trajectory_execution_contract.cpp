#include "massage_motion/trajectory_execution_contract.hpp"

#include <cmath>
#include <sstream>

namespace massage_motion
{

TrajectoryExecutionContractResult validate_trajectory_execution_contract(
  double controller_goal_tolerance,
  double controller_endpoint_margin,
  double endpoint_tolerance,
  double execution_timeout_margin)
{
  if (!std::isfinite(controller_goal_tolerance) ||
    controller_goal_tolerance <= 0.0 ||
    !std::isfinite(controller_endpoint_margin) ||
    controller_endpoint_margin <= 0.0 ||
    !std::isfinite(endpoint_tolerance) || endpoint_tolerance <= 0.0 ||
    !std::isfinite(execution_timeout_margin) || execution_timeout_margin < 0.0)
  {
    return {false, "轨迹执行契约参数必须为有限正数，外层余量允许为零"};
  }

  if (controller_goal_tolerance > endpoint_tolerance)
  {
    std::ostringstream message;
    message << "控制器终点容差 " << controller_goal_tolerance
            << " rad 大于任务终点容差 " << endpoint_tolerance << " rad";
    return {false, message.str()};
  }

  if (controller_endpoint_margin < execution_timeout_margin)
  {
    std::ostringstream message;
    message << "控制器终点收敛余量 " << controller_endpoint_margin
            << " s 小于任务执行余量 " << execution_timeout_margin << " s";
    return {false, message.str()};
  }

  return {true, "轨迹执行契约一致"};
}

}  // namespace massage_motion
