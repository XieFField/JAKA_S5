#ifndef MASSAGE_MOTION__NATIVE_PTP_CONTRACT_HPP_
#define MASSAGE_MOTION__NATIVE_PTP_CONTRACT_HPP_

#include <string>
#include <vector>

#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace massage_motion
{

struct NativePtpConfig
{
  std::vector<std::string> joint_names{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};
  double maximum_start_error{0.002};
  double maximum_path_deviation{0.002};
  double maximum_speed{0.20};
  double maximum_acceleration{0.50};
  double endpoint_tolerance{0.002};
  // JAKA joint_move includes controller-side smoothing that is not represented
  // by the ideal trapezoidal estimate used here.
  double duration_safety_factor{3.0};
  double timeout_margin{15.0};
};

struct NativePtpCommand
{
  bool valid{false};
  bool already_at_target{false};
  std::vector<double> target_positions;
  double speed{0.0};
  double acceleration{0.0};
  double rapid_rate{1.0};
  double effective_speed{0.0};
  double estimated_duration{0.0};
  double effective_timeout{0.0};
  double maximum_start_error{0.0};
  double maximum_path_deviation{0.0};
  std::string message;
};

NativePtpCommand make_native_ptp_command(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const sensor_msgs::msg::JointState & current_state,
  double requested_timeout,
  const NativePtpConfig & config = NativePtpConfig{},
  double rapid_rate = 1.0);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__NATIVE_PTP_CONTRACT_HPP_
