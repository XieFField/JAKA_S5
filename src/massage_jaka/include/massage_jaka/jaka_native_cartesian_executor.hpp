#ifndef MASSAGE_JAKA__JAKA_NATIVE_CARTESIAN_EXECUTOR_HPP_
#define MASSAGE_JAKA__JAKA_NATIVE_CARTESIAN_EXECUTOR_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "jaka_msgs/action/execute_cartesian_move.hpp"
#include "jaka_msgs/srv/get_rapid_rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_motion/trajectory_executor.hpp"

namespace massage_jaka
{

struct JakaNativeCartesianExecutorConfig
{
  std::string action_name{"/jaka_driver/execute_cartesian_move"};
  std::string rapid_rate_service{"/jaka_driver/get_rapid_rate"};
  std::string joint_state_topic{"/joint_states"};
  std::string controller_frame{"world"};
  double joint_state_timeout{1.0};
  double action_server_timeout{5.0};
  double result_grace_period{2.0};
  double maximum_start_error{0.002};
  double maximum_linear_speed_mm_s{100.0};
  double maximum_linear_acceleration_mm_s2{500.0};
  double orientation_speed_rad_s{0.50};
  double orientation_acceleration_rad_s2{1.0};
  double translation_tolerance_mm{1.0};
  double rotation_tolerance_rad{0.01};
};

class JakaNativeCartesianExecutor final :
  public massage_motion::ITrajectoryExecutor
{
public:
  using Action = jaka_msgs::action::ExecuteCartesianMove;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  JakaNativeCartesianExecutor(
    rclcpp::Node::SharedPtr node,
    JakaNativeCartesianExecutorConfig config = {});

  massage_motion::ExecutionResult execute(
    const massage_motion::ExecutionRequest & request) override;
  bool cancel() override;
  massage_motion::ExecutionStatus status() const override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  JakaNativeCartesianExecutorConfig config_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Client<jaka_msgs::srv::GetRapidRate>::SharedPtr rapid_rate_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  mutable std::mutex state_mutex_;
  sensor_msgs::msg::JointState latest_state_;
  std::chrono::steady_clock::time_point state_received_at_;
  mutable std::mutex goal_mutex_;
  GoalHandle::SharedPtr active_goal_;
  std::mutex execution_mutex_;
  std::atomic<massage_motion::ExecutionStatus> status_{
    massage_motion::ExecutionStatus::kIdle};
};

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__JAKA_NATIVE_CARTESIAN_EXECUTOR_HPP_
