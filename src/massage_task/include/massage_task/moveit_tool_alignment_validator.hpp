#ifndef MASSAGE_TASK__MOVEIT_TOOL_ALIGNMENT_VALIDATOR_HPP_
#define MASSAGE_TASK__MOVEIT_TOOL_ALIGNMENT_VALIDATOR_HPP_

#include <memory>
#include <string>

#include "moveit/robot_model/robot_model.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "massage_task/massage_task_state_machine.hpp"

namespace massage_task
{

class MoveItToolAlignmentValidator final :
  public IToolAlignmentValidator,
  public IReturnTrajectorySafetyValidator
{
public:
  MoveItToolAlignmentValidator(
    rclcpp::Node::SharedPtr node,
    moveit::core::RobotModelConstPtr robot_model,
    std::string tool_link = "massage_tool_tip",
    double current_pose_timeout = 1.0);

  massage_motion::ToolAlignmentMetrics validate_trajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const massage_motion::ToolAlignmentGateConfig & config) override;

  massage_motion::ToolAlignmentMetrics validate_current(
    const std::string & reference_frame,
    const massage_motion::ToolAlignmentGateConfig & config) override;

  ReturnTrajectorySafetyResult validate(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const ReturnTrajectorySafetyConfig & config) override;

private:
  void log_metrics(
    const std::string & scope,
    const massage_motion::ToolAlignmentMetrics & metrics) const;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string tool_link_;
  double current_pose_timeout_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace massage_task

#endif  // MASSAGE_TASK__MOVEIT_TOOL_ALIGNMENT_VALIDATOR_HPP_
