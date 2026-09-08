#ifndef MASSAGE_TASK__MOVEIT_TECHNIQUE_TRAJECTORY_PLANNER_HPP_
#define MASSAGE_TASK__MOVEIT_TECHNIQUE_TRAJECTORY_PLANNER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "massage_motion/motion_planner.hpp"
#include "massage_task/massage_task_state_machine.hpp"

namespace massage_task
{

// Converts a sampled push/press/knead path into continuous Pilz LIN/CIRC
// segments. This component is shared by simulation and real task assembly.
class MoveItTechniqueTrajectoryPlanner final :
  public ITechniqueTrajectoryPlanner
{
public:
  MoveItTechniqueTrajectoryPlanner(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<massage_motion::IMotionPlanner> planner,
    double velocity_scale, double acceleration_scale, double timeout,
    std::string tool_frame = "massage_tool_tip");

  massage_motion::PlanResult plan(
    const massage_motion::TechniquePath & path,
    const moveit_msgs::msg::RobotState & start_state,
    bool align_to_current_tcp = true) override;

private:
  rclcpp::Logger logger_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<massage_motion::IMotionPlanner> planner_;
  double velocity_scale_;
  double acceleration_scale_;
  double timeout_;
  std::string tool_frame_;
};

}  // namespace massage_task

#endif  // MASSAGE_TASK__MOVEIT_TECHNIQUE_TRAJECTORY_PLANNER_HPP_
