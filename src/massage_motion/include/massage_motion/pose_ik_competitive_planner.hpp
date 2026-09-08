#ifndef MASSAGE_MOTION__POSE_IK_COMPETITIVE_PLANNER_HPP_
#define MASSAGE_MOTION__POSE_IK_COMPETITIVE_PLANNER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "moveit/robot_model/robot_model.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/motion_planner.hpp"
#include "massage_motion/pose_ik_candidate_generator.hpp"

namespace massage_motion
{

struct PoseIkCompetitivePlannerConfig
{
  std::string planning_group;
  std::string tip_link;
  std::string joint_state_topic{"/joint_states"};
  double joint_state_timeout{2.0};
  PoseIkCandidateGeneratorConfig ik;
  PlanCompetitionConfig competition;
};

using PoseIkGeneratorFunction = std::function<PoseIkCandidateReport(
    const sensor_msgs::msg::JointState &,
    const geometry_msgs::msg::PoseStamped &)>;

// Expands a PTP pose target into unique IK joint targets and selects the best
// resulting trajectory. All other requests are forwarded unchanged.
class PoseIkCompetitivePlanner final : public IMotionPlanner
{
public:
  PoseIkCompetitivePlanner(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<IMotionPlanner> planner,
    moveit::core::RobotModelConstPtr robot_model,
    PoseIkCompetitivePlannerConfig config);

  // The injectable generator keeps routing and candidate selection testable
  // without loading a process-wide MoveIt kinematics plugin.
  PoseIkCompetitivePlanner(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<IMotionPlanner> planner,
    PoseIkGeneratorFunction generator,
    PoseIkCompetitivePlannerConfig config);

  PlanResult plan(const MotionRequest & request) override;

private:
  sensor_msgs::msg::JointState fixed_start_joint_state(
    const MotionRequest & request, std::string * error) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::shared_ptr<IMotionPlanner> planner_;
  PoseIkCompetitivePlannerConfig config_;
  PoseIkGeneratorFunction generator_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::chrono::steady_clock::time_point latest_joint_state_received_{};
  std::mutex plan_mutex_;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__POSE_IK_COMPETITIVE_PLANNER_HPP_
