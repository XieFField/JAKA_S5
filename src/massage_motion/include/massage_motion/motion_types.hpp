#ifndef MASSAGE_MOTION__MOTION_TYPES_HPP_
#define MASSAGE_MOTION__MOTION_TYPES_HPP_

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace massage_motion
{

enum class MotionType : std::uint8_t
{
  kPtp = 0,
  kLin,
  kCirc,
};

enum class MotionError : std::int32_t
{
  kNone = 0,
  kInvalidRequest,
  kTargetTypeMismatch,
  kPlanningFailed,
  kEmptyTrajectory,
  kUnsupportedMotion,
};

struct JointTarget
{
  std::vector<double> positions;
};

struct PoseTarget
{
  geometry_msgs::msg::PoseStamped pose;
};

struct CircularTarget
{
  geometry_msgs::msg::PoseStamped interim_pose;
  geometry_msgs::msg::PoseStamped goal_pose;
};

using MotionTarget = std::variant<JointTarget, PoseTarget, CircularTarget>;

struct PlannerConfig
{
  std::string planning_group;
  std::string end_effector_link;
  std::string reference_frame;
  std::string planning_pipeline;
};

struct MotionRequest
{
  std::string request_id;
  MotionType motion_type{MotionType::kPtp};
  MotionTarget target{JointTarget{}};
  double velocity_scale{0.1};
  double acceleration_scale{0.1};
  double planning_timeout{5.0};
  bool avoid_collisions{true};
};

struct PlanResult
{
  bool success{false};
  MotionError error{MotionError::kNone};
  std::int32_t moveit_error_code{0};
  std::string message;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double planning_time{0.0};
  std::string planner_id;
};

struct ValidationResult
{
  bool valid{false};
  MotionError error{MotionError::kNone};
  std::string message;
};

std::string to_string(MotionType motion_type);
std::string to_string(MotionError error);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__MOTION_TYPES_HPP_

