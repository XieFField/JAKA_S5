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

// 只描述运动路径的几何类型，时间参数化和轨迹执行属于其他模块。
enum class MotionType : std::uint8_t
{
  kPtp = 0,
  kLin,
  kCirc,
};

// 项目内部统一错误码，使上层代码不直接依赖 MoveIt 的错误码常量。
enum class MotionError : std::int32_t
{
  kNone = 0,
  kInvalidRequest,
  kTargetTypeMismatch,
  kPlanningFailed,
  kEmptyTrajectory,
  kUnsupportedMotion,
};

// PTP 规划可以直接使用规划组中各关节的目标位置。
struct JointTarget
{
  std::vector<double> positions;
};

// PoseStamped 同时保存目标位姿及该位姿所属的参考坐标系。
struct PoseTarget
{
  geometry_msgs::msg::PoseStamped pose;
};

// Pilz CIRC 规划需要一个圆弧中间点和一个最终目标位姿。
struct CircularTarget
{
  geometry_msgs::msg::PoseStamped interim_pose;
  geometry_msgs::msg::PoseStamped goal_pose;
};

// 使用 variant 保证一次请求只保存一种目标，避免一个大结构体同时包含
// 关节、位姿和圆弧字段，并产生无效的字段组合。
using MotionTarget = std::variant<JointTarget, PoseTarget, CircularTarget>;

// 规划器实例生命周期内通常保持不变的配置。
struct PlannerConfig
{
  std::string planning_group;
  std::string end_effector_link;
  std::string reference_frame;
  std::string planning_pipeline;
};

// 每次规划请求都可能变化的参数。
struct MotionRequest
{
  std::string request_id;
  MotionType motion_type{MotionType::kPtp};
  MotionTarget target{JointTarget{}};
  double velocity_scale{0.1};
  double acceleration_scale{0.1};
  double planning_timeout{5.0};
  // 为支持该选项的其他规划后端保留。MoveIt 运动规划默认进行碰撞检查，
  // MoveGroupInterface 中没有与该字段直接对应的 bool 设置函数。
  bool avoid_collisions{true};
};

// 对外结果包含 ROS 轨迹，但不暴露 MoveGroupInterface 等具体规划器对象。
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

// 校验结果通过统一错误码报告问题，不使用异常传递普通输入错误。
struct ValidationResult
{
  bool valid{false};
  MotionError error{MotionError::kNone};
  std::string message;
};

// 将枚举转换为字符串，供日志和故障诊断使用。
std::string to_string(MotionType motion_type);
std::string to_string(MotionError error);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__MOTION_TYPES_HPP_
