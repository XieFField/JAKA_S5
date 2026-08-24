
#include "massage_motion/motion_validation.hpp"

#include <cmath>
#include <string>
#include <unordered_set>
#include <variant>

namespace massage_motion
{

namespace
{

// 用小型结果构造函数统一返回格式，让各校验分支只关注判断条件。
ValidationResult valid_result()
{
  return {true, MotionError::kNone, "valid"};
}

ValidationResult invalid_request(const std::string & message)
{
  return {false, MotionError::kInvalidRequest, message};
}

ValidationResult target_type_mismatch(const std::string & message)
{
  return {false, MotionError::kTargetTypeMismatch, message};
}

bool valid_scale(double scale)
{
  // isfinite 同时排除 NaN 和无穷大，避免异常数值进入规划后端。
  return std::isfinite(scale) && scale > 0.0 && scale <= 1.0;
}

bool has_frame(const geometry_msgs::msg::PoseStamped & pose)
{
  return !pose.header.frame_id.empty();
}

bool valid_start_state(
  const moveit_msgs::msg::RobotState & start_state,
  std::string & message)
{
  const auto & joint_state = start_state.joint_state;
  if (joint_state.name.empty() ||
    joint_state.name.size() != joint_state.position.size())
  {
    message = "start_state joint names and positions must be non-empty and aligned";
    return false;
  }
  std::unordered_set<std::string> names;
  names.reserve(joint_state.name.size());
  for (std::size_t index = 0; index < joint_state.name.size(); ++index)
  {
    if (joint_state.name[index].empty() ||
      !names.emplace(joint_state.name[index]).second ||
      !std::isfinite(joint_state.position[index]))
    {
      message = "start_state contains an empty/duplicate joint name or non-finite position";
      return false;
    }
  }
  return true;
}

}  // namespace

ValidationResult validate_motion_request(const MotionRequest & request)
{
  // 先校验所有动作共有的参数，再检查各动作对应的目标类型。
  if (!valid_scale(request.velocity_scale)) 
    return invalid_request("velocity_scale must be finite and in (0, 1]");
  

  if (!valid_scale(request.acceleration_scale)) 
    return invalid_request("acceleration_scale must be finite and in (0, 1]");
  

  if (!std::isfinite(request.planning_timeout) 
      || request.planning_timeout <= 0.0) 
    return invalid_request("planning_timeout must be finite and greater than zero");

  if (request.start_state.has_value())
  {
    std::string message;
    if (!valid_start_state(*request.start_state, message))
      return invalid_request(message);
  }
  

  // 每种动作只接受自身能够解释的目标类型。
  switch (request.motion_type) 
  {
    case MotionType::kPtp:
      if (const auto * joint_target = std::get_if<JointTarget>(&request.target)) 
      {
        if (joint_target->positions.empty()) 
          return invalid_request("PTP joint target must not be empty");
        
        return valid_result();
      }

      if (const auto * pose_target = std::get_if<PoseTarget>(&request.target)) 
      {
        if (!has_frame(pose_target->pose)) 
          return invalid_request("PTP pose target must contain a frame_id");
        
        return valid_result();
      }

      if (const auto * named_target = std::get_if<NamedJointTarget>(&request.target))
      {
        if (named_target->name.empty())
          return invalid_request("PTP named joint target must not be empty");

        return valid_result();
      }

      return target_type_mismatch(
        "PTP requires a JointTarget, NamedJointTarget or PoseTarget");

    case MotionType::kLin:
      if (const auto * pose_target = std::get_if<PoseTarget>(&request.target)) 
      {
        if (!has_frame(pose_target->pose)) 
          return invalid_request("LIN pose target must contain a frame_id");
        
        return valid_result();
      }

      return target_type_mismatch("LIN requires a PoseTarget");

    case MotionType::kCirc:
      if (const auto * circular_target = std::get_if<CircularTarget>(&request.target)) 
      {
        if (!has_frame(circular_target->interim_pose) 
            || !has_frame(circular_target->goal_pose)) 
          return invalid_request("CIRC poses must contain frame_id values");
        

        // 此处只比较坐标系名称是否一致。TF 是否真实存在，应由运行时规划器检查，
        // 不属于基础请求结构的静态校验职责。
        if (circular_target->interim_pose.header.frame_id !=
          circular_target->goal_pose.header.frame_id)
        {
          return invalid_request("CIRC interim and goal poses must use the same frame_id");
        }

        return valid_result();
      }

      return target_type_mismatch("CIRC requires a CircularTarget");

    default:
      return {
        false, 
        MotionError::kUnsupportedMotion, 
        "unsupported motion type"
      };
  }
}

}  // namespace massage_motion
