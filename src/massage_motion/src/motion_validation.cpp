#include "massage_motion/motion_validation.hpp"

#include <cmath>
#include <string>
#include <variant>

namespace massage_motion
{

namespace
{

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
  return std::isfinite(scale) && scale > 0.0 && scale <= 1.0;
}

bool has_frame(const geometry_msgs::msg::PoseStamped & pose)
{
  return !pose.header.frame_id.empty();
}

}  // namespace

ValidationResult validate_motion_request(const MotionRequest & request)
{
  if (!valid_scale(request.velocity_scale)) {
    return invalid_request("velocity_scale must be finite and in (0, 1]");
  }

  if (!valid_scale(request.acceleration_scale)) {
    return invalid_request("acceleration_scale must be finite and in (0, 1]");
  }

  if (!std::isfinite(request.planning_timeout) || request.planning_timeout <= 0.0) {
    return invalid_request("planning_timeout must be finite and greater than zero");
  }

  switch (request.motion_type) {
    case MotionType::kPtp:
      if (const auto * joint_target = std::get_if<JointTarget>(&request.target)) {
        if (joint_target->positions.empty()) {
          return invalid_request("PTP joint target must not be empty");
        }
        return valid_result();
      }

      if (const auto * pose_target = std::get_if<PoseTarget>(&request.target)) {
        if (!has_frame(pose_target->pose)) {
          return invalid_request("PTP pose target must contain a frame_id");
        }
        return valid_result();
      }

      return target_type_mismatch("PTP requires a JointTarget or PoseTarget");

    case MotionType::kLin:
      if (const auto * pose_target = std::get_if<PoseTarget>(&request.target)) {
        if (!has_frame(pose_target->pose)) {
          return invalid_request("LIN pose target must contain a frame_id");
        }
        return valid_result();
      }

      return target_type_mismatch("LIN requires a PoseTarget");

    case MotionType::kCirc:
      if (const auto * circular_target = std::get_if<CircularTarget>(&request.target)) {
        if (!has_frame(circular_target->interim_pose) || !has_frame(circular_target->goal_pose)) {
          return invalid_request("CIRC poses must contain frame_id values");
        }

        if (circular_target->interim_pose.header.frame_id !=
          circular_target->goal_pose.header.frame_id)
        {
          return invalid_request("CIRC interim and goal poses must use the same frame_id");
        }

        return valid_result();
      }

      return target_type_mismatch("CIRC requires a CircularTarget");

    default:
      return {false, MotionError::kUnsupportedMotion, "unsupported motion type"};
  }
}

}  // namespace massage_motion

