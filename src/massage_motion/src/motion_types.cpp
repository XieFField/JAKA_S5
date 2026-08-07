#include "massage_motion/motion_types.hpp"

namespace massage_motion
{

std::string to_string(MotionType motion_type)
{
  switch (motion_type) {
    case MotionType::kPtp:
      return "PTP";
    case MotionType::kLin:
      return "LIN";
    case MotionType::kCirc:
      return "CIRC";
    default:
      return "UNKNOWN_MOTION";
  }
}

std::string to_string(MotionError error)
{
  switch (error) {
    case MotionError::kNone:
      return "NONE";
    case MotionError::kInvalidRequest:
      return "INVALID_REQUEST";
    case MotionError::kTargetTypeMismatch:
      return "TARGET_TYPE_MISMATCH";
    case MotionError::kPlanningFailed:
      return "PLANNING_FAILED";
    case MotionError::kEmptyTrajectory:
      return "EMPTY_TRAJECTORY";
    case MotionError::kUnsupportedMotion:
      return "UNSUPPORTED_MOTION";
    default:
      return "UNKNOWN_ERROR";
  }
}

}  // namespace massage_motion

