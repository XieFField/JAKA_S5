#ifndef EXECUTION_TYPE_HPP_
#define EXECUTION_TYPE_HPP_


#include <cstdint>
#include <string>

#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "massage_motion/motion_types.hpp"

namespace massage_motion
{

enum class ExecutionStatus : std::int32_t
{
    kIdle = 0,
    kExecuting,
    kSucceeded,
    kCanceled,
    kTimedOut,
    kFailed
};

enum class ExecutionError : std::int32_t
{
    kNone = 0,
    kInvalidRequest,
    kEmptyTrajectory,
    kBackendUnavailable,
    kRejected,
    kExecutionFailed,
    kCanceled,
    kTimeout
};

struct ExecutionRequest
{
    std::string request_id;
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    double timeout{0.0};
    bool has_motion_semantics{false};
    MotionType motion_type{MotionType::kPtp};
    double velocity_scale{0.0};
    double acceleration_scale{0.0};
    std::string planner_id;
    MotionTarget motion_target{JointTarget{}};
    // Zero means use the executor's configured speed and request scale.
    double desired_cartesian_speed_m_s{0.0};
};

struct ExecutionResult
{
    bool success{false};
    ExecutionError error{ExecutionError::kNone};
    std::int32_t backend_error_code{0};
    std::string message;
    ExecutionStatus status{ExecutionStatus::kIdle};
};

struct ExecutionValidationResult
{
    bool valid{false};
    ExecutionError error{ExecutionError::kNone};
    std::string message;
};

}// namespace massage_motion
#endif // EXECUTION_TYPE_HPP_
