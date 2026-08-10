#ifndef EXECUTION_TYPE_HPP_
#define EXECUTION_TYPE_HPP_


#include <cstdint>
#include <string>

#include "moveit_msgs/msg/robot_trajectory.hpp"

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
    double timeout;
};

struct ExecutionResult
{
    bool success{false};
    ExecutionError error{ExecutionError::kNone};
    std::int32_t error_code{0};
    std::string message;
    ExecutionStatus status{ExecutionStatus::kIdle};
};
}// namespace massage_motion
#endif // EXECUTION_TYPE_HPP_