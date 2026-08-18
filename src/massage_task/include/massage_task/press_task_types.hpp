#ifndef MASSAGE_TASK__PRESS_TASK_TYPES_HPP_
#define MASSAGE_TASK__PRESS_TASK_TYPES_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "geometry_msgs/msg/vector3.hpp"

#include "massage_motion/compliance_types.hpp"
#include "massage_motion/execution_types.hpp"
#include "massage_motion/execution_timing.hpp"
#include "massage_motion/motion_types.hpp"

namespace massage_task
{

enum class PressTaskState : std::int32_t
{
    kIdle = 0,
    kPlanApproach,
    kExecuteApproach,
    kPlanPrecontact,
    kExecutePrecontact,
    kPlanPress,
    kCompliantPress,
    kRetreat,
    kCompleted,
    kCanceled,
    kFault
};

enum class PressTaskError : std::int32_t
{
    kNone = 0,
    kBusy,
    kInvalidRequest,
    kSceneFailed,
    kPlanningFailed,
    kExecutionFailed,
    kComplianceFailed,
    kContactNotDetected,
    kCanceled,
    kRecoveryFailed
};

struct PressTaskRequest
{
    std::string task_id;
    massage_motion::PoseTarget contact_target;
    geometry_msgs::msg::Vector3 surface_normal;
    double approach_distance{0.04};
    double precontact_distance{0.0055};
    double approach_velocity_scale{0.15};
    double precontact_velocity_scale{0.05};
    double press_velocity_scale{0.005};
    double planning_timeout{5.0};
    massage_motion::ExecutionTimingPolicy execution_timing;
    double contact_threshold{0.1};
    double maximum_contact_wrench{1.0};
    double contact_wait_timeout{1.0};
    double hold_duration{2.0};
    std::size_t contact_axis{2};
    massage_motion::ComplianceRequest compliance_request;
};

struct PressTaskResult
{
    bool success{false};
    PressTaskState final_state{PressTaskState::kIdle};
    PressTaskError error{PressTaskError::kNone};
    std::string message;
    PressTaskError primary_error{PressTaskError::kNone};
    std::string primary_message;
    bool contact_detected{false};
    double peak_absolute_wrench{0.0};
    bool recovery_attempted{false};
    bool recovery_succeeded{false};
    std::string recovery_message;
    massage_motion::PlanResult last_plan_result;
    massage_motion::ExecutionResult last_execution_result;
    massage_motion::ComplianceResult compliance_result;
};

}  // namespace massage_task

#endif  // MASSAGE_TASK__PRESS_TASK_TYPES_HPP_
