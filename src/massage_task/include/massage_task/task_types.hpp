/**
 * @file task_types.hpp
 * @brief 定义任务状态机与上层业务之间使用的公共数据类型。
 */

#ifndef MASSAGE_TASK__TASK_TYPES_HPP_
#define MASSAGE_TASK__TASK_TYPES_HPP_

#include <cstdint>
#include <string>

#include "massage_motion/execution_types.hpp"
#include "massage_motion/motion_types.hpp"

namespace massage_task
{

// 最小状态机当前只描述一次规划与执行任务的生命周期。
enum class TaskState : std::int32_t
{
  kIdle = 0,
  kPlanning,
  kExecuting,
  kCompleted,
  kCanceled,
  kFault,
};

// 任务层错误码只表达编排阶段，具体规划和执行错误保留在 TaskResult 中。
enum class TaskError : std::int32_t
{
  kNone = 0,
  kBusy,
  kPlanningFailed,
  kExecutionFailed,
  kCanceled,
  kTimeout,
};

// 上层提交运动请求及该轨迹允许使用的总执行时间。
struct TaskRequest
{
  std::string task_id;
  massage_motion::MotionRequest motion_request;
  double execution_timeout{0.0};
};

// 同时保留规划和执行原始结果，便于状态机上层进行日志与故障诊断。
struct TaskResult
{
  bool success{false};
  TaskState final_state{TaskState::kIdle};
  TaskError error{TaskError::kNone};
  std::string message;
  massage_motion::PlanResult plan_result;
  massage_motion::ExecutionResult execution_result;
};

}  // namespace massage_task

#endif  // MASSAGE_TASK__TASK_TYPES_HPP_
