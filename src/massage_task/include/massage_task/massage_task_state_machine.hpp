#ifndef MASSAGE_TASK__MASSAGE_TASK_STATE_MACHINE_HPP_
#define MASSAGE_TASK__MASSAGE_TASK_STATE_MACHINE_HPP_

#include <atomic>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"

#include "massage_motion/compliance_controller.hpp"
#include "massage_motion/execution_timing.hpp"
#include "massage_motion/motion_planner.hpp"
#include "massage_motion/technique_path_generator.hpp"
#include "massage_motion/tool_orientation.hpp"
#include "massage_motion/trajectory_executor.hpp"
#include "massage_task/press_task_state_machine.hpp"

namespace massage_task
{

enum class MassageTaskState : std::int32_t
{
  kIdle = 0,
  kReady,
  kMoveStandby,
  kMoveWorkReady,
  kVerifyToolAlignment,
  kMovePreContact,
  kMoveTechniqueStart,
  kZeroAndValidateFt,
  kGuardedContactEntry,
  kContactConfirmed,
  kRampNormalForce,
  kExecuteTechnique,
  kReleaseNormalForce,
  kInterCycleRetreat,
  kInterCycleReturn,
  kRetreat,
  kReturnOverhead,
  kMoveSafeReturnExit,
  kComplete,
  kCanceled,
  kFault,
};

enum class MassageExecutionMode : std::int32_t
{
  kPlanOnly = 0,
  kFreeSpace,
  kCompliantContact,
};

enum class MassageTaskError : std::int32_t
{
  kNone = 0,
  kBusy,
  kInvalidRequest,
  kAuthorizationRequired,
  kSceneFailed,
  kPlanningFailed,
  kExecutionFailed,
  kToolAlignmentFailed,
  kFtValidationFailed,
  kComplianceFailed,
  kUnsupportedComplianceMode,
  kContactNotDetected,
  kTargetForceNotReached,
  kForceLimitExceeded,
  kCanceled,
  kRecoveryFailed,
};

struct ForceTorqueResult
{
  bool success{false};
  std::string message;
  double maximum_absolute_bias{0.0};
};

class IForceTorqueManager
{
public:
  virtual ~IForceTorqueManager() = default;
  virtual ForceTorqueResult zero_and_validate() = 0;
};

class ITechniqueTrajectoryPlanner
{
public:
  virtual ~ITechniqueTrajectoryPlanner() = default;
  virtual massage_motion::PlanResult plan(
    const massage_motion::TechniquePath & path,
    const moveit_msgs::msg::RobotState & start_state,
    bool align_to_current_tcp = true) = 0;
};

class IToolAlignmentValidator
{
public:
  virtual ~IToolAlignmentValidator() = default;
  virtual massage_motion::ToolAlignmentMetrics validate_trajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const massage_motion::ToolAlignmentGateConfig & config) = 0;
  virtual massage_motion::ToolAlignmentMetrics validate_current(
    const std::string & reference_frame,
    const massage_motion::ToolAlignmentGateConfig & config) = 0;
};

struct ReturnTrajectorySafetyConfig
{
  std::string tool_link{"massage_tool_tip"};
  std::string diagnostic_link{"Link_03"};
  double minimum_tool_z{0.0};
  double minimum_diagnostic_link_z{0.0};
  std::array<double, 3> surface_origin{};
  std::array<double, 3> surface_normal{{0.0, 0.0, 1.0}};
  std::array<double, 3> surface_tangent{{0.0, 1.0, 0.0}};
  double protected_tangent_min{0.0};
  double protected_tangent_max{0.0};
  double protected_lateral_half_width{0.0};
  double minimum_surface_clearance{0.0};
};

struct ReturnTrajectorySafetyResult
{
  bool valid{false};
  bool accepted{false};
  std::string message;
  double minimum_tool_z{0.0};
  double minimum_diagnostic_link_z{0.0};
  double minimum_surface_clearance{0.0};
  std::size_t protected_sample_count{0U};
};

class IReturnTrajectorySafetyValidator
{
public:
  virtual ~IReturnTrajectorySafetyValidator() = default;
  virtual ReturnTrajectorySafetyResult validate(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const ReturnTrajectorySafetyConfig & config) = 0;
};

struct MassageTaskRequest
{
  std::string task_id;
  MassageExecutionMode execution_mode{MassageExecutionMode::kCompliantContact};
  massage_motion::TechniquePathType technique{
    massage_motion::TechniquePathType::kPush};
  massage_motion::MotionTarget standby_target;
  geometry_msgs::msg::PoseStamped contact_pose;
  geometry_msgs::msg::Vector3 surface_normal;
  geometry_msgs::msg::Vector3 surface_tangent;
  double precontact_distance{0.020};
  double work_ready_clearance{0.050};
  double maximum_tool_axis_error{0.0523598775598299};
  double contact_search_depth{0.010};
  double free_space_velocity_scale{0.10};
  double free_space_acceleration_scale{0.08};
  double contact_velocity_scale{0.01};
  double planning_timeout{8.0};
  massage_motion::ExecutionTimingPolicy execution_timing;
  double contact_threshold{0.15};
  double target_normal_force{1.0};
  double maximum_normal_force{5.0};
  double contact_wait_timeout{1.0};
  double force_ramp_timeout{2.0};
  std::size_t push_repetitions{1U};
  double safe_return_patient_margin{0.030};
  double safe_return_exit_margin{0.020};
  double safe_return_minimum_tool_clearance{0.005};
  double safe_return_minimum_diagnostic_link_z{0.0};
  bool return_to_standby{true};
  // 真机执行必须同时显式选择 real、execute 和 parameters_confirmed。
  std::string execution_environment{"simulation"};
  bool execute{false};
  bool parameters_confirmed{false};
  massage_motion::ComplianceRequest compliance_request;
  massage_motion::PushPathRequest push;
  massage_motion::PressPathRequest press;
  massage_motion::KneadPathRequest knead;
};

struct MassageTaskResult
{
  bool success{false};
  MassageTaskState final_state{MassageTaskState::kIdle};
  MassageTaskError error{MassageTaskError::kNone};
  MassageTaskError primary_error{MassageTaskError::kNone};
  std::string message;
  std::string primary_message;
  bool contact_detected{false};
  bool target_force_reached{false};
  double peak_normal_force{0.0};
  bool recovery_attempted{false};
  bool recovery_succeeded{false};
  // Contact/compliance sessions completed. Push repeats create one session each.
  std::size_t completed_repetitions{0U};
  // Business-level motions completed: pushes, press cycles, or knead circles.
  std::size_t completed_technique_cycles{0U};
  std::vector<MassageTaskState> state_trace;
  massage_motion::TechniquePathResult path_result;
  massage_motion::PlanResult last_plan_result;
  massage_motion::ExecutionResult last_execution_result;
  massage_motion::ComplianceResult compliance_result;
  ForceTorqueResult ft_result;
  std::vector<massage_motion::ToolAlignmentMetrics> alignment_results;
  std::vector<ReturnTrajectorySafetyResult> return_safety_results;
};

class MassageTaskStateMachine
{
public:
  MassageTaskStateMachine(
    std::shared_ptr<massage_motion::IMotionPlanner> motion_planner,
    std::shared_ptr<ITechniqueTrajectoryPlanner> technique_planner,
    std::shared_ptr<IToolAlignmentValidator> alignment_validator,
    std::shared_ptr<IReturnTrajectorySafetyValidator> return_safety_validator,
    std::shared_ptr<massage_motion::ITrajectoryExecutor> executor,
    std::shared_ptr<massage_motion::IComplianceController> compliance,
    std::shared_ptr<IForceTorqueManager> ft_manager,
    std::shared_ptr<IContactSceneManager> scene_manager);

  MassageTaskResult run(const MassageTaskRequest & request);
  bool cancel();
  bool reset();
  MassageTaskState state() const;

private:
  std::shared_ptr<massage_motion::IMotionPlanner> motion_planner_;
  std::shared_ptr<ITechniqueTrajectoryPlanner> technique_planner_;
  std::shared_ptr<IToolAlignmentValidator> alignment_validator_;
  std::shared_ptr<IReturnTrajectorySafetyValidator> return_safety_validator_;
  std::shared_ptr<massage_motion::ITrajectoryExecutor> executor_;
  std::shared_ptr<massage_motion::IComplianceController> compliance_;
  std::shared_ptr<IForceTorqueManager> ft_manager_;
  std::shared_ptr<IContactSceneManager> scene_manager_;
  std::mutex run_mutex_;
  std::atomic<MassageTaskState> state_{MassageTaskState::kIdle};
  std::atomic_bool cancel_requested_{false};
};

std::string to_string(MassageTaskState state);
std::string to_string(MassageExecutionMode mode);
bool parse_execution_mode(
  const std::string & value, MassageExecutionMode * mode);

}  // namespace massage_task

#endif  // MASSAGE_TASK__MASSAGE_TASK_STATE_MACHINE_HPP_
