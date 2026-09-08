#include "massage_task/massage_task_state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace massage_task
{
namespace
{

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

moveit_msgs::msg::RobotState terminal_state(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  moveit_msgs::msg::RobotState state;
  if (trajectory.joint_trajectory.points.empty())
  {
    return state;
  }
  state.joint_state.name = trajectory.joint_trajectory.joint_names;
  state.joint_state.position =
    trajectory.joint_trajectory.points.back().positions;
  state.is_diff = true;
  return state;
}

massage_motion::TechniquePathResult generate_path(
  const MassageTaskRequest & request)
{
  switch (request.technique)
  {
    case massage_motion::TechniquePathType::kPush:
    {
      auto path_request = request.push;
      path_request.start_pose = request.contact_pose;
      return massage_motion::TechniquePathGenerator::generate_push(path_request);
    }
    case massage_motion::TechniquePathType::kPress:
    {
      auto path_request = request.press;
      path_request.contact_pose = request.contact_pose;
      path_request.surface_normal_x = request.surface_normal.x;
      path_request.surface_normal_y = request.surface_normal.y;
      path_request.surface_normal_z = request.surface_normal.z;
      return massage_motion::TechniquePathGenerator::generate_press(path_request);
    }
    case massage_motion::TechniquePathType::kKnead:
    {
      auto path_request = request.knead;
      path_request.center_pose = request.contact_pose;
      return massage_motion::TechniquePathGenerator::generate_knead(path_request);
    }
    default:
    {
      massage_motion::TechniquePathResult result;
      result.message = "不支持的手法类型";
      return result;
    }
  }
}

}  // namespace

MassageTaskStateMachine::MassageTaskStateMachine(
  std::shared_ptr<massage_motion::IMotionPlanner> motion_planner,
  std::shared_ptr<ITechniqueTrajectoryPlanner> technique_planner,
  std::shared_ptr<IToolAlignmentValidator> alignment_validator,
  std::shared_ptr<IReturnTrajectorySafetyValidator> return_safety_validator,
  std::shared_ptr<massage_motion::ITrajectoryExecutor> executor,
  std::shared_ptr<massage_motion::IComplianceController> compliance,
  std::shared_ptr<IForceTorqueManager> ft_manager,
  std::shared_ptr<IContactSceneManager> scene_manager)
: motion_planner_(std::move(motion_planner)),
  technique_planner_(std::move(technique_planner)),
  alignment_validator_(std::move(alignment_validator)),
  return_safety_validator_(std::move(return_safety_validator)),
  executor_(std::move(executor)),
  compliance_(std::move(compliance)),
  ft_manager_(std::move(ft_manager)),
  scene_manager_(std::move(scene_manager))
{
  if (!motion_planner_ || !technique_planner_ || !alignment_validator_ ||
    !return_safety_validator_)
  {
    throw std::invalid_argument("推拿状态机核心运动依赖不能为空");
  }
}

MassageTaskResult MassageTaskStateMachine::run(const MassageTaskRequest & request)
{
  std::unique_lock<std::mutex> run_lock(run_mutex_, std::try_to_lock);
  if (!run_lock.owns_lock() || state_.load() != MassageTaskState::kIdle)
  {
    MassageTaskResult busy;
    busy.final_state = state_.load();
    busy.error = MassageTaskError::kBusy;
    busy.primary_error = busy.error;
    busy.message = "推拿状态机正在运行或尚未复位";
    busy.primary_message = busy.message;
    return busy;
  }

  MassageTaskResult result;
  cancel_requested_.store(false);
  const auto transition = [&](MassageTaskState next)
    {
      state_.store(next);
      result.state_trace.push_back(next);
    };
  const auto fail = [&](MassageTaskError error, const std::string & message)
    {
      if (result.primary_error == MassageTaskError::kNone)
      {
        result.primary_error = error;
        result.primary_message = message;
      }
    };

  const double normal_norm = std::sqrt(
    request.surface_normal.x * request.surface_normal.x +
    request.surface_normal.y * request.surface_normal.y +
    request.surface_normal.z * request.surface_normal.z);
  massage_motion::ToolOrientationRequest orientation_request;
  orientation_request.surface_normal_world = {
    request.surface_normal.x, request.surface_normal.y,
    request.surface_normal.z};
  orientation_request.tangent_direction_world = {
    request.surface_tangent.x, request.surface_tangent.y,
    request.surface_tangent.z};
  const auto tool_orientation =
    massage_motion::make_surface_aligned_tool_orientation(orientation_request);
  const bool real = request.execution_environment == "real";
  const bool plan_only =
    request.execution_mode == MassageExecutionMode::kPlanOnly;
  const bool free_space =
    request.execution_mode == MassageExecutionMode::kFreeSpace;
  const bool compliant_contact =
    request.execution_mode == MassageExecutionMode::kCompliantContact;
  const bool valid_environment = real ||
    request.execution_environment == "simulation";
  const bool valid_common = !request.task_id.empty() && valid_environment &&
    (plan_only || request.execute) &&
    std::isfinite(normal_norm) && normal_norm > 1.0e-9 &&
    tool_orientation.valid &&
    finite_positive(request.precontact_distance) &&
    finite_positive(request.work_ready_clearance) &&
    finite_positive(request.maximum_tool_axis_error) &&
    request.maximum_tool_axis_error <= M_PI &&
    finite_positive(request.contact_search_depth) &&
    finite_positive(request.free_space_velocity_scale) &&
    request.free_space_velocity_scale <= 1.0 &&
    finite_positive(request.free_space_acceleration_scale) &&
    request.free_space_acceleration_scale <= 1.0 &&
    finite_positive(request.planning_timeout) &&
    request.push_repetitions > 0U && request.push_repetitions <= 100U &&
    finite_positive(request.safe_return_patient_margin) &&
    finite_positive(request.safe_return_exit_margin) &&
    std::isfinite(request.safe_return_minimum_tool_clearance) &&
    request.safe_return_minimum_tool_clearance >= 0.0 &&
    std::isfinite(request.safe_return_minimum_diagnostic_link_z);
  const bool valid_contact = !compliant_contact ||
    (finite_positive(request.contact_search_depth) &&
    finite_positive(request.contact_velocity_scale) &&
    request.contact_velocity_scale <= 1.0 &&
    finite_positive(request.contact_threshold) &&
    finite_positive(request.target_normal_force) &&
    finite_positive(request.maximum_normal_force) &&
    request.contact_threshold <= request.target_normal_force &&
    request.target_normal_force < request.maximum_normal_force &&
    finite_positive(request.contact_wait_timeout) &&
    finite_positive(request.force_ramp_timeout));
  const bool contact_dependencies_available = !compliant_contact ||
    (compliance_ && ft_manager_ && scene_manager_);
  const bool execution_dependency_available = plan_only || executor_;
  if (!valid_common || !valid_contact || (!plan_only && !free_space && !compliant_contact))
  {
    fail(
      MassageTaskError::kInvalidRequest,
      tool_orientation.valid ? "推拿任务参数无效" :
      "推拿工具姿态参数无效: " + tool_orientation.message);
  }
  else if (real && !plan_only && !request.parameters_confirmed)
  {
    fail(
      MassageTaskError::kAuthorizationRequired,
      "真机执行要求 execute=true 且 parameters_confirmed=true");
  }
  else if (!contact_dependencies_available)
  {
    fail(
      MassageTaskError::kInvalidRequest,
      "柔顺接触模式缺少 compliance、FT 或接触场景依赖");
  }
  else if (!execution_dependency_available)
  {
    fail(MassageTaskError::kInvalidRequest, "执行模式缺少轨迹执行器");
  }
  else if (compliant_contact && !compliance_->capabilities().reference_tracking)
  {
    fail(
      MassageTaskError::kUnsupportedComplianceMode,
      "当前柔顺后端不支持接触阶段参考轨迹跟踪；拒绝在导纳状态混用未经验证的运动通道");
  }

  if (result.primary_error != MassageTaskError::kNone)
  {
    transition(MassageTaskState::kFault);
    result.final_state = state_.load();
    result.error = result.primary_error;
    result.message = result.primary_message;
    return result;
  }

  auto effective_request = request;
  effective_request.contact_pose.pose.orientation = tool_orientation.orientation;
  result.path_result = generate_path(effective_request);
  if (!result.path_result.success || result.path_result.path.points.empty())
  {
    fail(
      MassageTaskError::kInvalidRequest,
      "手法路径生成失败: " + result.path_result.message);
    transition(MassageTaskState::kFault);
    result.final_state = state_.load();
    result.error = result.primary_error;
    result.message = result.primary_message;
    return result;
  }

  const double nx = request.surface_normal.x / normal_norm;
  const double ny = request.surface_normal.y / normal_norm;
  const double nz = request.surface_normal.z / normal_norm;
  auto first_contact_pose = effective_request.contact_pose;
  first_contact_pose.pose = result.path_result.path.points.front().pose;
  auto precontact_pose = first_contact_pose;
  precontact_pose.pose.position.x += nx * request.precontact_distance;
  precontact_pose.pose.position.y += ny * request.precontact_distance;
  precontact_pose.pose.position.z += nz * request.precontact_distance;
  auto work_ready_pose = precontact_pose;
  work_ready_pose.pose.position.x += nx * request.work_ready_clearance;
  work_ready_pose.pose.position.y += ny * request.work_ready_clearance;
  work_ready_pose.pose.position.z += nz * request.work_ready_clearance;
  auto search_pose = first_contact_pose;
  search_pose.pose.position.x -= nx * request.contact_search_depth;
  search_pose.pose.position.y -= ny * request.contact_search_depth;
  search_pose.pose.position.z -= nz * request.contact_search_depth;
  auto last_contact_pose = effective_request.contact_pose;
  last_contact_pose.pose = result.path_result.path.points.back().pose;

  const double tangent_normal_projection =
    request.surface_tangent.x * nx + request.surface_tangent.y * ny +
    request.surface_tangent.z * nz;
  double tx = request.surface_tangent.x - tangent_normal_projection * nx;
  double ty = request.surface_tangent.y - tangent_normal_projection * ny;
  double tz = request.surface_tangent.z - tangent_normal_projection * nz;
  const double tangent_norm = std::sqrt(tx * tx + ty * ty + tz * tz);
  tx /= tangent_norm;
  ty /= tangent_norm;
  tz /= tangent_norm;
  const double lx = ny * tz - nz * ty;
  const double ly = nz * tx - nx * tz;
  const double lz = nx * ty - ny * tx;
  double protected_tangent_min = std::numeric_limits<double>::infinity();
  double protected_tangent_max = -std::numeric_limits<double>::infinity();
  double protected_lateral_half_width = 0.0;
  for (const auto & point : result.path_result.path.points)
  {
    const double dx = point.pose.position.x -
      effective_request.contact_pose.pose.position.x;
    const double dy = point.pose.position.y -
      effective_request.contact_pose.pose.position.y;
    const double dz = point.pose.position.z -
      effective_request.contact_pose.pose.position.z;
    const double tangent_coordinate = dx * tx + dy * ty + dz * tz;
    const double lateral_coordinate = dx * lx + dy * ly + dz * lz;
    protected_tangent_min = std::min(
      protected_tangent_min, tangent_coordinate);
    protected_tangent_max = std::max(
      protected_tangent_max, tangent_coordinate);
    protected_lateral_half_width = std::max(
      protected_lateral_half_width, std::abs(lateral_coordinate));
  }
  protected_tangent_min -= request.safe_return_patient_margin;
  protected_tangent_max += request.safe_return_patient_margin;
  protected_lateral_half_width += request.safe_return_patient_margin;

  bool scene_prepared = false;
  bool contact_allowed = false;
  bool compliance_started = false;
  bool robot_moved = false;
  bool route_progressed = false;
  bool precontact_reached = false;
  std::optional<moveit_msgs::msg::RobotState> planned_terminal_state;

  massage_motion::ToolAlignmentGateConfig alignment_config;
  alignment_config.desired_tool_z_world = {-nx, -ny, -nz};
  alignment_config.expected_orientation = tool_orientation.orientation;
  alignment_config.maximum_axis_error = request.maximum_tool_axis_error;

  const auto motion_request = [&](
    const std::string & suffix, massage_motion::MotionType type,
    const massage_motion::MotionTarget & target, double velocity_scale,
    double acceleration_scale = 0.0)
    {
      massage_motion::MotionRequest motion;
      motion.request_id = request.task_id + "_" + suffix;
      motion.motion_type = type;
      motion.target = target;
      motion.velocity_scale = velocity_scale;
      motion.acceleration_scale = acceleration_scale > 0.0 ?
        acceleration_scale : velocity_scale;
      motion.planning_timeout = request.planning_timeout;
      motion.avoid_collisions = true;
      return motion;
    };

  const auto execute_plan = [&](const massage_motion::PlanResult & plan,
      const std::string & id,
      const massage_motion::MotionRequest * motion = nullptr,
      bool defer_start_mismatch = false) -> bool
    {
      if (plan_only)
      {
        return true;
      }
      if (!plan.success || plan.trajectory.joint_trajectory.points.empty())
      {
        fail(
          MassageTaskError::kPlanningFailed,
          id + " 执行门禁拒绝：规划未成功或轨迹为空；未发送运动 Goal");
        return false;
      }
      const auto dispatch = [&] (
        const moveit_msgs::msg::RobotTrajectory & trajectory,
        const std::string & request_id,
        const massage_motion::MotionRequest * semantic_motion,
        const std::string & planner_id,
        double desired_cartesian_speed_m_s) -> bool
        {
          const auto timing = massage_motion::calculate_execution_timing(
            trajectory, request.execution_timing);
          if (!timing.valid)
          {
            fail(
              MassageTaskError::kInvalidRequest,
              "轨迹执行时限无效: " + timing.message);
            return false;
          }
          massage_motion::ExecutionRequest execution;
          execution.request_id = request_id;
          execution.robot_trajectory = trajectory;
          execution.timeout = timing.timeout;
          if (semantic_motion)
          {
            execution.has_motion_semantics = true;
            execution.motion_type = semantic_motion->motion_type;
            execution.velocity_scale = semantic_motion->velocity_scale;
            execution.acceleration_scale = semantic_motion->acceleration_scale;
            execution.planner_id = planner_id;
            execution.motion_target = semantic_motion->target;
            execution.desired_cartesian_speed_m_s =
              desired_cartesian_speed_m_s;
          }
          result.last_execution_result = executor_->execute(execution);
          if (!result.last_execution_result.success)
          {
            const bool retryable_start_mismatch =
              defer_start_mismatch && semantic_motion &&
              result.last_execution_result.error ==
              massage_motion::ExecutionError::kRejected &&
              (result.last_execution_result.message.find(
              "原生 PTP 起点与当前关节状态不一致") != std::string::npos ||
              result.last_execution_result.message.find(
              "NATIVE CARTESIAN START GATE: REJECTED") != std::string::npos);
            if (!retryable_start_mismatch)
            {
              fail(
                result.last_execution_result.status ==
                massage_motion::ExecutionStatus::kCanceled ?
                MassageTaskError::kCanceled : MassageTaskError::kExecutionFailed,
                result.last_execution_result.message);
            }
            return false;
          }
          robot_moved = true;
          return true;
        };

      if (!plan.execution_segments.empty())
      {
        for (std::size_t index = 0U; index < plan.execution_segments.size(); ++index)
        {
          const auto & segment = plan.execution_segments[index];
          massage_motion::MotionRequest semantic_motion;
          semantic_motion.request_id = segment.request_id;
          semantic_motion.motion_type = segment.motion_type;
          semantic_motion.target = segment.target;
          semantic_motion.velocity_scale = segment.velocity_scale;
          semantic_motion.acceleration_scale = segment.acceleration_scale;
          semantic_motion.planning_timeout = request.planning_timeout;
          semantic_motion.avoid_collisions = true;

          // Native LIN/CIRC finishes within Cartesian tolerances, so its joint
          // endpoint need not equal MoveIt's nominal IK endpoint. Replan every
          // semantic segment from the latest measured state before dispatch.
          const auto refreshed = motion_planner_->plan(semantic_motion);
          if (!refreshed.success ||
            refreshed.trajectory.joint_trajectory.points.empty())
          {
            fail(
              MassageTaskError::kPlanningFailed,
              "笛卡尔分段实时起点重规划失败: " + refreshed.message);
            return false;
          }
          auto refreshed_alignment_config = alignment_config;
          refreshed_alignment_config.inspect_all_samples = true;
          const auto refreshed_alignment =
            alignment_validator_->validate_trajectory(
            refreshed.trajectory, refreshed_alignment_config);
          result.alignment_results.push_back(refreshed_alignment);
          if (!refreshed_alignment.valid || !refreshed_alignment.accepted)
          {
            fail(
              MassageTaskError::kToolAlignmentFailed,
              "笛卡尔分段实时起点工具姿态门禁失败: " +
              refreshed_alignment.message);
            return false;
          }
          if (!dispatch(
              refreshed.trajectory,
              request.task_id + "_" + id + "_segment_" +
              std::to_string(index + 1U),
              &semantic_motion, refreshed.planner_id,
              segment.desired_cartesian_speed_m_s))
          {
            return false;
          }
        }
        return true;
      }

      return dispatch(
        plan.trajectory, request.task_id + "_" + id + "_execution",
        motion, plan.planner_id, 0.0);
    };

  const auto validate_alignment = [&] (
    const massage_motion::PlanResult & plan, bool inspect_all_samples,
    const std::string & scope) -> bool
    {
      auto config = alignment_config;
      config.inspect_all_samples = inspect_all_samples;
      auto metrics = alignment_validator_->validate_trajectory(
        plan.trajectory, config);
      result.alignment_results.push_back(metrics);
      if (!metrics.valid || !metrics.accepted)
      {
        fail(
          MassageTaskError::kToolAlignmentFailed,
          scope + " 工具姿态门禁失败: " + metrics.message);
        return false;
      }
      return true;
    };

  const auto plan_motion = [&](massage_motion::MotionRequest motion)
    {
      if (plan_only && planned_terminal_state.has_value())
      {
        motion.start_state = planned_terminal_state;
      }
      auto plan = motion_planner_->plan(motion);
      if (plan.success && !plan.trajectory.joint_trajectory.points.empty() &&
        plan_only)
      {
        planned_terminal_state = terminal_state(plan.trajectory);
      }
      return plan;
    };

  const auto plan_and_execute = [&](massage_motion::MotionRequest motion,
      MassageTaskState next, bool gate_alignment = false,
      bool inspect_all_samples = true) -> bool
    {
      if (cancel_requested_.load())
      {
        fail(MassageTaskError::kCanceled, "任务在运动前被取消");
        return false;
      }
      transition(next);
      result.last_plan_result = plan_motion(motion);
      if (!result.last_plan_result.success)
      {
        fail(MassageTaskError::kPlanningFailed, result.last_plan_result.message);
        return false;
      }
      if (gate_alignment && !validate_alignment(
          result.last_plan_result, inspect_all_samples, to_string(next)))
      {
        return false;
      }
      if (!execute_plan(
          result.last_plan_result, to_string(next), &motion, true))
      {
        const bool retryable_start_mismatch =
          result.primary_error == MassageTaskError::kNone &&
          result.last_execution_result.error ==
          massage_motion::ExecutionError::kRejected &&
          (result.last_execution_result.message.find(
          "原生 PTP 起点与当前关节状态不一致") != std::string::npos ||
          result.last_execution_result.message.find(
          "NATIVE CARTESIAN START GATE: REJECTED") != std::string::npos);
        if (!retryable_start_mismatch)
        {
          return false;
        }

        // The robot did not move: refresh the live planning start and repeat
        // this segment's planning and safety validation exactly once.
        result.last_plan_result = plan_motion(motion);
        if (!result.last_plan_result.success)
        {
          fail(
            MassageTaskError::kPlanningFailed,
            "PTP 起点刷新后的重规划失败: " +
            result.last_plan_result.message);
          return false;
        }
        if (gate_alignment && !validate_alignment(
            result.last_plan_result, inspect_all_samples,
            to_string(next) + "_start_refresh"))
        {
          return false;
        }
        if (!execute_plan(
            result.last_plan_result, to_string(next) + "_start_refresh",
            &motion))
        {
          return false;
        }
      }
      route_progressed = true;
      return true;
    };

  transition(MassageTaskState::kReady);
  if (compliant_contact && !scene_manager_->prepare())
  {
    fail(MassageTaskError::kSceneFailed, "准备接触场景失败");
  }
  else if (compliant_contact)
  {
    scene_prepared = true;
  }

  const auto standby = motion_request(
    "standby", massage_motion::MotionType::kPtp,
    request.standby_target, request.free_space_velocity_scale,
    request.free_space_acceleration_scale);
  const auto work_ready = motion_request(
    "work_ready", massage_motion::MotionType::kPtp,
    massage_motion::PoseTarget{work_ready_pose}, request.free_space_velocity_scale,
    request.free_space_acceleration_scale);
  const auto precontact = motion_request(
    "precontact", massage_motion::MotionType::kLin,
    massage_motion::PoseTarget{precontact_pose},
    compliant_contact ? request.contact_velocity_scale :
    request.free_space_velocity_scale,
    compliant_contact ? request.contact_velocity_scale :
    request.free_space_acceleration_scale);
  const auto search = motion_request(
    "guarded_contact", massage_motion::MotionType::kLin,
    massage_motion::PoseTarget{search_pose}, request.contact_velocity_scale);
  if (result.primary_error == MassageTaskError::kNone)
  {
    plan_and_execute(standby, MassageTaskState::kMoveStandby);
  }
  if (result.primary_error == MassageTaskError::kNone)
  {
    plan_and_execute(
      work_ready, MassageTaskState::kMoveWorkReady, true, false);
  }
  if (result.primary_error == MassageTaskError::kNone && !plan_only)
  {
    transition(MassageTaskState::kVerifyToolAlignment);
    auto current_metrics = alignment_validator_->validate_current(
      effective_request.contact_pose.header.frame_id, alignment_config);
    result.alignment_results.push_back(current_metrics);
    if (!current_metrics.valid || !current_metrics.accepted)
    {
      fail(
        MassageTaskError::kToolAlignmentFailed,
        "work-ready 当前工具姿态门禁失败: " + current_metrics.message);
    }
  }
  if (result.primary_error == MassageTaskError::kNone)
  {
    precontact_reached = plan_and_execute(
      precontact, MassageTaskState::kMovePreContact, true, true);
  }
  if (result.primary_error == MassageTaskError::kNone && compliant_contact)
  {
    transition(MassageTaskState::kZeroAndValidateFt);
    result.ft_result = ft_manager_->zero_and_validate();
    if (!result.ft_result.success)
    {
      fail(MassageTaskError::kFtValidationFailed, result.ft_result.message);
    }
  }
  if (result.primary_error == MassageTaskError::kNone && compliant_contact)
  {
    contact_allowed = scene_manager_->allow_tool_contact(true);
    if (!contact_allowed)
    {
      fail(MassageTaskError::kSceneFailed, "无法允许按摩头接触目标表面");
    }
  }

  const auto monitor_feedback = [&] (
    bool & cycle_contact_detected, bool & cycle_target_force_reached) -> bool
    {
      const auto feedback = compliance_->feedback();
      if (feedback.stale ||
        feedback.status != massage_motion::ComplianceStatus::kActive)
      {
        fail(MassageTaskError::kComplianceFailed, "柔顺反馈失效或控制器提前退出");
        return false;
      }
      const double normal_force = std::abs(
        feedback.wrench[0] * nx + feedback.wrench[1] * ny +
        feedback.wrench[2] * nz);
      result.peak_normal_force = std::max(result.peak_normal_force, normal_force);
      if (normal_force >= request.maximum_normal_force)
      {
        fail(MassageTaskError::kForceLimitExceeded, "法向力达到任务安全上限");
        return false;
      }
      cycle_contact_detected = cycle_contact_detected ||
        normal_force >= request.contact_threshold;
      cycle_target_force_reached = cycle_target_force_reached ||
        normal_force >= request.target_normal_force;
      result.contact_detected = result.contact_detected || cycle_contact_detected;
      result.target_force_reached =
        result.target_force_reached || cycle_target_force_reached;
      return true;
    };

  const auto stream_plan = [&](const massage_motion::PlanResult & plan,
      bool stop_at_contact, bool stop_at_target_force,
      std::size_t begin_index, std::size_t * next_index,
      bool & cycle_contact_detected,
      bool & cycle_target_force_reached) -> bool
    {
      const auto & trajectory = plan.trajectory.joint_trajectory;
      const auto started = std::chrono::steady_clock::now();
      const double initial_time = begin_index < trajectory.points.size() ?
        duration_seconds(trajectory.points[begin_index].time_from_start) : 0.0;
      for (std::size_t index = begin_index; index < trajectory.points.size(); ++index)
      {
        const auto & point = trajectory.points[index];
        std::this_thread::sleep_until(
          started + std::chrono::duration<double>(
            duration_seconds(point.time_from_start) - initial_time));
        if (cancel_requested_.load())
        {
          fail(MassageTaskError::kCanceled, "柔顺运动被取消");
          return false;
        }
        if (!monitor_feedback(
            cycle_contact_detected, cycle_target_force_reached))
        {
          return false;
        }
        if ((stop_at_contact && cycle_contact_detected) ||
          (stop_at_target_force && cycle_target_force_reached))
        {
          if (next_index)
          {
            *next_index = index + 1U;
          }
          return true;
        }
        massage_motion::ComplianceReference reference;
        reference.joint_names = trajectory.joint_names;
        reference.positions = point.positions;
        reference.velocities = point.velocities;
        reference.time_from_start = duration_seconds(point.time_from_start);
        if (!compliance_->update_reference(reference))
        {
          fail(MassageTaskError::kComplianceFailed, "柔顺控制器拒绝关节参考");
          return false;
        }
      }
      if (next_index)
      {
        *next_index = trajectory.points.size();
      }
      return true;
    };

  geometry_msgs::msg::PoseStamped recovery_surface_pose = first_contact_pose;
  const std::size_t repetition_count =
    request.technique == massage_motion::TechniquePathType::kPush ?
    request.push_repetitions : 1U;

  if (free_space || plan_only)
  {
    for (std::size_t repetition = 0U;
      repetition < repetition_count &&
      result.primary_error == MassageTaskError::kNone; ++repetition)
    {
      const auto technique_start = motion_request(
        "technique_start_" + std::to_string(repetition + 1U),
        massage_motion::MotionType::kLin,
        massage_motion::PoseTarget{first_contact_pose},
        request.free_space_velocity_scale,
        request.free_space_acceleration_scale);
      if (!plan_and_execute(
          technique_start, MassageTaskState::kMoveTechniqueStart, true, true))
      {
        break;
      }

      moveit_msgs::msg::RobotState technique_start_state =
        terminal_state(result.last_plan_result.trajectory);
      auto technique_plan = technique_planner_->plan(
        result.path_result.path, technique_start_state, !plan_only);
      result.last_plan_result = technique_plan;
      if (!technique_plan.success ||
        technique_plan.trajectory.joint_trajectory.points.empty())
      {
        fail(
          MassageTaskError::kPlanningFailed,
          "第 " + std::to_string(repetition + 1U) +
          " 次自由空间手法轨迹规划失败: " + technique_plan.message);
        break;
      }
      if (!validate_alignment(
          technique_plan, true,
          "execute_technique_" + std::to_string(repetition + 1U)))
      {
        break;
      }
      if (plan_only)
      {
        planned_terminal_state = terminal_state(technique_plan.trajectory);
      }
      transition(MassageTaskState::kExecuteTechnique);
      if (!execute_plan(
          technique_plan,
          "execute_technique_" + std::to_string(repetition + 1U)))
      {
        break;
      }
      route_progressed = true;
      ++result.completed_repetitions;
      switch (request.technique)
      {
        case massage_motion::TechniquePathType::kPush:
          ++result.completed_technique_cycles;
          break;
        case massage_motion::TechniquePathType::kPress:
          result.completed_technique_cycles += request.press.cycles;
          break;
        case massage_motion::TechniquePathType::kKnead:
          result.completed_technique_cycles += request.knead.cycles;
          break;
      }
      recovery_surface_pose = last_contact_pose;

      if (repetition + 1U < repetition_count)
      {
        auto end_precontact_pose = last_contact_pose;
        end_precontact_pose.pose.position.x += nx * request.precontact_distance;
        end_precontact_pose.pose.position.y += ny * request.precontact_distance;
        end_precontact_pose.pose.position.z += nz * request.precontact_distance;
        const auto inter_cycle_retreat = motion_request(
          "inter_cycle_retreat_" + std::to_string(repetition + 1U),
          massage_motion::MotionType::kLin,
          massage_motion::PoseTarget{end_precontact_pose},
          request.free_space_velocity_scale,
          request.free_space_acceleration_scale);
        if (!plan_and_execute(
            inter_cycle_retreat, MassageTaskState::kInterCycleRetreat,
            true, true))
        {
          break;
        }

        const auto inter_cycle_return = motion_request(
          "inter_cycle_return_" + std::to_string(repetition + 1U),
          massage_motion::MotionType::kLin,
          massage_motion::PoseTarget{precontact_pose},
          request.free_space_velocity_scale,
          request.free_space_acceleration_scale);
        if (!plan_and_execute(
            inter_cycle_return, MassageTaskState::kInterCycleReturn,
            true, true))
        {
          break;
        }
        recovery_surface_pose = first_contact_pose;
      }
    }
  }

  if (compliant_contact)
  {
    for (std::size_t repetition = 0U;
      repetition < repetition_count &&
      result.primary_error == MassageTaskError::kNone; ++repetition)
    {
    auto search_plan = plan_motion(search);
    if (!search_plan.success ||
      search_plan.trajectory.joint_trajectory.points.empty())
    {
      fail(
        MassageTaskError::kPlanningFailed,
        "第 " + std::to_string(repetition + 1U) +
        " 次接触搜索规划失败: " + search_plan.message);
      break;
    }
    if (!validate_alignment(
        search_plan, true,
        "guarded_contact_entry_" + std::to_string(repetition + 1U)))
    {
      break;
    }

    auto compliance_request = request.compliance_request;
    compliance_request.request_id += "_repeat_" +
      std::to_string(repetition + 1U);
    result.compliance_result = compliance_->start(compliance_request);
    compliance_started = result.compliance_result.success;
    if (!compliance_started)
    {
      fail(MassageTaskError::kComplianceFailed, result.compliance_result.message);
      break;
    }

    bool cycle_contact_detected = false;
    bool cycle_target_force_reached = false;
    std::size_t next_search_index = 0U;
    transition(MassageTaskState::kGuardedContactEntry);
    stream_plan(
      search_plan, true, false, 0U, &next_search_index,
      cycle_contact_detected, cycle_target_force_reached);
    if (result.primary_error == MassageTaskError::kNone &&
      !cycle_contact_detected)
    {
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(request.contact_wait_timeout);
      while (std::chrono::steady_clock::now() < deadline &&
        !cycle_contact_detected)
      {
        if (cancel_requested_.load())
        {
          fail(MassageTaskError::kCanceled, "等待接触时任务被取消");
          break;
        }
        if (!monitor_feedback(
            cycle_contact_detected, cycle_target_force_reached))
        {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (result.primary_error == MassageTaskError::kNone &&
        !cycle_contact_detected)
      {
        fail(
          MassageTaskError::kContactNotDetected,
          "第 " + std::to_string(repetition + 1U) +
          " 次接触搜索结束仍未检测到人体接触");
      }
    }
    if (result.primary_error != MassageTaskError::kNone) break;

    transition(MassageTaskState::kContactConfirmed);
    transition(MassageTaskState::kRampNormalForce);
    if (!cycle_target_force_reached &&
      next_search_index < search_plan.trajectory.joint_trajectory.points.size())
    {
      stream_plan(
        search_plan, false, true, next_search_index, &next_search_index,
        cycle_contact_detected, cycle_target_force_reached);
    }
    const auto force_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(request.force_ramp_timeout);
    while (result.primary_error == MassageTaskError::kNone &&
      std::chrono::steady_clock::now() < force_deadline &&
      !cycle_target_force_reached)
    {
      if (cancel_requested_.load())
      {
        fail(MassageTaskError::kCanceled, "建立法向力时任务被取消");
        break;
      }
      if (!monitor_feedback(
          cycle_contact_detected, cycle_target_force_reached))
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (result.primary_error == MassageTaskError::kNone &&
      !cycle_target_force_reached)
    {
      fail(
        MassageTaskError::kTargetForceNotReached,
        "第 " + std::to_string(repetition + 1U) +
        " 次法向力未在规定时间内建立");
      break;
    }

    const auto live_feedback = compliance_->feedback();
    const auto & search_joint_names =
      search_plan.trajectory.joint_trajectory.joint_names;
    const auto & live_joint_names = live_feedback.joint_names.empty() ?
      search_joint_names : live_feedback.joint_names;
    if (live_feedback.stale ||
      live_feedback.status != massage_motion::ComplianceStatus::kActive ||
      live_joint_names.size() != live_feedback.joint_positions.size() ||
      live_joint_names.empty())
    {
      fail(
        MassageTaskError::kComplianceFailed,
        "建立手法规划起点时实时关节反馈无效");
      break;
    }

    moveit_msgs::msg::RobotState live_start;
    live_start.joint_state.name = live_joint_names;
    live_start.joint_state.position = live_feedback.joint_positions;
    live_start.is_diff = true;
    auto technique_plan = technique_planner_->plan(
      result.path_result.path, live_start, true);
    result.last_plan_result = technique_plan;
    if (!technique_plan.success ||
      technique_plan.trajectory.joint_trajectory.points.empty())
    {
      fail(
        MassageTaskError::kPlanningFailed,
        "第 " + std::to_string(repetition + 1U) +
        " 次手法轨迹规划失败: " + technique_plan.message);
      break;
    }
    if (!validate_alignment(
        technique_plan, true,
        "execute_technique_" + std::to_string(repetition + 1U)))
    {
      break;
    }

    transition(MassageTaskState::kExecuteTechnique);
    if (!stream_plan(
        technique_plan, false, false, 0U, nullptr,
        cycle_contact_detected, cycle_target_force_reached))
    {
      break;
    }
    ++result.completed_repetitions;
    switch (request.technique)
    {
      case massage_motion::TechniquePathType::kPush:
        ++result.completed_technique_cycles;
        break;
      case massage_motion::TechniquePathType::kPress:
        result.completed_technique_cycles += request.press.cycles;
        break;
      case massage_motion::TechniquePathType::kKnead:
        result.completed_technique_cycles += request.knead.cycles;
        break;
    }
    recovery_surface_pose = last_contact_pose;

    if (repetition + 1U < repetition_count)
    {
      transition(MassageTaskState::kReleaseNormalForce);
      result.compliance_result = compliance_->stop();
      compliance_started = false;
      if (!result.compliance_result.success)
      {
        fail(
          MassageTaskError::kComplianceFailed,
          "轮间退出柔顺控制失败: " + result.compliance_result.message);
        break;
      }

      auto end_precontact_pose = last_contact_pose;
      end_precontact_pose.pose.position.x += nx * request.precontact_distance;
      end_precontact_pose.pose.position.y += ny * request.precontact_distance;
      end_precontact_pose.pose.position.z += nz * request.precontact_distance;
      const auto inter_cycle_retreat = motion_request(
        "inter_cycle_retreat_" + std::to_string(repetition + 1U),
        massage_motion::MotionType::kLin,
        massage_motion::PoseTarget{end_precontact_pose},
        request.contact_velocity_scale);
      if (!plan_and_execute(
          inter_cycle_retreat, MassageTaskState::kInterCycleRetreat,
          true, true))
      {
        break;
      }
      recovery_surface_pose = last_contact_pose;

      const auto inter_cycle_return = motion_request(
        "inter_cycle_return_" + std::to_string(repetition + 1U),
        massage_motion::MotionType::kLin,
        massage_motion::PoseTarget{precontact_pose},
        request.free_space_velocity_scale,
        request.free_space_acceleration_scale);
      if (!plan_and_execute(
          inter_cycle_return, MassageTaskState::kInterCycleReturn,
          true, true))
      {
        break;
      }
      recovery_surface_pose = first_contact_pose;
    }
    }
  }

  result.recovery_attempted =
    scene_prepared || route_progressed || robot_moved || compliance_started;
  bool recovery_ok = true;
  if (compliant_contact && (compliance_started ||
    compliance_->status() == massage_motion::ComplianceStatus::kActive))
  {
    transition(MassageTaskState::kReleaseNormalForce);
    result.compliance_result = compliance_->stop();
    // 安全监控主动停止后会返回原始超限/超时错误，但只要控制权回切本身
    // 没有失败，恢复动作仍然有效。业务错误继续保留在 primary_error。
    recovery_ok =
      (result.compliance_result.success ||
      result.compliance_result.error !=
      massage_motion::ComplianceError::kControlFailed) && recovery_ok;
    compliance_started = false;
  }
  if (contact_allowed)
  {
    recovery_ok = scene_manager_->allow_tool_contact(false) && recovery_ok;
    contact_allowed = false;
  }
  if (precontact_reached)
  {
    auto recovery_raise_pose = recovery_surface_pose;
    const double recovery_clearance =
      request.precontact_distance + request.work_ready_clearance;
    recovery_raise_pose.pose.position.x += nx * recovery_clearance;
    recovery_raise_pose.pose.position.y += ny * recovery_clearance;
    recovery_raise_pose.pose.position.z += nz * recovery_clearance;
    const auto recovery_raise = motion_request(
      "recovery_raise", massage_motion::MotionType::kLin,
      massage_motion::PoseTarget{recovery_raise_pose},
      request.free_space_velocity_scale,
      request.free_space_acceleration_scale);
    transition(MassageTaskState::kRetreat);
    const auto retreat_plan = plan_motion(recovery_raise);
    recovery_ok = retreat_plan.success &&
      validate_alignment(retreat_plan, true, "retreat") &&
      execute_plan(retreat_plan, "retreat", &recovery_raise) && recovery_ok;

    const double overhead_distance = std::hypot(
      recovery_raise_pose.pose.position.x - work_ready_pose.pose.position.x,
      recovery_raise_pose.pose.position.y - work_ready_pose.pose.position.y);
    if (recovery_ok && overhead_distance > 1.0e-6)
    {
      const auto overhead_return = motion_request(
        "return_overhead", massage_motion::MotionType::kLin,
        massage_motion::PoseTarget{work_ready_pose},
        request.free_space_velocity_scale,
        request.free_space_acceleration_scale);
      transition(MassageTaskState::kReturnOverhead);
      const auto overhead_plan = plan_motion(overhead_return);
      recovery_ok = overhead_plan.success &&
        validate_alignment(overhead_plan, true, "return_overhead") &&
        execute_plan(overhead_plan, "return_overhead", &overhead_return) && recovery_ok;
    }
    if (recovery_ok)
    {
      auto safe_exit_pose = effective_request.contact_pose;
      const double exit_tangent =
        protected_tangent_min - request.safe_return_exit_margin;
      safe_exit_pose.pose.position.x +=
        tx * exit_tangent + nx * recovery_clearance;
      safe_exit_pose.pose.position.y +=
        ty * exit_tangent + ny * recovery_clearance;
      safe_exit_pose.pose.position.z +=
        tz * exit_tangent + nz * recovery_clearance;
      const auto safe_exit = motion_request(
        "safe_return_exit", massage_motion::MotionType::kLin,
        massage_motion::PoseTarget{safe_exit_pose},
        request.free_space_velocity_scale,
        request.free_space_acceleration_scale);
      transition(MassageTaskState::kMoveSafeReturnExit);
      const auto safe_exit_plan = plan_motion(safe_exit);
      recovery_ok = safe_exit_plan.success &&
        validate_alignment(safe_exit_plan, true, "safe_return_exit") &&
        execute_plan(safe_exit_plan, "safe_return_exit", &safe_exit) && recovery_ok;
    }
  }
  if (route_progressed && recovery_ok && request.return_to_standby)
  {
    const auto standby_plan = plan_motion(standby);
    if (standby_plan.success)
    {
      ReturnTrajectorySafetyConfig safety_config;
      safety_config.minimum_tool_z = 0.0;
      safety_config.minimum_diagnostic_link_z =
        request.safe_return_minimum_diagnostic_link_z;
      safety_config.surface_origin = {{
          effective_request.contact_pose.pose.position.x,
          effective_request.contact_pose.pose.position.y,
          effective_request.contact_pose.pose.position.z}};
      safety_config.surface_normal = {{nx, ny, nz}};
      safety_config.surface_tangent = {{tx, ty, tz}};
      safety_config.protected_tangent_min = protected_tangent_min;
      safety_config.protected_tangent_max = protected_tangent_max;
      safety_config.protected_lateral_half_width =
        protected_lateral_half_width;
      safety_config.minimum_surface_clearance =
        request.safe_return_minimum_tool_clearance;
      auto safety_result = return_safety_validator_->validate(
        standby_plan.trajectory, safety_config);
      result.return_safety_results.push_back(safety_result);
      if (!safety_result.valid || !safety_result.accepted)
      {
        fail(
          MassageTaskError::kExecutionFailed,
          "回待机轨迹高度门禁失败: " + safety_result.message);
      }
      recovery_ok = safety_result.valid && safety_result.accepted &&
        execute_plan(standby_plan, "return_standby", &standby) && recovery_ok;
    }
    else
    {
      fail(MassageTaskError::kPlanningFailed, "上方回撤后的回待机规划失败");
      recovery_ok = false;
    }
  }
  if (scene_prepared)
  {
    recovery_ok = scene_manager_->restore() && recovery_ok;
  }
  if (compliant_contact &&
    compliance_->status() == massage_motion::ComplianceStatus::kStopped)
  {
    recovery_ok = compliance_->reset() && recovery_ok;
  }
  result.recovery_succeeded = recovery_ok;

  if (result.primary_error == MassageTaskError::kNone && recovery_ok)
  {
    transition(MassageTaskState::kComplete);
    result.success = true;
    result.final_state = state_.load();
    result.message = plan_only ? "推拿任务完整流程只规划成功" :
      (free_space ? "推拿任务自由空间完整流程执行成功" :
      "推拿任务柔顺接触完整流程执行成功");
    return result;
  }
  if (!recovery_ok)
  {
    result.error = MassageTaskError::kRecoveryFailed;
    result.message = result.primary_message + "；控制器、退出运动或场景恢复失败";
    transition(MassageTaskState::kFault);
  }
  else if (result.primary_error == MassageTaskError::kCanceled)
  {
    result.error = result.primary_error;
    result.message = result.primary_message;
    transition(MassageTaskState::kCanceled);
  }
  else
  {
    result.error = result.primary_error;
    result.message = result.primary_message;
    transition(MassageTaskState::kFault);
  }
  result.final_state = state_.load();
  return result;
}

bool MassageTaskStateMachine::cancel()
{
  const auto current = state_.load();
  if (current == MassageTaskState::kIdle || current == MassageTaskState::kComplete ||
    current == MassageTaskState::kCanceled || current == MassageTaskState::kFault)
  {
    return false;
  }
  cancel_requested_.store(true);
  if (executor_)
  {
    executor_->cancel();
  }
  if (compliance_ &&
    compliance_->status() == massage_motion::ComplianceStatus::kActive)
  {
    compliance_->stop();
  }
  return true;
}

bool MassageTaskStateMachine::reset()
{
  const auto current = state_.load();
  if (current != MassageTaskState::kIdle && current != MassageTaskState::kComplete &&
    current != MassageTaskState::kCanceled && current != MassageTaskState::kFault)
  {
    return false;
  }
  cancel_requested_.store(false);
  state_.store(MassageTaskState::kIdle);
  return true;
}

MassageTaskState MassageTaskStateMachine::state() const
{
  return state_.load();
}

std::string to_string(MassageTaskState state)
{
  switch (state)
  {
    case MassageTaskState::kIdle: return "idle";
    case MassageTaskState::kReady: return "ready";
    case MassageTaskState::kMoveStandby: return "move_standby";
    case MassageTaskState::kMoveWorkReady: return "move_work_ready";
    case MassageTaskState::kVerifyToolAlignment: return "verify_tool_alignment";
    case MassageTaskState::kMovePreContact: return "move_pre_contact";
    case MassageTaskState::kMoveTechniqueStart: return "move_technique_start";
    case MassageTaskState::kZeroAndValidateFt: return "zero_and_validate_ft";
    case MassageTaskState::kGuardedContactEntry: return "guarded_contact_entry";
    case MassageTaskState::kContactConfirmed: return "contact_confirmed";
    case MassageTaskState::kRampNormalForce: return "ramp_normal_force";
    case MassageTaskState::kExecuteTechnique: return "execute_technique";
    case MassageTaskState::kReleaseNormalForce: return "release_normal_force";
    case MassageTaskState::kInterCycleRetreat: return "inter_cycle_retreat";
    case MassageTaskState::kInterCycleReturn: return "inter_cycle_return";
    case MassageTaskState::kRetreat: return "retreat";
    case MassageTaskState::kReturnOverhead: return "return_overhead";
    case MassageTaskState::kMoveSafeReturnExit: return "move_safe_return_exit";
    case MassageTaskState::kComplete: return "complete";
    case MassageTaskState::kCanceled: return "canceled";
    case MassageTaskState::kFault: return "fault";
    default: return "unknown";
  }
}

std::string to_string(MassageExecutionMode mode)
{
  switch (mode)
  {
    case MassageExecutionMode::kPlanOnly: return "plan_only";
    case MassageExecutionMode::kFreeSpace: return "free_space";
    case MassageExecutionMode::kCompliantContact: return "compliant_contact";
    default: return "unknown";
  }
}

bool parse_execution_mode(
  const std::string & value, MassageExecutionMode * mode)
{
  if (!mode) return false;
  if (value == "plan_only")
  {
    *mode = MassageExecutionMode::kPlanOnly;
    return true;
  }
  if (value == "free_space")
  {
    *mode = MassageExecutionMode::kFreeSpace;
    return true;
  }
  if (value == "compliant_contact")
  {
    *mode = MassageExecutionMode::kCompliantContact;
    return true;
  }
  return false;
}

}  // namespace massage_task
