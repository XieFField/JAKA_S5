#include "massage_task/press_task_state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
        static_cast<double>(duration.nanosec) * 1e-9;
}

bool finite_positive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

}  // namespace

bool offset_pose_along_normal(
    const massage_motion::PoseTarget & source,
    const geometry_msgs::msg::Vector3 & normal,
    double distance,
    massage_motion::PoseTarget & target,
    std::string & error_message)
{
    const double norm = std::sqrt(
        normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (source.pose.header.frame_id.empty())
    {
        error_message = "接触目标缺少参考坐标系";
        return false;
    }
    if (!std::isfinite(norm) || norm <= 1e-9 || !finite_positive(distance))
    {
        error_message = "表面法向必须有限且非零，偏移距离必须为有限正数";
        return false;
    }

    target = source;
    target.pose.pose.position.x += normal.x / norm * distance;
    target.pose.pose.position.y += normal.y / norm * distance;
    target.pose.pose.position.z += normal.z / norm * distance;
    error_message.clear();
    return true;
}

PressTaskStateMachine::PressTaskStateMachine(
    std::shared_ptr<massage_motion::IMotionPlanner> planner,
    std::shared_ptr<massage_motion::ITrajectoryExecutor> executor,
    std::shared_ptr<massage_motion::IComplianceController> compliance,
    std::shared_ptr<IContactSceneManager> scene_manager)
    : planner_(std::move(planner)),
      executor_(std::move(executor)),
      compliance_(std::move(compliance)),
      scene_manager_(std::move(scene_manager))
{
    if (!planner_ || !executor_ || !compliance_ || !scene_manager_)
    {
        throw std::invalid_argument("按压状态机依赖不能为空");
    }
}

PressTaskResult PressTaskStateMachine::run(const PressTaskRequest & request)
{
    std::unique_lock<std::mutex> run_lock(run_mutex_, std::try_to_lock);
    if (!run_lock.owns_lock() || state_.load() != PressTaskState::kIdle)
    {
        PressTaskResult busy_result;
        busy_result.final_state = state_.load();
        busy_result.error = PressTaskError::kBusy;
        busy_result.primary_error = PressTaskError::kBusy;
        busy_result.message = "按压状态机正在执行其他任务或尚未复位";
        busy_result.primary_message = busy_result.message;
        return busy_result;
    }

    PressTaskResult result;
    cancel_requested_.store(false);

    massage_motion::PoseTarget approach_target;
    massage_motion::PoseTarget precontact_target;
    std::string validation_message;
    const bool valid_request =
        !request.task_id.empty() &&
        request.contact_axis < massage_motion::kCartesianDof &&
        request.approach_distance > request.precontact_distance &&
        finite_positive(request.precontact_distance) &&
        finite_positive(request.approach_velocity_scale) &&
        request.approach_velocity_scale <= 1.0 &&
        finite_positive(request.precontact_velocity_scale) &&
        request.precontact_velocity_scale <= 1.0 &&
        finite_positive(request.press_velocity_scale) &&
        request.press_velocity_scale <= 1.0 &&
        finite_positive(request.planning_timeout) &&
        finite_positive(request.execution_timeout) &&
        finite_positive(request.contact_threshold) &&
        finite_positive(request.maximum_contact_wrench) &&
        request.contact_threshold < request.maximum_contact_wrench &&
        finite_positive(request.contact_wait_timeout) &&
        std::isfinite(request.hold_duration) &&
        request.hold_duration >= 0.0 &&
        request.compliance_request.enabled_axes[request.contact_axis] &&
        request.compliance_request.max_absolute_wrench[request.contact_axis] >=
            request.maximum_contact_wrench &&
        offset_pose_along_normal(
            request.contact_target,
            request.surface_normal,
            request.approach_distance,
            approach_target,
            validation_message) &&
        offset_pose_along_normal(
            request.contact_target,
            request.surface_normal,
            request.precontact_distance,
            precontact_target,
            validation_message);

    if (!valid_request)
    {
        state_.store(PressTaskState::kFault);
        result.final_state = PressTaskState::kFault;
        result.error = PressTaskError::kInvalidRequest;
        result.primary_error = result.error;
        result.message = validation_message.empty() ?
            "按压任务参数无效" : validation_message;
        result.primary_message = result.message;
        return result;
    }

    bool scene_prepared = false;
    bool tool_contact_allowed = false;
    bool robot_moved = false;
    bool compliance_started = false;

    auto set_primary_failure = [&](PressTaskError error, const std::string & message)
    {
        if (result.primary_error == PressTaskError::kNone)
        {
            result.primary_error = error;
            result.primary_message = message;
        }
    };

    auto make_motion_request = [&](
        const std::string & suffix,
        massage_motion::MotionType motion_type,
        const massage_motion::PoseTarget & target,
        double scale)
    {
        massage_motion::MotionRequest motion_request;
        motion_request.request_id = request.task_id + "_" + suffix;
        motion_request.motion_type = motion_type;
        motion_request.target = target;
        motion_request.velocity_scale = scale;
        motion_request.acceleration_scale = scale;
        motion_request.planning_timeout = request.planning_timeout;
        motion_request.avoid_collisions = true;
        return motion_request;
    };

    auto plan_and_execute = [&](
        const massage_motion::MotionRequest & motion_request,
        PressTaskState plan_state,
        PressTaskState execute_state,
        bool honor_cancel) -> bool
    {
        if (honor_cancel && cancel_requested_.load())
        {
            set_primary_failure(PressTaskError::kCanceled, "任务在规划前被取消");
            return false;
        }

        state_.store(plan_state);
        result.last_plan_result = planner_->plan(motion_request);
        if (!result.last_plan_result.success)
        {
            set_primary_failure(
                PressTaskError::kPlanningFailed,
                result.last_plan_result.message);
            return false;
        }

        if (honor_cancel && cancel_requested_.load())
        {
            set_primary_failure(PressTaskError::kCanceled, "任务在执行前被取消");
            return false;
        }

        massage_motion::ExecutionRequest execution_request;
        execution_request.request_id = motion_request.request_id + "_execution";
        execution_request.robot_trajectory = result.last_plan_result.trajectory;
        execution_request.timeout = request.execution_timeout;
        state_.store(execute_state);
        result.last_execution_result = executor_->execute(execution_request);
        if (!result.last_execution_result.success)
        {
            const auto error =
                result.last_execution_result.status ==
                    massage_motion::ExecutionStatus::kCanceled ?
                PressTaskError::kCanceled : PressTaskError::kExecutionFailed;
            set_primary_failure(error, result.last_execution_result.message);
            return false;
        }
        robot_moved = true;
        return true;
    };

    if (!scene_manager_->prepare())
    {
        set_primary_failure(
            PressTaskError::kSceneFailed,
            "添加接触工装到 Planning Scene 失败");
    }
    else
    {
        scene_prepared = true;
    }

    const auto approach_request = make_motion_request(
        "approach", massage_motion::MotionType::kPtp,
        approach_target, request.approach_velocity_scale);
    const auto precontact_request = make_motion_request(
        "precontact", massage_motion::MotionType::kLin,
        precontact_target, request.precontact_velocity_scale);
    const auto press_request = make_motion_request(
        "press", massage_motion::MotionType::kLin,
        request.contact_target, request.press_velocity_scale);
    const auto retreat_request = make_motion_request(
        "retreat", massage_motion::MotionType::kLin,
        approach_target, request.precontact_velocity_scale);

    if (result.primary_error == PressTaskError::kNone &&
        !plan_and_execute(
            approach_request,
            PressTaskState::kPlanApproach,
            PressTaskState::kExecuteApproach,
            true))
    {
        // 失败信息已经由 plan_and_execute 保存。
    }

    if (result.primary_error == PressTaskError::kNone &&
        !plan_and_execute(
            precontact_request,
            PressTaskState::kPlanPrecontact,
            PressTaskState::kExecutePrecontact,
            true))
    {
        // 失败信息已经由 plan_and_execute 保存。
    }

    if (result.primary_error == PressTaskError::kNone)
    {
        if (!scene_manager_->allow_tool_contact(true))
        {
            set_primary_failure(
                PressTaskError::kSceneFailed,
                "无法只允许按摩头与接触工装发生碰撞");
        }
        else
        {
            tool_contact_allowed = true;
        }
    }

    if (result.primary_error == PressTaskError::kNone)
    {
        state_.store(PressTaskState::kPlanPress);
        result.last_plan_result = planner_->plan(press_request);
        if (!result.last_plan_result.success)
        {
            set_primary_failure(
                PressTaskError::kPlanningFailed,
                result.last_plan_result.message);
        }
    }

    if (result.primary_error == PressTaskError::kNone)
    {
        state_.store(PressTaskState::kCompliantPress);
        result.compliance_result = compliance_->start(request.compliance_request);
        compliance_started = result.compliance_result.success;
        if (!compliance_started)
        {
            set_primary_failure(
                PressTaskError::kComplianceFailed,
                result.compliance_result.message);
        }
    }

    if (result.primary_error == PressTaskError::kNone)
    {
        const auto & trajectory = result.last_plan_result.trajectory.joint_trajectory;
        if (trajectory.points.empty())
        {
            set_primary_failure(
                PressTaskError::kPlanningFailed,
                "按压名义轨迹为空");
        }
        else
        {
            const auto started_at = std::chrono::steady_clock::now();
            for (const auto & point : trajectory.points)
            {
                std::this_thread::sleep_until(
                    started_at + std::chrono::duration<double>(
                        duration_seconds(point.time_from_start)));

                if (cancel_requested_.load())
                {
                    set_primary_failure(PressTaskError::kCanceled, "柔顺按压被取消");
                    break;
                }

                const auto feedback = compliance_->feedback();
                if (feedback.stale ||
                    feedback.status != massage_motion::ComplianceStatus::kActive)
                {
                    set_primary_failure(
                        PressTaskError::kComplianceFailed,
                        "柔顺反馈超时或控制器提前退出");
                    break;
                }

                const double wrench = std::abs(
                    feedback.wrench[request.contact_axis]);
                result.peak_absolute_wrench = std::max(
                    result.peak_absolute_wrench, wrench);
                if (wrench >= request.contact_threshold)
                {
                    result.contact_detected = true;
                    break;
                }

                massage_motion::ComplianceReference reference;
                reference.joint_names = trajectory.joint_names;
                reference.positions = point.positions;
                reference.velocities = point.velocities;
                reference.time_from_start = duration_seconds(point.time_from_start);
                if (!compliance_->update_reference(reference))
                {
                    set_primary_failure(
                        PressTaskError::kComplianceFailed,
                        "柔顺控制器拒绝名义关节参考");
                    break;
                }
            }
        }
    }

    if (result.primary_error == PressTaskError::kNone && !result.contact_detected)
    {
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(request.contact_wait_timeout);
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (cancel_requested_.load())
            {
                set_primary_failure(PressTaskError::kCanceled, "等待接触时被取消");
                break;
            }
            const auto feedback = compliance_->feedback();
            if (feedback.stale ||
                feedback.status != massage_motion::ComplianceStatus::kActive)
            {
                set_primary_failure(
                    PressTaskError::kComplianceFailed,
                    "等待接触期间柔顺反馈失效");
                break;
            }
            const double wrench = std::abs(
                feedback.wrench[request.contact_axis]);
            result.peak_absolute_wrench = std::max(
                result.peak_absolute_wrench, wrench);
            if (wrench >= request.contact_threshold)
            {
                result.contact_detected = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (result.primary_error == PressTaskError::kNone &&
            !result.contact_detected)
        {
            set_primary_failure(
                PressTaskError::kContactNotDetected,
                "名义按压轨迹结束后仍未检测到接触");
        }
    }

    if (result.primary_error == PressTaskError::kNone)
    {
        const auto hold_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(request.hold_duration);
        while (std::chrono::steady_clock::now() < hold_deadline)
        {
            if (cancel_requested_.load())
            {
                set_primary_failure(PressTaskError::kCanceled, "按压保持阶段被取消");
                break;
            }
            const auto feedback = compliance_->feedback();
            if (feedback.stale ||
                feedback.status != massage_motion::ComplianceStatus::kActive)
            {
                set_primary_failure(
                    PressTaskError::kComplianceFailed,
                    "按压保持阶段柔顺反馈失效");
                break;
            }
            result.peak_absolute_wrench = std::max(
                result.peak_absolute_wrench,
                std::abs(feedback.wrench[request.contact_axis]));
            if (result.peak_absolute_wrench >= request.maximum_contact_wrench)
            {
                set_primary_failure(
                    PressTaskError::kComplianceFailed,
                    "按压峰值达到任务层安全上限");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    result.recovery_attempted = scene_prepared || robot_moved || compliance_started;
    bool recovery_ok = true;
    if (compliance_started ||
        compliance_->status() == massage_motion::ComplianceStatus::kActive)
    {
        result.compliance_result = compliance_->stop();
        compliance_started = false;
        result.peak_absolute_wrench = std::max(
            result.peak_absolute_wrench,
            result.compliance_result
                .peak_absolute_wrench[request.contact_axis]);
        if (!result.compliance_result.success)
        {
            if (result.primary_error == PressTaskError::kNone)
            {
                set_primary_failure(
                    PressTaskError::kComplianceFailed,
                    result.compliance_result.message);
            }
            // 力超限或超时仍可能已经成功切回轨迹控制器；只有控制权
            // 回切本身失败时，才把停止步骤判定为恢复失败。
            recovery_ok =
                result.compliance_result.error !=
                    massage_motion::ComplianceError::kControlFailed &&
                recovery_ok;
        }
    }

    if (robot_moved)
    {
        state_.store(PressTaskState::kRetreat);
        const bool retreat_ok = plan_and_execute(
            retreat_request,
            PressTaskState::kRetreat,
            PressTaskState::kRetreat,
            false);
        recovery_ok = recovery_ok && retreat_ok;
    }

    if (tool_contact_allowed)
    {
        recovery_ok = scene_manager_->allow_tool_contact(false) && recovery_ok;
    }
    if (scene_prepared)
    {
        recovery_ok = scene_manager_->restore() && recovery_ok;
    }
    if (compliance_->status() == massage_motion::ComplianceStatus::kStopped)
    {
        recovery_ok = compliance_->reset() && recovery_ok;
    }

    result.recovery_succeeded = recovery_ok;
    result.recovery_message = recovery_ok ?
        "控制器、接触权限和退出轨迹均已恢复" :
        "至少一个停止、回切、退出或场景恢复步骤失败";

    if (result.primary_error == PressTaskError::kNone && recovery_ok)
    {
        result.success = true;
        result.final_state = PressTaskState::kCompleted;
        result.error = PressTaskError::kNone;
        result.message = "柔顺按压任务完成并安全退出";
        state_.store(PressTaskState::kCompleted);
        return result;
    }

    result.success = false;
    if (!recovery_ok)
    {
        result.error = PressTaskError::kRecoveryFailed;
        result.message = result.primary_message + "；" + result.recovery_message;
        result.final_state = PressTaskState::kFault;
        state_.store(PressTaskState::kFault);
        return result;
    }

    result.error = result.primary_error;
    result.message = result.primary_message;
    if (result.primary_error == PressTaskError::kCanceled)
    {
        result.final_state = PressTaskState::kCanceled;
        state_.store(PressTaskState::kCanceled);
    }
    else
    {
        result.final_state = PressTaskState::kFault;
        state_.store(PressTaskState::kFault);
    }
    return result;
}

bool PressTaskStateMachine::cancel()
{
    const auto current_state = state_.load();
    if (current_state == PressTaskState::kIdle ||
        current_state == PressTaskState::kCompleted ||
        current_state == PressTaskState::kCanceled ||
        current_state == PressTaskState::kFault)
    {
        return false;
    }

    cancel_requested_.store(true);
    if (current_state == PressTaskState::kExecuteApproach ||
        current_state == PressTaskState::kExecutePrecontact ||
        current_state == PressTaskState::kRetreat)
    {
        executor_->cancel();
    }
    if (current_state == PressTaskState::kCompliantPress)
    {
        compliance_->stop();
    }
    return true;
}

bool PressTaskStateMachine::reset()
{
    const auto current_state = state_.load();
    if (current_state == PressTaskState::kPlanApproach ||
        current_state == PressTaskState::kExecuteApproach ||
        current_state == PressTaskState::kPlanPrecontact ||
        current_state == PressTaskState::kExecutePrecontact ||
        current_state == PressTaskState::kPlanPress ||
        current_state == PressTaskState::kCompliantPress ||
        current_state == PressTaskState::kRetreat)
    {
        return false;
    }
    cancel_requested_.store(false);
    state_.store(PressTaskState::kIdle);
    return true;
}

PressTaskState PressTaskStateMachine::state() const
{
    return state_.load();
}

}  // namespace massage_task
