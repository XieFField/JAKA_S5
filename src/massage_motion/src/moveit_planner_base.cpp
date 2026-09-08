#include "massage_motion/moveit_planner_base.hpp"

namespace massage_motion
{

PlanResult MoveItPlannerBase::plan(const MotionRequest & request)
{
    // 校验请求
    const auto validation_result = validate_motion_request(request);

    if(!validation_result.valid)
    {
        RCLCPP_INFO(
            this->logger_,
            "[Motion request fail] 错误码:%d, 错误信息:%s",
            static_cast<std::int32_t>(validation_result.error),
            validation_result.message.c_str()
        );

        return {
            false,
            validation_result.error,
            0,
            validation_result.message,
            moveit_msgs::msg::RobotTrajectory{},
            0.0,
            this->planner_id().c_str(),
            {}
        };
    }
    // 检查请求的运动类型是否受支持
    if(request.motion_type != this->supported_motion_type())
    {
        RCLCPP_INFO(
            this->logger_,
            "[Motion request fail] 错误码:%d, 错误信息:%s",
            static_cast<std::int32_t>(MotionError::kUnsupportedMotion),
            "不支持的运动类型"
        );

        return {
            false,
            MotionError::kUnsupportedMotion,
            0,
            "不支持的运动类型",
            moveit_msgs::msg::RobotTrajectory{},
            0.0,
            this->planner_id().c_str(),
            {}
        };
    }

    // 清除上一次的目标和路径约束
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    // 多段规划使用上一段终点；普通单段请求仍从当前监控状态开始。
    if (request.start_state.has_value())
    {
        move_group_->setStartState(*request.start_state);
    }
    else
    {
        move_group_->setStartStateToCurrentState();
    }

    // 应用 PlannerConfig 中的配置
    move_group_->setPlanningPipelineId(planner_config_.planning_pipeline);
    move_group_->setPlannerId(planner_id());
    move_group_->setEndEffectorLink(planner_config_.end_effector_link);
    move_group_->setPoseReferenceFrame(planner_config_.reference_frame);

    // 设置请求参数
    move_group_->setMaxVelocityScalingFactor(request.velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(request.acceleration_scale);
    move_group_->setPlanningTime(request.planning_timeout);

    //设置MotionTarget给Moveit
    const auto target_result = this->configure_target(request);

    if (!target_result.valid)
    {
        return {
            false,
            target_result.error,
            0,
            target_result.message,
            moveit_msgs::msg::RobotTrajectory{},
            0.0,
            this->planner_id(),
            {}
        };
    }

    MoveGroupInterface::Plan moveit_plan;

    const auto error_code = move_group_->plan(moveit_plan);

    if(error_code.val != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(
            this->logger_,
            "[Motion plan fail] 错误码:%d, 错误信息:%s",
            error_code.val,
            moveit::core::error_code_to_string(error_code).c_str()
        );

        return {
            false,
            MotionError::kPlanningFailed,
            error_code.val,
            moveit::core::error_code_to_string(error_code),
            moveit_msgs::msg::RobotTrajectory{},
            0.0,
            this->planner_id().c_str(),
            {}
        };
    }

    // 检查空轨迹
    if(moveit_plan.trajectory_.joint_trajectory.points.empty() &&
       moveit_plan.trajectory_.multi_dof_joint_trajectory.points.empty())
    {
        RCLCPP_INFO(
            this->logger_,
            "[Motion plan fail] 错误码:%d, 错误信息:%s",
            static_cast<std::int32_t>(MotionError::kEmptyTrajectory),
            "规划结果为空轨迹"
        );

        return {
            false,
            MotionError::kEmptyTrajectory,
            error_code.val,
            "规划结果为空轨迹",
            moveit_msgs::msg::RobotTrajectory{},
            0.0,
            this->planner_id().c_str(),
            {}
        };
    }

    return {
        true,
        MotionError::kNone,
        error_code.val,
        moveit::core::error_code_to_string(error_code),
        moveit_plan.trajectory_,
        moveit_plan.planning_time_,
        this->planner_id().c_str(),
        {}
    };
}

}
