#include "massage_motion/ptp_planner.hpp"

namespace massage_motion
{

MotionType PtpPlanner::supported_motion_type() const
{
    return MotionType::kPtp;
}

std::string PtpPlanner::planner_id() const
{
    return "PTP";
}

ValidationResult PtpPlanner::configure_target(const MotionRequest & request)
{
    if(const auto * joint_target = 
        std::get_if<JointTarget>(&request.target))
    {
        // 关节目标必须覆盖规划组中的全部变量。

        const auto expected_joint_count = 
            move_group().getVariableCount();

        if(joint_target->positions.size() != expected_joint_count)
        {
            return {
                false,
                MotionError::kInvalidRequest,
                "JointTarget 的关节数量与规划组的关节数量不匹配"
            };
        }

        const bool target_set =
            move_group().setJointValueTarget(joint_target->positions);

        if(!target_set)
        {
            return {
                false,
                MotionError::kInvalidRequest,
                "关节目标超出关节限制"
            };
        }

        return {
            true,
            MotionError::kNone,
            "PTP关节目标设置成功"
        };
    }

    if(const auto * pose_target = std::get_if<PoseTarget>(&request.target))
    {
        // 位姿目标交给 MoveIt 求解逆运动学，并作为 PTP 的终点。
        const bool target_set =
            move_group().setPoseTarget(pose_target->pose);

        if(!target_set)
        {
            return {
                false,
                MotionError::kInvalidRequest,
                "PTP末端位姿目标设置失败"
            };
        }

        return {
            true,
            MotionError::kNone,
            "PTP位姿目标设置成功"
        };
    }

    if (const auto * named_target =
        std::get_if<NamedJointTarget>(&request.target))
    {
        if (!move_group().setNamedTarget(named_target->name))
        {
            return {
                false,
                MotionError::kInvalidRequest,
                "SRDF 中不存在命名关节目标: " + named_target->name
            };
        }

        return {
            true,
            MotionError::kNone,
            "PTP 命名关节目标设置成功: " + named_target->name
        };
    }

    // 即使前置校验未来发生变化，本层仍保证拒绝无法解释的目标类型。
    return {
        false,
        MotionError::kTargetTypeMismatch,
        "PtpPlanner 仅支持 JointTarget、NamedJointTarget 和 PoseTarget"
    };
}

}  // namespace massage_motion
