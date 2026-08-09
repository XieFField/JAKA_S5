#include "massage_motion/lin_planner.hpp"

namespace massage_motion
{

MotionType LinPlanner::supported_motion_type() const
{
    return MotionType::kLin;
}

std::string LinPlanner::planner_id() const
{
    return "LIN";
}

ValidationResult LinPlanner::configure_target(const MotionRequest & request)
{
    if(const auto *pose_target = 
        std::get_if<PoseTarget>(&request.target))
    {
        // 位姿目标交给 MoveIt 求解逆运动学，并作为 LIN 的终点。
        const bool target_set =
            move_group().setPoseTarget(pose_target->pose);

        if(!target_set)
        {
            return {
                false,
                MotionError::kInvalidRequest,
                "LIN末端位姿目标设置失败"
            };
        }

        return {
            true,
            MotionError::kNone,
            "LIN位姿目标设置成功"
        };
    }

    return {
        false,
        MotionError::kTargetTypeMismatch,
        "LIN规划器只支持 PoseTarget 类型的目标"
    };
}

}