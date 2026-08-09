#include "massage_motion/circ_planner.hpp"

namespace massage_motion
{
MotionType CircPlanner::supported_motion_type() const
{
    return MotionType::kCirc;
}

std::string CircPlanner::planner_id() const
{
    return "CIRC";
}

ValidationResult CircPlanner::configure_target(const MotionRequest & request)
{
    if(const auto *circular_target = 
        std::get_if<CircularTarget>(&request.target))
    {
        // 最终终点
        const bool goal_set
            = move_group().setPoseTarget(circular_target->goal_pose);

        if(!goal_set)
        {
            return {
                false,
                MotionError::kInvalidRequest,
                "CIRC末端位姿目标设置失败"
            };
        }

        // 定义中间点
        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "interim";

        moveit_msgs::msg::PositionConstraint interim_constraint;
        
        interim_constraint.header = circular_target->interim_pose.header;
        interim_constraint.link_name = move_group().getEndEffectorLink();
        interim_constraint.constraint_region.primitive_poses.push_back(
            circular_target->interim_pose.pose
        );
        interim_constraint.weight = 1.0;

        path_constraints.position_constraints.push_back(
            interim_constraint
        );

        move_group().setPathConstraints(path_constraints);

        return {
            true,
            MotionError::kNone,
            "CIRC位姿目标设置成功"
        };
    }

    return {
        false,
        MotionError::kTargetTypeMismatch,
        "CIRC规划器只支持 CircularTarget 类型的目标"
    };
}


}