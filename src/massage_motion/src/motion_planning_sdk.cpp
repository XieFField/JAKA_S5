#include "massage_motion/motion_planning_sdk.hpp"

namespace massage_motion
{

MotionPlanningSdk::MotionPlanningSdk(
        const rclcpp::Node::SharedPtr & node,
        const PlannerConfig & planner_config)
{
    planners_.emplace(
        MotionType::kPtp,
        std::make_unique<PtpPlanner>(node, planner_config)
    );

    planners_.emplace(
        MotionType::kLin,
        std::make_unique<LinPlanner>(node, planner_config)
    );

    planners_.emplace(
        MotionType::kCirc,
        std::make_unique<CircPlanner>(node, planner_config)
    );
}

PlanResult MotionPlanningSdk::plan(const MotionRequest & request)
{
    const auto iterator = planners_.find(request.motion_type);
    if (iterator == planners_.end() || !iterator->second)
    {
        return {
            false,
            MotionError::kUnsupportedMotion,
            0,
            "没有注册对应动作类型的规划器",
            moveit_msgs::msg::RobotTrajectory{},
            0.0,
            "",
            {}
        };
    }

    return iterator->second->plan(request);
}

}
