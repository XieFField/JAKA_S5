#ifndef MASSAGE_MOTION__PTP_PLANNER_HPP_
#define MASSAGE_MOTION__PTP_PLANNER_HPP_

#include <string>

#include "massage_motion/moveit_planner_base.hpp"

namespace massage_motion
{

// 配置 Pilz PTP 目标；公共校验和规划流程由 MoveItPlannerBase 负责。
class PtpPlanner : public MoveItPlannerBase
{
public:
    using MoveItPlannerBase::MoveItPlannerBase;

protected:
    MotionType supported_motion_type() const override;
    std::string planner_id() const override;

    ValidationResult configure_target(
        const MotionRequest & request) override;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__PTP_PLANNER_HPP_
