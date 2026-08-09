#ifndef CIRC_PLANNER_HPP_
#define CIRC_PLANNER_HPP_

#include <string>
#include "massage_motion/moveit_planner_base.hpp"
#include <variant>

namespace massage_motion
{

class CircPlanner : public MoveItPlannerBase
{
public:
    using MoveItPlannerBase::MoveItPlannerBase;

protected:
    MotionType supported_motion_type() const override;
    std::string planner_id() const override;
    ValidationResult configure_target(
        const MotionRequest & request) override;
};


}

#endif // CIRC_PLANNER_HPP_