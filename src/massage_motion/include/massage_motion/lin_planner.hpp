#ifndef LIN_PLANNER_HPP_
#define LIN_PLANNER_HPP_

#pragma once

#include <string>
#include <variant>
#include "massage_motion/moveit_planner_base.hpp"

namespace massage_motion
{

class LinPlanner : public MoveItPlannerBase
{
public:
    using MoveItPlannerBase::MoveItPlannerBase;

protected:
    MotionType supported_motion_type() const override;
    std::string planner_id() const override;

    ValidationResult configure_target(
        const MotionRequest & request) override;

private:

};

}

#endif // LIN_PLANNER_HPP_