#ifndef MASSAGE_MOTION__MOTION_PLANNER_HPP_
#define MASSAGE_MOTION__MOTION_PLANNER_HPP_

#include "massage_motion/motion_types.hpp"

namespace massage_motion
{

class IMotionPlanner
{
public:
  virtual ~IMotionPlanner() = default;

  virtual PlanResult plan(const MotionRequest & request) = 0;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__MOTION_PLANNER_HPP_

