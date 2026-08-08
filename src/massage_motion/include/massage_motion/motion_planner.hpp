#ifndef MASSAGE_MOTION__MOTION_PLANNER_HPP_
#define MASSAGE_MOTION__MOTION_PLANNER_HPP_

#include "massage_motion/motion_types.hpp"

namespace massage_motion
{

// 业务代码只依赖该接口，不直接依赖 MoveGroupInterface。
// 仿真实现可以使用 MoveIt，后续也可以接入其他规划后端。
class IMotionPlanner
{
public:
  virtual ~IMotionPlanner() = default;

  // 该函数只负责生成轨迹，不允许在规划器内部执行轨迹。
  virtual PlanResult plan(const MotionRequest & request) = 0;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__MOTION_PLANNER_HPP_
