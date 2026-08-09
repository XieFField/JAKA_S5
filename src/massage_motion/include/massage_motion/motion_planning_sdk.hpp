#ifndef MOTION_PLANNING_SDK_HPP_
#define MOTION_PLANNING_SDK_HPP_

#include <map>
#include <memory>

#include "massage_motion/motion_types.hpp"
#include "massage_motion/lin_planner.hpp"
#include "massage_motion/circ_planner.hpp"
#include "massage_motion/ptp_planner.hpp"
#include "massage_motion/moveit_planner_base.hpp"

namespace massage_motion
{


class MotionPlanningSdk final : public IMotionPlanner
{
public:
    MotionPlanningSdk(
        const rclcpp::Node::SharedPtr & node,
        const PlannerConfig & planner_config);

    PlanResult plan(const MotionRequest & request) override;
protected:

private:
    std::map<MotionType, std::shared_ptr<IMotionPlanner>> planners_;
};

} // namespace massage_motion


#endif // MOTION_PLANNING_SDK_HPP_