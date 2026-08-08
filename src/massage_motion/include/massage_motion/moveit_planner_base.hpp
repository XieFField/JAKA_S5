#ifndef MOVEIT_PLANNER_BASE_HPP_
#define MOVEIT_PLANNER_BASE_HPP_

#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "massage_motion/motion_types.hpp"
#include "massage_motion/motion_planner.hpp"
#include "massage_motion/motion_validation.hpp"

namespace massage_motion
{

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class MoveItPlannerBase : public massage_motion::IMotionPlanner
{
public:
    PlanResult plan(const MotionRequest & request) override;

protected:
    virtual void supported_motion_types(std::vector<MotionType> & types);
    
    virtual std::string planner_id() = 0; //返回 ptp lin circ

    virtual ValidationResult configure_target(
        const MotionRequest & request  //设置MotionTarget给Moveit
    ) = 0;

private:
    std::shared_ptr<MoveGroupInterface> move_group_;

    PlannerConfig planner_config_;
    const rclcpp::Node::SharedPtr & logger_;
public:
    MoveItPlannerBase(const rclcpp::Node::SharedPtr & node,
        PlannerConfig planner_config
    ) 
        : logger_ (node->get_logger()), 
        planner_config_(planner_config)
    {
        move_group_ = 
            std::make_shared<MoveGroupInterface>(
                node, 
                planner_config_.planning_group
            )
        
        ;
    }
};

} // namespace massage_motion
#endif  // MOVEIT_PLANNER_BASE_HPP_