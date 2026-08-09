#ifndef MOVEIT_PLANNER_BASE_HPP_
#define MOVEIT_PLANNER_BASE_HPP_

#pragma once

#include <memory>
#include <string>
#include <utility>

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
    // 返回当前具体规划器唯一支持的动作类型。
    virtual MotionType supported_motion_type() const = 0;

    virtual std::string planner_id() const = 0; //返回 ptp lin circ

    virtual ValidationResult configure_target(
        const MotionRequest & request  //设置MotionTarget给Moveit
    ) = 0;

    MoveGroupInterface & move_group() { return *move_group_; }

private:
    std::shared_ptr<MoveGroupInterface> move_group_;

    PlannerConfig planner_config_;
    rclcpp::Logger logger_;
public:
    MoveItPlannerBase(
    const rclcpp::Node::SharedPtr & node,
    PlannerConfig planner_config)
    : planner_config_(std::move(planner_config)),
      logger_(node->get_logger())
    {
        move_group_ = 
            std::make_shared<MoveGroupInterface>(
                node, 
                planner_config_.planning_group
            );
    }
};

} // namespace massage_motion
#endif  // MOVEIT_PLANNER_BASE_HPP_
