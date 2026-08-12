#ifndef MOVEIT_TRAJECTORY_EXECUTOR_HPP_
#define MOVEIT_TRAJECTORY_EXECUTOR_HPP_

#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "std_msgs/msg/string.hpp"

#include "massage_motion/execution_validation.hpp"
#include "massage_motion/trajectory_executor.hpp"

namespace massage_motion
{

class MoveItTrajectoryExecutor final : public ITrajectoryExecutor
{
public:
    using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
    using GoalHandle = 
        rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

    explicit MoveItTrajectoryExecutor(
        const rclcpp::Node::SharedPtr & node,
        const std::string & action_name = "/execute_trajectory");

    ExecutionResult execute(
        const ExecutionRequest & request) override;

    bool cancel() override;

    ExecutionStatus status() const override;

private:
    rclcpp_action::Client<ExecuteTrajectory>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        execution_event_publisher_;
    rclcpp::Logger logger_;

    // 保证同一个执行器只能执行一条轨迹
    std::mutex execution_mutex_;

    // 保护actic_goal_
    mutable std::mutex goal_mutex_;
    GoalHandle::SharedPtr active_goal_;

    std::atomic<ExecutionStatus> status_{ExecutionStatus::kIdle};
};// class MoveItTrajectoryExecutor

}// namespace massage_motion

#endif // MOVEIT_TRAJECTORY_EXECUTOR_HPP_
