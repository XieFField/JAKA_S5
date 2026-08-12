#include "massage_motion/trajectory_endpoint_error.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <utility>

namespace massage_motion
{

TrajectoryEndpointErrorResult calculate_trajectory_endpoint_error(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const sensor_msgs::msg::JointState & actual_state)
{
    const auto & joint_trajectory = trajectory.joint_trajectory;

    if (joint_trajectory.points.empty())
    {
        return {false, "规划轨迹没有轨迹点", {}, 0.0};
    }

    if (joint_trajectory.joint_names.empty())
    {
        return {false, "规划轨迹没有关节名称", {}, 0.0};
    }

    const auto & last_point = joint_trajectory.points.back();

    if (joint_trajectory.joint_names.size() != last_point.positions.size())
    {
        return {false, "规划关节名称数量与轨迹末点位置数量不匹配", {}, 0.0};
    }

    if (actual_state.name.size() != actual_state.position.size())
    {
        return {false, "实际关节状态名称数量与位置数量不匹配", {}, 0.0};
    }

    std::unordered_map<std::string, double> actual_joint_positions;
    actual_joint_positions.reserve(actual_state.name.size());

    for (std::size_t i = 0; i < actual_state.name.size(); ++i)
    {
        const auto inserted = actual_joint_positions.emplace(
            actual_state.name[i], actual_state.position[i]);

        if (!inserted.second)
        {
            return {
                false,
                "实际关节状态中存在重复关节名称: " + actual_state.name[i],
                {},
                0.0
            };
        }
    }

    std::vector<JointPositionError> joint_errors;
    joint_errors.reserve(joint_trajectory.joint_names.size());
    double max_absolute_error = 0.0;

    for (std::size_t i = 0; i < joint_trajectory.joint_names.size(); ++i)
    {
        const auto & joint_name = joint_trajectory.joint_names[i];
        const auto actual_iterator = actual_joint_positions.find(joint_name);

        if (actual_iterator == actual_joint_positions.end())
        {
            return {
                false,
                "实际关节状态缺少规划关节: " + joint_name,
                {},
                0.0
            };
        }

        const double target_position = last_point.positions[i];
        const double actual_position = actual_iterator->second;

        if (!std::isfinite(target_position) || !std::isfinite(actual_position))
        {
            return {
                false,
                "关节位置包含非有限数值: " + joint_name,
                {},
                0.0
            };
        }

        const double absolute_error =
            std::abs(target_position - actual_position);

        joint_errors.push_back({
            joint_name,
            target_position,
            actual_position,
            absolute_error
        });

        max_absolute_error = std::max(max_absolute_error, absolute_error);
    }

    return {
        true,
        "计算成功",
        std::move(joint_errors),
        max_absolute_error
    };
}

}  // namespace massage_motion
