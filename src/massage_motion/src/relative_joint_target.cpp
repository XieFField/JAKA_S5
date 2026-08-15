#include "massage_motion/relative_joint_target.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace massage_motion
{

RelativeJointTargetResult make_relative_joint_target(
    const sensor_msgs::msg::JointState & current_state,
    const std::vector<std::string> & expected_joint_names,
    const std::string & selected_joint,
    double delta,
    double maximum_absolute_delta)
{
    RelativeJointTargetResult result;
    const auto invalid = [&result](const std::string & message)
    {
        result.validation = {
            false, MotionError::kInvalidRequest, message};
        return result;
    };

    if (expected_joint_names.empty())
    {
        return invalid("期望关节列表不能为空");
    }
    const std::set<std::string> expected_unique(
        expected_joint_names.begin(), expected_joint_names.end());
    if (expected_unique.size() != expected_joint_names.size())
    {
        return invalid("期望关节列表不能包含重复名称");
    }
    if (selected_joint.empty() ||
        expected_unique.find(selected_joint) == expected_unique.end())
    {
        return invalid("指定关节不属于当前规划组");
    }
    if (!std::isfinite(delta) || delta == 0.0)
    {
        return invalid("关节相对增量必须是非零有限数值");
    }
    if (!std::isfinite(maximum_absolute_delta) ||
        maximum_absolute_delta <= 0.0 ||
        std::abs(delta) > maximum_absolute_delta)
    {
        return invalid("关节相对增量超过允许范围");
    }
    if (current_state.name.size() != current_state.position.size())
    {
        return invalid("JointState 的名称和位置数组长度不一致");
    }
    const std::set<std::string> actual_unique(
        current_state.name.begin(), current_state.name.end());
    if (actual_unique.size() != current_state.name.size())
    {
        return invalid("JointState 包含重复关节名称");
    }

    result.target.positions.reserve(expected_joint_names.size());
    for (const auto & joint_name : expected_joint_names)
    {
        const auto iterator = std::find(
            current_state.name.begin(), current_state.name.end(), joint_name);
        if (iterator == current_state.name.end())
        {
            return invalid("JointState 缺少规划组关节: " + joint_name);
        }
        const auto index = static_cast<std::size_t>(
            std::distance(current_state.name.begin(), iterator));
        const double position = current_state.position[index];
        if (!std::isfinite(position))
        {
            return invalid("JointState 包含非有限关节位置");
        }
        const double target_position =
            position + (joint_name == selected_joint ? delta : 0.0);
        if (!std::isfinite(target_position))
        {
            return invalid("相对关节目标计算结果不是有限数值");
        }
        result.target.positions.push_back(target_position);
    }

    result.validation = {
        true, MotionError::kNone, "相对关节目标有效"};
    return result;
}

}  // namespace massage_motion
