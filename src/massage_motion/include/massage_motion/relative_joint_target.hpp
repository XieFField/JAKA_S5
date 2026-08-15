#ifndef MASSAGE_MOTION__RELATIVE_JOINT_TARGET_HPP_
#define MASSAGE_MOTION__RELATIVE_JOINT_TARGET_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_motion/motion_types.hpp"

namespace massage_motion
{

struct RelativeJointTargetResult
{
    ValidationResult validation;
    JointTarget target;
};

// 按 expected_joint_names 的顺序读取当前关节状态，并只给指定关节增加相对量。
RelativeJointTargetResult make_relative_joint_target(
    const sensor_msgs::msg::JointState & current_state,
    const std::vector<std::string> & expected_joint_names,
    const std::string & selected_joint,
    double delta,
    double maximum_absolute_delta);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__RELATIVE_JOINT_TARGET_HPP_
