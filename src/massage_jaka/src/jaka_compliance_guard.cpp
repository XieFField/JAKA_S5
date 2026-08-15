#include "massage_jaka/jaka_compliance_guard.hpp"

#include <algorithm>
#include <cmath>

namespace massage_jaka
{

ComplianceDisplacementResult calculate_compliance_displacement(
    const std::vector<double> & initial_joints,
    const std::vector<double> & current_joints,
    const std::array<double, 3> & initial_translation,
    const std::array<double, 3> & current_translation)
{
    ComplianceDisplacementResult result;
    if (initial_joints.empty() ||
        initial_joints.size() != current_joints.size())
    {
        result.message = "初始和当前关节数组长度不一致";
        return result;
    }

    for (std::size_t index = 0; index < initial_joints.size(); ++index)
    {
        if (!std::isfinite(initial_joints[index]) ||
            !std::isfinite(current_joints[index]))
        {
            result.message = "关节数组包含非有限数值";
            return result;
        }
        result.maximum_joint_displacement = std::max(
            result.maximum_joint_displacement,
            std::abs(current_joints[index] - initial_joints[index]));
    }

    double squared_distance = 0.0;
    for (std::size_t axis = 0; axis < initial_translation.size(); ++axis)
    {
        if (!std::isfinite(initial_translation[axis]) ||
            !std::isfinite(current_translation[axis]))
        {
            result.message = "TCP 平移包含非有限数值";
            return result;
        }
        const double difference =
            current_translation[axis] - initial_translation[axis];
        squared_distance += difference * difference;
    }

    result.linear_displacement = std::sqrt(squared_distance);
    result.valid = true;
    result.message = "柔顺位移有效";
    return result;
}

ComplianceGuardResult evaluate_compliance_guard(
    const std::vector<double> & initial_joints,
    const std::vector<double> & current_joints,
    const std::array<double, 3> & initial_translation,
    const std::array<double, 3> & current_translation,
    const std::array<double, 6> & wrench,
    const std::array<double, 6> & maximum_absolute_wrench,
    double maximum_joint_displacement,
    double maximum_linear_displacement)
{
    ComplianceGuardResult result;
    result.displacement = calculate_compliance_displacement(
        initial_joints,
        current_joints,
        initial_translation,
        current_translation);
    if (!result.displacement.valid ||
        !std::isfinite(maximum_joint_displacement) ||
        maximum_joint_displacement <= 0.0 ||
        !std::isfinite(maximum_linear_displacement) ||
        maximum_linear_displacement <= 0.0)
    {
        result.violation = ComplianceGuardViolation::kInvalidSample;
        result.message = result.displacement.valid ?
            "柔顺位移上限必须是有限正数" : result.displacement.message;
        return result;
    }

    if (result.displacement.maximum_joint_displacement >
        maximum_joint_displacement)
    {
        result.valid = true;
        result.limit_exceeded = true;
        result.violation = ComplianceGuardViolation::kJointDisplacement;
        result.message = "关节位移超过柔顺请求上限";
        return result;
    }
    if (result.displacement.linear_displacement > maximum_linear_displacement)
    {
        result.valid = true;
        result.limit_exceeded = true;
        result.violation = ComplianceGuardViolation::kLinearDisplacement;
        result.message = "TCP 直线位移超过柔顺请求上限";
        return result;
    }

    for (std::size_t axis = 0; axis < wrench.size(); ++axis)
    {
        if (!std::isfinite(wrench[axis]) ||
            !std::isfinite(maximum_absolute_wrench[axis]) ||
            maximum_absolute_wrench[axis] < 0.0)
        {
            result.violation = ComplianceGuardViolation::kInvalidSample;
            result.wrench_axis = axis;
            result.message = "力反馈或力限幅包含无效数值";
            return result;
        }
        // 上限为零表示该轴没有配置项目层限幅；驱动层软限幅仍独立生效。
        if (maximum_absolute_wrench[axis] > 0.0 &&
            std::abs(wrench[axis]) > maximum_absolute_wrench[axis])
        {
            result.valid = true;
            result.limit_exceeded = true;
            result.violation = ComplianceGuardViolation::kWrench;
            result.wrench_axis = axis;
            result.message = "六维力反馈超过柔顺请求上限";
            return result;
        }
    }

    result.valid = true;
    result.message = "柔顺反馈处于请求上限内";
    return result;
}

}  // namespace massage_jaka
