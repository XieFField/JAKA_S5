#include "massage_motion/compliance_validation.hpp"

#include <cmath>

namespace massage_motion
{

ComplianceValidationResult validate_compliance_request(
    const ComplianceRequest & request)
{
    if (request.request_id.empty())
    {
        return {false, ComplianceError::kInvalidRequest, "request_id 不能为空"};
    }

    bool has_enabled_axis = false;
    for (std::size_t index = 0; index < kCartesianDof; ++index)
    {
        has_enabled_axis = has_enabled_axis || request.enabled_axes[index];

        if (!std::isfinite(request.target_wrench[index]) ||
            !std::isfinite(request.max_absolute_wrench[index]) ||
            request.max_absolute_wrench[index] < 0.0)
        {
            return {
                false,
                ComplianceError::kInvalidRequest,
                "力/力矩目标和上限必须是有限数值，且上限不能小于零"};
        }

        if (request.enabled_axes[index] &&
            request.max_absolute_wrench[index] <= 0.0)
        {
            return {
                false,
                ComplianceError::kInvalidRequest,
                "每个启用轴都必须设置大于零的绝对力/力矩上限"};
        }

        if (std::abs(request.target_wrench[index]) >
            request.max_absolute_wrench[index])
        {
            return {
                false,
                ComplianceError::kInvalidRequest,
                "目标力/力矩不能超过对应的绝对上限"};
        }
    }

    if (!has_enabled_axis)
    {
        return {false, ComplianceError::kInvalidRequest, "至少需要启用一个柔顺轴"};
    }

    if (!std::isfinite(request.max_joint_displacement) ||
        request.max_joint_displacement <= 0.0)
    {
        return {
            false,
            ComplianceError::kInvalidRequest,
            "max_joint_displacement 必须是有限正数"};
    }

    if (!std::isfinite(request.max_linear_displacement) ||
        request.max_linear_displacement <= 0.0)
    {
        return {
            false,
            ComplianceError::kInvalidRequest,
            "max_linear_displacement 必须是有限正数"};
    }

    if (!std::isfinite(request.timeout) || request.timeout <= 0.0)
    {
        return {false, ComplianceError::kInvalidRequest, "timeout 必须是有限正数"};
    }

    return {true, ComplianceError::kNone, "请求有效"};
}

}  // namespace massage_motion
