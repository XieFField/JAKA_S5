#include "massage_motion/execution_validation.hpp"
#include <cmath>
namespace massage_motion
{

ExecutionValidationResult validate_execution_request(
    const ExecutionRequest & request)
{
    // 检查timeout 是否有限且大于0
    if (request.timeout <= 0.0 || !std::isfinite(request.timeout))
    {
        return {
            false, 
            ExecutionError::kInvalidRequest, 
            "timeout 必须是有限且大于0的数值"
        };
    }

    // 检查robot_trajectory 是否没有joint_trajectory 或 multi_dof_joint_trajectory
    if (request.robot_trajectory.joint_trajectory.points.empty() &&
        request.robot_trajectory.multi_dof_joint_trajectory.points.empty())
    {
        return {
            false, 
            ExecutionError::kEmptyTrajectory, 
            "robot_trajectory 必须包含至少一个 joint_trajectory 或 multi_dof_joint_trajectory"
        };
    }

    return {
        true,
        ExecutionError::kNone,
        "请求有效"
    };
}

}