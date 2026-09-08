#include "massage_motion/wrench_frame_transform.hpp"

#include <algorithm>
#include <cmath>

#include "Eigen/Geometry"

namespace massage_motion
{

WrenchFrameTransformResult transform_wrench_to_reference(
  const WrenchFrameTransformRequest & request)
{
  WrenchFrameTransformResult result;
  if (!std::all_of(
      request.source_wrench.begin(), request.source_wrench.end(),
      [](double value) {return std::isfinite(value);}))
  {
    result.message = "源六维力包含非有限数值";
    return result;
  }
  const auto & quaternion = request.target_from_source_rotation;
  const Eigen::Quaterniond rotation(
    quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  const double norm = rotation.norm();
  if (!std::isfinite(norm) || norm <= 1.0e-12 ||
    std::abs(norm - 1.0) > 1.0e-3)
  {
    result.message = "坐标变换四元数无效或未归一化";
    return result;
  }
  const Eigen::Vector3d source_origin(
    request.source_origin_in_target.x,
    request.source_origin_in_target.y,
    request.source_origin_in_target.z);
  const Eigen::Vector3d reference_origin(
    request.reference_origin_in_target.x,
    request.reference_origin_in_target.y,
    request.reference_origin_in_target.z);
  if (!source_origin.allFinite() || !reference_origin.allFinite())
  {
    result.message = "源或目标参考点包含非有限数值";
    return result;
  }

  const Eigen::Vector3d source_force(
    request.source_wrench[0], request.source_wrench[1],
    request.source_wrench[2]);
  const Eigen::Vector3d source_torque(
    request.source_wrench[3], request.source_wrench[4],
    request.source_wrench[5]);
  const Eigen::Vector3d target_force = rotation * source_force;
  const Eigen::Vector3d lever_arm = source_origin - reference_origin;
  const Eigen::Vector3d target_torque =
    rotation * source_torque + lever_arm.cross(target_force);
  if (!target_force.allFinite() || !target_torque.allFinite())
  {
    result.message = "六维力变换结果不是有限数值";
    return result;
  }
  result.wrench = {
    target_force.x(), target_force.y(), target_force.z(),
    target_torque.x(), target_torque.y(), target_torque.z()};
  result.valid = true;
  result.message = "六维力已变换到目标坐标轴和参考点";
  return result;
}

}  // namespace massage_motion
