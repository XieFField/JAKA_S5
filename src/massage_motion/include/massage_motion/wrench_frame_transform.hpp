#ifndef MASSAGE_MOTION__WRENCH_FRAME_TRANSFORM_HPP_
#define MASSAGE_MOTION__WRENCH_FRAME_TRANSFORM_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "massage_motion/wrench_processor.hpp"

namespace massage_motion
{

struct WrenchFrameTransformRequest
{
  WrenchVector source_wrench{};
  // target_R_source：把源坐标系向量旋转到目标表达坐标系。
  geometry_msgs::msg::Quaternion target_from_source_rotation;
  // 两个原点都用目标坐标系表达。输出力矩以 reference_origin 为取矩点。
  geometry_msgs::msg::Point source_origin_in_target;
  geometry_msgs::msg::Point reference_origin_in_target;
};

struct WrenchFrameTransformResult
{
  bool valid{false};
  std::string message;
  WrenchVector wrench{};
};

// 空间力变换：F_t = R F_s，tau_ref = R tau_s +
// (p_source - p_reference) x F_t。
WrenchFrameTransformResult transform_wrench_to_reference(
  const WrenchFrameTransformRequest & request);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__WRENCH_FRAME_TRANSFORM_HPP_
