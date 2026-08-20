#include "massage_motion/technique_path_generator.hpp"

#include <cmath>
#include <cstddef>

namespace massage_motion
{
namespace
{

constexpr double kDirectionEpsilon = 1.0e-12;
constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr std::size_t kMaximumPathPoints = 100000U;

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool unit_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return std::isfinite(norm_squared) &&
         std::abs(norm_squared - 1.0) <= kQuaternionNormTolerance;
}

TechniquePathResult invalid_result(
  TechniquePathError error, const std::string & message)
{
  TechniquePathResult result;
  result.error = error;
  result.message = message;
  return result;
}

}  // namespace

TechniquePathResult TechniquePathGenerator::generate_push(
  const PushPathRequest & request)
{
  if (request.start_pose.header.frame_id.empty())
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest, "推法起点必须包含参考坐标系");
  }
  if (!finite_pose(request.start_pose.pose) ||
    !unit_quaternion(request.start_pose.pose.orientation))
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest, "推法起点位姿必须有限且四元数为单位四元数");
  }
  if (!std::isfinite(request.direction_x) ||
    !std::isfinite(request.direction_y))
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest, "推法 XY 方向必须是有限数值");
  }

  const double direction_norm = std::hypot(
    request.direction_x, request.direction_y);
  if (!std::isfinite(direction_norm) || direction_norm <= kDirectionEpsilon)
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest, "推法 XY 方向不能为零向量");
  }
  if (!std::isfinite(request.length) || request.length <= 0.0 ||
    !std::isfinite(request.speed) || request.speed <= 0.0 ||
    !std::isfinite(request.sample_period) || request.sample_period <= 0.0 ||
    !std::isfinite(request.maximum_speed) || request.maximum_speed <= 0.0)
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest,
      "推法长度、速度、采样周期和速度上限必须是有限正数");
  }
  if (request.speed > request.maximum_speed)
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest, "推法名义速度超过速度上限");
  }

  const double duration = request.length / request.speed;
  const double raw_segment_count = std::ceil(duration / request.sample_period);
  if (!std::isfinite(duration) || duration <= 0.0 ||
    !std::isfinite(raw_segment_count) || raw_segment_count < 1.0)
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest, "推法持续时间或采样数量无效");
  }
  if (raw_segment_count > static_cast<double>(kMaximumPathPoints - 1U))
  {
    return invalid_result(
      TechniquePathError::kPathTooLarge, "推法采样点数量超过离线路径上限");
  }

  const auto segment_count = static_cast<std::size_t>(raw_segment_count);
  const double segment_duration = duration / static_cast<double>(segment_count);
  const double unit_x = request.direction_x / direction_norm;
  const double unit_y = request.direction_y / direction_norm;

  TechniquePathResult result;
  result.path.type = TechniquePathType::kPush;
  result.path.reference_frame = request.start_pose.header.frame_id;
  result.path.reference_stamp = request.start_pose.header.stamp;
  result.path.duration = duration;
  result.path.nominal_speed = request.speed;
  result.path.points.reserve(segment_count + 1U);

  for (std::size_t index = 0; index <= segment_count; ++index)
  {
    const double progress =
      static_cast<double>(index) / static_cast<double>(segment_count);
    TechniquePathPoint point;
    point.pose = request.start_pose.pose;
    point.pose.position.x += unit_x * request.length * progress;
    point.pose.position.y += unit_y * request.length * progress;
    point.time_from_start = segment_duration * static_cast<double>(index);
    result.path.points.push_back(point);
  }

  result.success = true;
  result.error = TechniquePathError::kNone;
  result.message = "参考系 XY 直线推法路径生成成功";
  return result;
}

std::string to_string(TechniquePathType type)
{
  switch (type)
  {
    case TechniquePathType::kPush:
      return "push";
    default:
      return "unknown";
  }
}

std::string to_string(TechniquePathError error)
{
  switch (error)
  {
    case TechniquePathError::kNone:
      return "none";
    case TechniquePathError::kInvalidRequest:
      return "invalid_request";
    case TechniquePathError::kPathTooLarge:
      return "path_too_large";
    default:
      return "unknown";
  }
}

}  // namespace massage_motion
