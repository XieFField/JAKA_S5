#include "massage_motion/technique_path_generator.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>

namespace massage_motion
{
namespace
{

constexpr double kDirectionEpsilon = 1.0e-12;
constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr double kPi = 3.14159265358979323846;
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

bool valid_sampling_request(
  double duration, double sample_period, double maximum_speed,
  double nominal_speed, std::size_t & segment_count)
{
  if (!std::isfinite(duration) || duration <= 0.0 ||
    !std::isfinite(sample_period) || sample_period <= 0.0 ||
    !std::isfinite(maximum_speed) || maximum_speed <= 0.0 ||
    !std::isfinite(nominal_speed) || nominal_speed < 0.0 ||
    nominal_speed > maximum_speed)
  {
    return false;
  }
  const double raw_segment_count = std::ceil(duration / sample_period);
  if (!std::isfinite(raw_segment_count) || raw_segment_count < 1.0 ||
    raw_segment_count > static_cast<double>(kMaximumPathPoints - 1U))
  {
    return false;
  }
  segment_count = static_cast<std::size_t>(raw_segment_count);
  return true;
}

std::string sampling_diagnostic(
  const std::string & technique, double duration, double sample_period,
  double nominal_speed, double maximum_speed)
{
  const double raw_segments =
    sample_period > 0.0 ? std::ceil(duration / sample_period) :
    std::numeric_limits<double>::quiet_NaN();
  std::ostringstream message;
  message << technique << "采样校验失败: duration=" << duration
          << " s, sample_period=" << sample_period
          << " s, nominal_speed=" << nominal_speed
          << " m/s, maximum_speed=" << maximum_speed
          << " m/s, raw_segments=" << raw_segments
          << ", maximum_segments=" << (kMaximumPathPoints - 1U);
  return message.str();
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

TechniquePathResult TechniquePathGenerator::generate_press(
  const PressPathRequest & request)
{
  if (request.contact_pose.header.frame_id.empty() ||
    !finite_pose(request.contact_pose.pose) ||
    !unit_quaternion(request.contact_pose.pose.orientation))
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest,
      "按法接触基准必须包含参考系、有限位姿和单位四元数");
  }
  const double normal_norm = std::sqrt(
    request.surface_normal_x * request.surface_normal_x +
    request.surface_normal_y * request.surface_normal_y +
    request.surface_normal_z * request.surface_normal_z);
  if (!std::isfinite(normal_norm) || normal_norm <= kDirectionEpsilon ||
    !std::isfinite(request.stroke) || request.stroke <= 0.0 ||
    !std::isfinite(request.cycle_duration) || request.cycle_duration <= 0.0 ||
    request.cycles == 0U)
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest,
      "按法法向、行程、周期和循环次数无效");
  }
  const double duration = request.cycle_duration *
    static_cast<double>(request.cycles);
  const double nominal_speed = kPi * request.stroke / request.cycle_duration;
  std::size_t segment_count = 0U;
  if (!valid_sampling_request(
      duration, request.sample_period, request.maximum_speed,
      nominal_speed, segment_count))
  {
    const double raw_points = request.sample_period > 0.0 ?
      std::ceil(duration / request.sample_period) :
      std::numeric_limits<double>::quiet_NaN();
    return invalid_result(
      std::isfinite(raw_points) &&
      raw_points > static_cast<double>(kMaximumPathPoints - 1U) ?
      TechniquePathError::kPathTooLarge : TechniquePathError::kInvalidRequest,
      sampling_diagnostic(
        "按法", duration, request.sample_period, nominal_speed,
        request.maximum_speed));
  }

  const double nx = request.surface_normal_x / normal_norm;
  const double ny = request.surface_normal_y / normal_norm;
  const double nz = request.surface_normal_z / normal_norm;
  const double segment_duration = duration / static_cast<double>(segment_count);
  TechniquePathResult result;
  result.path.type = TechniquePathType::kPress;
  result.path.reference_frame = request.contact_pose.header.frame_id;
  result.path.reference_stamp = request.contact_pose.header.stamp;
  result.path.duration = duration;
  result.path.nominal_speed = nominal_speed;
  result.path.points.reserve(segment_count + 1U);
  for (std::size_t index = 0; index <= segment_count; ++index)
  {
    const double time = segment_duration * static_cast<double>(index);
    const double phase = 2.0 * kPi * time / request.cycle_duration;
    const double displacement = 0.5 * request.stroke * (1.0 - std::cos(phase));
    TechniquePathPoint point;
    point.pose = request.contact_pose.pose;
    point.pose.position.x -= nx * displacement;
    point.pose.position.y -= ny * displacement;
    point.pose.position.z -= nz * displacement;
    point.time_from_start = time;
    result.path.points.push_back(point);
  }
  result.success = true;
  result.message = "接触基准内半余弦周期按法路径生成成功";
  return result;
}

TechniquePathResult TechniquePathGenerator::generate_knead(
  const KneadPathRequest & request)
{
  if (request.center_pose.header.frame_id.empty() ||
    !finite_pose(request.center_pose.pose) ||
    !unit_quaternion(request.center_pose.pose.orientation))
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest,
      "揉法圆心必须包含参考系、有限位姿和单位四元数");
  }
  const double direction_norm = std::hypot(
    request.initial_direction_x, request.initial_direction_y);
  if (!std::isfinite(direction_norm) ||
    direction_norm <= kDirectionEpsilon ||
    !std::isfinite(request.radius) || request.radius <= 0.0 ||
    !std::isfinite(request.cycle_duration) || request.cycle_duration <= 0.0 ||
    request.cycles == 0U)
  {
    return invalid_result(
      TechniquePathError::kInvalidRequest,
      "揉法初始方向、半径、周期或循环次数无效");
  }
  const double duration = request.cycle_duration *
    static_cast<double>(request.cycles);
  const double nominal_speed = 2.0 * kPi * request.radius /
    request.cycle_duration;
  std::size_t segment_count = 0U;
  if (!valid_sampling_request(
      duration, request.sample_period, request.maximum_speed,
      nominal_speed, segment_count))
  {
    const double raw_points = request.sample_period > 0.0 ?
      std::ceil(duration / request.sample_period) :
      std::numeric_limits<double>::quiet_NaN();
    return invalid_result(
      std::isfinite(raw_points) &&
      raw_points > static_cast<double>(kMaximumPathPoints - 1U) ?
      TechniquePathError::kPathTooLarge : TechniquePathError::kInvalidRequest,
      sampling_diagnostic(
        "揉法", duration, request.sample_period, nominal_speed,
        request.maximum_speed));
  }

  const double ux = request.initial_direction_x / direction_norm;
  const double uy = request.initial_direction_y / direction_norm;
  const double vx = -uy;
  const double vy = ux;
  const double segment_duration = duration / static_cast<double>(segment_count);
  TechniquePathResult result;
  result.path.type = TechniquePathType::kKnead;
  result.path.reference_frame = request.center_pose.header.frame_id;
  result.path.reference_stamp = request.center_pose.header.stamp;
  result.path.duration = duration;
  result.path.nominal_speed = nominal_speed;
  result.path.points.reserve(segment_count + 1U);
  for (std::size_t index = 0; index <= segment_count; ++index)
  {
    const double time = segment_duration * static_cast<double>(index);
    const double angle = 2.0 * kPi * time / request.cycle_duration;
    TechniquePathPoint point;
    point.pose = request.center_pose.pose;
    point.pose.position.x += request.radius *
      (ux * std::cos(angle) + vx * std::sin(angle));
    point.pose.position.y += request.radius *
      (uy * std::cos(angle) + vy * std::sin(angle));
    point.time_from_start = time;
    result.path.points.push_back(point);
  }
  result.success = true;
  result.message = "参考系 XY 平面闭合圆揉法路径生成成功";
  return result;
}

std::string to_string(TechniquePathType type)
{
  switch (type)
  {
    case TechniquePathType::kPush:
      return "push";
    case TechniquePathType::kPress:
      return "press";
    case TechniquePathType::kKnead:
      return "knead";
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
