#include "massage_motion/native_ptp_contract.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace massage_motion
{
namespace
{

double seconds(const builtin_interfaces::msg::Duration & value)
{
  return static_cast<double>(value.sec) +
         static_cast<double>(value.nanosec) * 1e-9;
}

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

NativePtpCommand invalid(const std::string & message)
{
  NativePtpCommand result;
  result.message = message;
  return result;
}

std::vector<double> reordered(
  const std::vector<std::string> & source_names,
  const std::vector<double> & source_values,
  const std::vector<std::string> & expected_names)
{
  if (source_names.size() != source_values.size()) return {};
  std::unordered_map<std::string, double> values;
  for (std::size_t index = 0; index < source_names.size(); ++index)
  {
    values[source_names[index]] = source_values[index];
  }
  std::vector<double> result;
  result.reserve(expected_names.size());
  for (const auto & name : expected_names)
  {
    const auto found = values.find(name);
    if (found == values.end()) return {};
    result.push_back(found->second);
  }
  return result;
}

}  // namespace

NativePtpCommand make_native_ptp_command(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const sensor_msgs::msg::JointState & current_state,
  double requested_timeout,
  const NativePtpConfig & config,
  double rapid_rate)
{
  if (config.joint_names.size() != 6U ||
    !finite_positive(config.maximum_start_error) ||
    !finite_positive(config.maximum_path_deviation) ||
    !finite_positive(config.maximum_speed) ||
    !finite_positive(config.maximum_acceleration) ||
    !finite_positive(config.endpoint_tolerance) ||
    !finite_positive(config.duration_safety_factor) ||
    !finite_positive(config.timeout_margin) ||
    !finite_positive(requested_timeout) || !finite_positive(rapid_rate) ||
    rapid_rate > 1.0)
  {
    return invalid("原生 PTP 配置或请求超时无效");
  }
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.points.empty())
  {
    return invalid("原生 PTP 规划轨迹为空");
  }
  if (joint_trajectory.joint_names != config.joint_names)
  {
    return invalid("规划轨迹关节名称或顺序与 JAKA S5 不一致");
  }
  const auto current = reordered(
    current_state.name, current_state.position, config.joint_names);
  if (current.size() != config.joint_names.size())
  {
    return invalid("当前关节状态不完整");
  }
  const auto & start = joint_trajectory.points.front().positions;
  const auto & target = joint_trajectory.points.back().positions;
  if (start.size() != config.joint_names.size() ||
    target.size() != config.joint_names.size())
  {
    return invalid("规划轨迹起点或终点关节数无效");
  }

  NativePtpCommand result;
  result.target_positions = target;
  std::vector<double> delta(target.size(), 0.0);
  double squared_length = 0.0;
  double maximum_delta = 0.0;
  double maximum_target_error = 0.0;
  std::size_t maximum_start_error_joint = 0U;
  std::size_t maximum_target_error_joint = 0U;
  for (std::size_t joint = 0; joint < target.size(); ++joint)
  {
    if (!std::isfinite(current[joint]) || !std::isfinite(start[joint]) ||
      !std::isfinite(target[joint]))
    {
      return invalid("原生 PTP 轨迹包含非有限关节值");
    }
    const double start_error = std::abs(current[joint] - start[joint]);
    if (start_error > result.maximum_start_error)
    {
      result.maximum_start_error = start_error;
      maximum_start_error_joint = joint;
    }
    const double target_error = std::abs(current[joint] - target[joint]);
    if (target_error > maximum_target_error)
    {
      maximum_target_error = target_error;
      maximum_target_error_joint = joint;
    }
    delta[joint] = target[joint] - start[joint];
    maximum_delta = std::max(maximum_delta, std::abs(delta[joint]));
    squared_length += delta[joint] * delta[joint];
  }
  // A no-op is safe even when MoveIt supplied a stale or single-point start:
  // no native command will be sent. Compare the live state to the target.
  if (maximum_target_error <= config.endpoint_tolerance)
  {
    result.valid = true;
    result.already_at_target = true;
    result.effective_timeout = requested_timeout;
    result.message = "原生 PTP 当前状态已在终点容差内；不发送运动命令";
    return result;
  }
  if (joint_trajectory.points.size() == 1U)
  {
    std::ostringstream message;
    message << std::fixed << std::setprecision(9)
            << "原生 PTP 只有一个轨迹点且当前状态尚未到达目标: joint="
            << config.joint_names[maximum_target_error_joint]
            << ", target_error=" << maximum_target_error
            << " rad, tolerance=" << config.endpoint_tolerance << " rad";
    result.message = message.str();
    return result;
  }
  if (result.maximum_start_error > config.maximum_start_error)
  {
    std::ostringstream message;
    message << std::fixed << std::setprecision(9)
            << "原生 PTP 起点与当前关节状态不一致: joint="
            << config.joint_names[maximum_start_error_joint]
            << ", start_error=" << result.maximum_start_error
            << " rad, limit=" << config.maximum_start_error << " rad";
    result.message = message.str();
    return result;
  }
  if (squared_length <= 1e-16) return invalid("原生 PTP 起终点没有有效位移");

  double previous_progress = -1e-9;
  std::vector<std::vector<double>> velocities;
  velocities.reserve(joint_trajectory.points.size() - 1U);
  for (std::size_t index = 0; index < joint_trajectory.points.size(); ++index)
  {
    const auto & point = joint_trajectory.points[index];
    if (point.positions.size() != target.size())
    {
      return invalid("原生 PTP 中间轨迹点关节数无效");
    }
    if (!point.velocities.empty() && point.velocities.size() != target.size())
    {
      return invalid("原生 PTP 轨迹点速度数量无效");
    }
    if (!point.accelerations.empty() &&
      point.accelerations.size() != target.size())
    {
      return invalid("原生 PTP 轨迹点加速度数量无效");
    }
    for (double value : point.velocities)
    {
      if (!std::isfinite(value)) return invalid("原生 PTP 速度包含非有限值");
      result.speed = std::max(result.speed, std::abs(value));
    }
    for (double value : point.accelerations)
    {
      if (!std::isfinite(value)) return invalid("原生 PTP 加速度包含非有限值");
      result.acceleration = std::max(result.acceleration, std::abs(value));
    }
    double dot = 0.0;
    for (std::size_t joint = 0; joint < target.size(); ++joint)
    {
      dot += (point.positions[joint] - start[joint]) * delta[joint];
    }
    const double progress = dot / squared_length;
    if (progress < -1e-6 || progress > 1.0 + 1e-6 ||
      progress + 1e-6 < previous_progress)
    {
      return invalid("规划轨迹存在回摆或越过 PTP 起终点");
    }
    previous_progress = progress;
    for (std::size_t joint = 0; joint < target.size(); ++joint)
    {
      const double expected = start[joint] + progress * delta[joint];
      result.maximum_path_deviation = std::max(
        result.maximum_path_deviation,
        std::abs(point.positions[joint] - expected));
    }
    if (index > 0U)
    {
      const double dt = seconds(point.time_from_start) -
        seconds(joint_trajectory.points[index - 1U].time_from_start);
      if (!finite_positive(dt)) return invalid("规划轨迹时间戳不递增");
      std::vector<double> segment_velocity(target.size(), 0.0);
      for (std::size_t joint = 0; joint < target.size(); ++joint)
      {
        segment_velocity[joint] =
          (point.positions[joint] -
          joint_trajectory.points[index - 1U].positions[joint]) / dt;
        result.speed = std::max(result.speed, std::abs(segment_velocity[joint]));
      }
      velocities.push_back(std::move(segment_velocity));
    }
  }
  if (result.maximum_path_deviation > config.maximum_path_deviation)
  {
    result.message = "规划轨迹不是与 JAKA joint_move 等价的直接关节路径";
    return result;
  }

  for (std::size_t index = 1U; index < velocities.size(); ++index)
  {
    const double t0 = seconds(joint_trajectory.points[index].time_from_start);
    const double t1 = seconds(joint_trajectory.points[index + 1U].time_from_start);
    const double dt = t1 - t0;
    for (std::size_t joint = 0; joint < target.size(); ++joint)
    {
      result.acceleration = std::max(
        result.acceleration,
        std::abs(velocities[index][joint] - velocities[index - 1U][joint]) / dt);
    }
  }
  if (!finite_positive(result.speed))
  {
    return invalid("无法从规划轨迹得到有效 PTP 速度");
  }
  if (!finite_positive(result.acceleration))
  {
    return invalid("无法从规划轨迹得到有效 PTP 加速度；拒绝采用隐式默认值");
  }
  const double desired_effective_speed =
    std::min(result.speed, config.maximum_speed);
  const double desired_effective_acceleration =
    std::min(result.acceleration, config.maximum_acceleration);
  result.rapid_rate = rapid_rate;
  // MoveIt scaling already describes the desired physical speed. Compensate
  // the controller-wide override once here so rapid_rate does not scale it a
  // second time. These caps bound the actual SDK arguments.
  result.speed = std::min(desired_effective_speed / rapid_rate, 1.0);
  result.acceleration = std::min(
    desired_effective_acceleration / rapid_rate, 5.0);
  result.effective_speed = result.speed * rapid_rate;

  const double effective_acceleration = result.acceleration * rapid_rate;
  const double ramp_distance =
    result.effective_speed * result.effective_speed / effective_acceleration;
  result.estimated_duration = maximum_delta <= ramp_distance ?
    2.0 * std::sqrt(maximum_delta / effective_acceleration) :
    2.0 * result.effective_speed / effective_acceleration +
    (maximum_delta - ramp_distance) / result.effective_speed;
  result.effective_timeout = std::max(
    requested_timeout,
    result.estimated_duration * config.duration_safety_factor +
    config.timeout_margin);
  result.valid = true;
  result.message = "原生 PTP 路径等价门禁通过";
  return result;
}

}  // namespace massage_motion
