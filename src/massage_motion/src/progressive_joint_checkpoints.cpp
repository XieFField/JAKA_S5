#include "massage_motion/progressive_joint_checkpoints.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_set>

namespace massage_motion
{

namespace
{

struct SampledJointTarget
{
  std::vector<double> positions;
  std::size_t source_segment_index{0U};
  double source_time{0.0};
};

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

double maximum_position_difference(
  const std::vector<double> & first, const std::vector<double> & second)
{
  if (first.size() != second.size() || first.empty())
  {
    return std::numeric_limits<double>::infinity();
  }
  double maximum = 0.0;
  for (std::size_t index = 0; index < first.size(); ++index)
  {
    maximum = std::max(maximum, std::abs(second[index] - first[index]));
  }
  return maximum;
}

}  // namespace

ProgressiveJointCheckpointResult generate_progressive_joint_checkpoints(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const ProgressiveJointCheckpointConfig & config)
{
  ProgressiveJointCheckpointResult result;
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.size() < 2U)
  {
    result.message = "轨迹至少需要关节名称和两个轨迹点";
    return result;
  }
  if (!std::isfinite(config.maximum_joint_step) ||
    config.maximum_joint_step <= 0.0 || config.maximum_checkpoint_count == 0U)
  {
    result.message = "最大关节步长或检查点数量上限无效";
    return result;
  }
  if (config.required_fractions.empty())
  {
    result.message = "必需检查点比例为空";
    return result;
  }
  double previous_fraction = 0.0;
  for (const double fraction : config.required_fractions)
  {
    if (!std::isfinite(fraction) || fraction <= previous_fraction ||
      fraction > 1.0)
    {
      result.message = "必需检查点比例必须在 (0, 1] 内严格递增";
      return result;
    }
    previous_fraction = fraction;
  }
  if (std::abs(config.required_fractions.back() - 1.0) > 1.0e-12)
  {
    result.message = "最后一个必需检查点比例必须为 1.0";
    return result;
  }

  const std::size_t joint_count = joint_trajectory.joint_names.size();
  std::unordered_set<std::string> unique_joint_names;
  unique_joint_names.reserve(joint_count);
  for (const auto & joint_name : joint_trajectory.joint_names)
  {
    if (joint_name.empty() || !unique_joint_names.emplace(joint_name).second)
    {
      result.message = "轨迹包含空关节名或重复关节名";
      return result;
    }
  }

  std::vector<double> cumulative_path(joint_trajectory.points.size(), 0.0);
  std::vector<double> point_times(joint_trajectory.points.size(), 0.0);
  double previous_time = -1.0;
  for (std::size_t point_index = 0;
    point_index < joint_trajectory.points.size(); ++point_index)
  {
    const auto & point = joint_trajectory.points[point_index];
    if (point.positions.size() != joint_count ||
      !std::all_of(
        point.positions.begin(), point.positions.end(),
        [](double value) {return std::isfinite(value);}))
    {
      result.message = "轨迹点位置数量不一致或包含非有限数值";
      return result;
    }
    const double point_time = duration_seconds(point.time_from_start);
    if (!std::isfinite(point_time) || point_time < 0.0 ||
      point_time < previous_time)
    {
      result.message = "轨迹时间无效或不是单调非递减";
      return result;
    }
    point_times[point_index] = point_time;
    previous_time = point_time;
    if (point_index == 0U)
    {
      continue;
    }
    const auto & previous = joint_trajectory.points[point_index - 1U];
    double squared_distance = 0.0;
    for (std::size_t joint = 0; joint < joint_count; ++joint)
    {
      const double delta = point.positions[joint] - previous.positions[joint];
      squared_distance += delta * delta;
    }
    const double segment_length = std::sqrt(squared_distance);
    if (!std::isfinite(segment_length))
    {
      result.message = "轨迹关节路径长度不是有限数值";
      return result;
    }
    cumulative_path[point_index] =
      cumulative_path[point_index - 1U] + segment_length;
  }
  const double path_length = cumulative_path.back();
  if (!std::isfinite(path_length) || path_length <= 1.0e-12)
  {
    result.message = "轨迹关节路径长度为零或无效";
    return result;
  }

  const auto sample = [&](double fraction) -> SampledJointTarget
    {
      const double target_distance = fraction * path_length;
      auto upper = std::lower_bound(
        cumulative_path.begin(), cumulative_path.end(), target_distance);
      std::size_t upper_index = static_cast<std::size_t>(
        std::distance(cumulative_path.begin(), upper));
      if (upper_index == 0U)
      {
        upper_index = 1U;
      }
      if (upper_index >= joint_trajectory.points.size())
      {
        upper_index = joint_trajectory.points.size() - 1U;
      }
      const std::size_t lower_index = upper_index - 1U;
      const double lower_distance = cumulative_path[lower_index];
      const double upper_distance = cumulative_path[upper_index];
      const double interval = upper_distance - lower_distance;
      const double interpolation = interval > 1.0e-12 ?
        std::clamp(
          (target_distance - lower_distance) / interval, 0.0, 1.0) : 1.0;
      SampledJointTarget target;
      target.positions.resize(joint_count);
      for (std::size_t joint = 0; joint < joint_count; ++joint)
      {
        const double lower_position =
          joint_trajectory.points[lower_index].positions[joint];
        const double upper_position =
          joint_trajectory.points[upper_index].positions[joint];
        target.positions[joint] = lower_position +
          interpolation * (upper_position - lower_position);
      }
      target.source_segment_index = lower_index;
      target.source_time = point_times[lower_index] +
        interpolation * (point_times[upper_index] - point_times[lower_index]);
      return target;
    };

  result.joint_names = joint_trajectory.joint_names;
  result.start_positions = joint_trajectory.points.front().positions;
  result.joint_path_length = path_length;
  std::vector<double> previous_positions = result.start_positions;
  double interval_start_fraction = 0.0;
  bool generation_failed = false;

  std::function<void(double, const std::vector<double> &, double,
    const SampledJointTarget &, bool)> append_interval;
  append_interval = [&](double left_fraction,
      const std::vector<double> & left_positions, double right_fraction,
      const SampledJointTarget & right_target, bool right_required)
    {
      if (generation_failed)
      {
        return;
      }
      const double maximum_step = maximum_position_difference(
        left_positions, right_target.positions);
      if (maximum_step <= config.maximum_joint_step)
      {
        if (result.checkpoints.size() >= config.maximum_checkpoint_count)
        {
          result.message = "自适应检查点数量超过配置上限";
          generation_failed = true;
          return;
        }
        ProgressiveJointCheckpoint checkpoint;
        checkpoint.fraction = right_fraction;
        checkpoint.required = right_required;
        checkpoint.positions = right_target.positions;
        checkpoint.source_segment_index = right_target.source_segment_index;
        checkpoint.source_time = right_target.source_time;
        checkpoint.maximum_joint_step_from_previous = maximum_step;
        result.maximum_generated_joint_step = std::max(
          result.maximum_generated_joint_step, maximum_step);
        result.checkpoints.push_back(std::move(checkpoint));
        return;
      }

      const double middle_fraction = 0.5 * (left_fraction + right_fraction);
      if (!std::isfinite(middle_fraction) ||
        middle_fraction <= left_fraction + 1.0e-12 ||
        middle_fraction >= right_fraction - 1.0e-12)
      {
        result.message = "无法在最大关节步长内继续细分检查点";
        generation_failed = true;
        return;
      }
      const auto middle_target = sample(middle_fraction);
      append_interval(
        left_fraction, left_positions, middle_fraction, middle_target, false);
      append_interval(
        middle_fraction, middle_target.positions, right_fraction,
        right_target, right_required);
    };

  for (const double required_fraction : config.required_fractions)
  {
    const auto required_target = sample(required_fraction);
    append_interval(
      interval_start_fraction, previous_positions, required_fraction,
      required_target, true);
    if (generation_failed)
    {
      result.checkpoints.clear();
      return result;
    }
    interval_start_fraction = required_fraction;
    previous_positions = required_target.positions;
  }

  result.valid = true;
  result.message = "渐进关节检查点生成成功";
  return result;
}

}  // namespace massage_motion
