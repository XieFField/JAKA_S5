#include "massage_motion/motion_repeatability.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace massage_motion
{

namespace
{

bool finite_trial(const MotionRepeatabilityTrial & trial)
{
  return trial.cycle > 0U &&
         std::isfinite(trial.initial_position) &&
         std::isfinite(trial.target_position) &&
         std::isfinite(trial.actual_position) &&
         std::isfinite(trial.commanded_delta) &&
         trial.commanded_delta != 0.0 &&
         std::isfinite(trial.achieved_delta) &&
         std::isfinite(trial.completion_ratio) &&
         std::isfinite(trial.endpoint_error) && trial.endpoint_error >= 0.0 &&
         std::isfinite(trial.maximum_endpoint_error) &&
         trial.maximum_endpoint_error >= 0.0 &&
         std::isfinite(trial.planned_duration) &&
         trial.planned_duration >= 0.0 &&
         std::isfinite(trial.execution_duration) &&
         trial.execution_duration >= 0.0;
}

bool direction_matches_command(const MotionRepeatabilityTrial & trial)
{
  return (trial.direction == RepeatabilityDirection::kPositive &&
          trial.commanded_delta > 0.0) ||
         (trial.direction == RepeatabilityDirection::kNegative &&
          trial.commanded_delta < 0.0);
}

DirectionRepeatabilityStatistics calculate_direction_statistics(
  const std::vector<MotionRepeatabilityTrial> & trials,
  RepeatabilityDirection direction)
{
  DirectionRepeatabilityStatistics statistics;
  double actual_sum = 0.0;
  double error_sum = 0.0;
  double completion_sum = 0.0;
  statistics.minimum_actual_position =
    std::numeric_limits<double>::infinity();
  statistics.maximum_actual_position =
    -std::numeric_limits<double>::infinity();
  statistics.minimum_completion_ratio =
    std::numeric_limits<double>::infinity();

  for (const auto & trial : trials)
  {
    if (trial.direction != direction)
    {
      continue;
    }
    ++statistics.samples;
    if (trial.passed)
    {
      ++statistics.passed;
    }
    actual_sum += trial.actual_position;
    error_sum += trial.endpoint_error;
    completion_sum += trial.completion_ratio;
    statistics.minimum_actual_position = std::min(
      statistics.minimum_actual_position, trial.actual_position);
    statistics.maximum_actual_position = std::max(
      statistics.maximum_actual_position, trial.actual_position);
    statistics.maximum_endpoint_error = std::max(
      statistics.maximum_endpoint_error, trial.endpoint_error);
    statistics.minimum_completion_ratio = std::min(
      statistics.minimum_completion_ratio, trial.completion_ratio);
  }

  if (statistics.samples == 0U)
  {
    statistics.minimum_actual_position = 0.0;
    statistics.maximum_actual_position = 0.0;
    statistics.minimum_completion_ratio = 0.0;
    return statistics;
  }

  const double count = static_cast<double>(statistics.samples);
  statistics.mean_actual_position = actual_sum / count;
  statistics.position_range =
    statistics.maximum_actual_position - statistics.minimum_actual_position;
  statistics.mean_endpoint_error = error_sum / count;
  statistics.mean_completion_ratio = completion_sum / count;
  return statistics;
}

}  // namespace

MotionRepeatabilitySummary analyze_motion_repeatability(
  const std::vector<MotionRepeatabilityTrial> & trials)
{
  MotionRepeatabilitySummary summary;
  if (trials.empty())
  {
    summary.message = "重复性样本不能为空";
    return summary;
  }
  if (!std::all_of(trials.begin(), trials.end(), finite_trial))
  {
    summary.message = "重复性样本包含无效数值";
    return summary;
  }
  if (!std::all_of(trials.begin(), trials.end(), direction_matches_command))
  {
    summary.message = "重复性样本方向与命令位移不一致";
    return summary;
  }

  summary.samples = trials.size();
  summary.passed = static_cast<std::size_t>(std::count_if(
    trials.begin(), trials.end(),
    [](const auto & trial) {return trial.passed;}));
  summary.positive = calculate_direction_statistics(
    trials, RepeatabilityDirection::kPositive);
  summary.negative = calculate_direction_statistics(
    trials, RepeatabilityDirection::kNegative);
  if (summary.positive.samples == 0U || summary.negative.samples == 0U)
  {
    summary.message = "重复性样本必须同时包含正向和反向运动";
    return summary;
  }

  summary.valid = true;
  summary.all_passed = summary.passed == summary.samples;
  summary.message = summary.all_passed ?
    "全部重复性样本通过" : "存在未通过的重复性样本";
  return summary;
}

const char * to_string(RepeatabilityDirection direction)
{
  return direction == RepeatabilityDirection::kPositive ?
    "positive" : "negative";
}

}  // namespace massage_motion
