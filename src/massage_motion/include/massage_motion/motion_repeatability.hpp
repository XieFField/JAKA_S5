#ifndef MASSAGE_MOTION__MOTION_REPEATABILITY_HPP_
#define MASSAGE_MOTION__MOTION_REPEATABILITY_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace massage_motion
{

enum class RepeatabilityDirection
{
  kPositive,
  kNegative,
};

struct MotionRepeatabilityTrial
{
  std::size_t cycle{0U};
  RepeatabilityDirection direction{RepeatabilityDirection::kPositive};
  bool passed{false};
  double initial_position{0.0};
  double target_position{0.0};
  double actual_position{0.0};
  double commanded_delta{0.0};
  double achieved_delta{0.0};
  double completion_ratio{0.0};
  double endpoint_error{0.0};
  double maximum_endpoint_error{0.0};
  double planned_duration{0.0};
  double execution_duration{0.0};
  std::string message;
};

struct DirectionRepeatabilityStatistics
{
  std::size_t samples{0U};
  std::size_t passed{0U};
  double mean_actual_position{0.0};
  double minimum_actual_position{0.0};
  double maximum_actual_position{0.0};
  double position_range{0.0};
  double mean_endpoint_error{0.0};
  double maximum_endpoint_error{0.0};
  double mean_completion_ratio{0.0};
  double minimum_completion_ratio{0.0};
};

struct MotionRepeatabilitySummary
{
  bool valid{false};
  bool all_passed{false};
  std::string message;
  std::size_t samples{0U};
  std::size_t passed{0U};
  DirectionRepeatabilityStatistics positive;
  DirectionRepeatabilityStatistics negative;
};

MotionRepeatabilitySummary analyze_motion_repeatability(
  const std::vector<MotionRepeatabilityTrial> & trials);

const char * to_string(RepeatabilityDirection direction);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__MOTION_REPEATABILITY_HPP_
