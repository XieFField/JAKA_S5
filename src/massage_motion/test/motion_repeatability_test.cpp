#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "massage_motion/motion_repeatability.hpp"

namespace
{

massage_motion::MotionRepeatabilityTrial trial(
  std::size_t cycle,
  massage_motion::RepeatabilityDirection direction,
  double actual,
  double endpoint_error,
  double completion,
  bool passed = true)
{
  const bool positive =
    direction == massage_motion::RepeatabilityDirection::kPositive;
  return {
    cycle, direction, passed,
    positive ? 0.0 : 0.01,
    positive ? 0.01 : 0.0,
    actual,
    positive ? 0.01 : -0.01,
    positive ? actual : 0.01 - actual,
    completion,
    endpoint_error,
    endpoint_error,
    1.4,
    1.8,
    passed ? "passed" : "failed"};
}

TEST(MotionRepeatabilityTest, SummarizesBothDirectionsAndPositionRange)
{
  const std::vector<massage_motion::MotionRepeatabilityTrial> trials{
    trial(1U, massage_motion::RepeatabilityDirection::kPositive, 0.0090, 0.0010, 0.90),
    trial(1U, massage_motion::RepeatabilityDirection::kNegative, 0.0008, 0.0008, 0.92),
    trial(2U, massage_motion::RepeatabilityDirection::kPositive, 0.0094, 0.0006, 0.94),
    trial(2U, massage_motion::RepeatabilityDirection::kNegative, 0.0005, 0.0005, 0.95),
  };

  const auto summary =
    massage_motion::analyze_motion_repeatability(trials);

  ASSERT_TRUE(summary.valid) << summary.message;
  EXPECT_TRUE(summary.all_passed);
  EXPECT_EQ(summary.samples, 4U);
  EXPECT_EQ(summary.passed, 4U);
  EXPECT_EQ(summary.positive.samples, 2U);
  EXPECT_EQ(summary.negative.samples, 2U);
  EXPECT_NEAR(summary.positive.mean_actual_position, 0.0092, 1e-12);
  EXPECT_NEAR(summary.positive.position_range, 0.0004, 1e-12);
  EXPECT_NEAR(summary.negative.maximum_endpoint_error, 0.0008, 1e-12);
  EXPECT_NEAR(summary.negative.minimum_completion_ratio, 0.92, 1e-12);
}

TEST(MotionRepeatabilityTest, PreservesFailedTrialInSummary)
{
  const std::vector<massage_motion::MotionRepeatabilityTrial> trials{
    trial(1U, massage_motion::RepeatabilityDirection::kPositive, 0.008, 0.002, 0.8, false),
    trial(1U, massage_motion::RepeatabilityDirection::kNegative, 0.001, 0.001, 0.9),
  };

  const auto summary =
    massage_motion::analyze_motion_repeatability(trials);

  ASSERT_TRUE(summary.valid);
  EXPECT_FALSE(summary.all_passed);
  EXPECT_EQ(summary.passed, 1U);
  EXPECT_EQ(summary.positive.passed, 0U);
}

TEST(MotionRepeatabilityTest, RejectsMissingDirectionAndInvalidValues)
{
  auto positive = trial(
    1U, massage_motion::RepeatabilityDirection::kPositive,
    0.009, 0.001, 0.9);
  EXPECT_FALSE(massage_motion::analyze_motion_repeatability({positive}).valid);

  auto invalid = trial(
    1U, massage_motion::RepeatabilityDirection::kNegative,
    0.001, 0.001, 0.9);
  invalid.actual_position = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    massage_motion::analyze_motion_repeatability({positive, invalid}).valid);

  auto wrong_direction = positive;
  wrong_direction.direction =
    massage_motion::RepeatabilityDirection::kNegative;
  EXPECT_FALSE(massage_motion::analyze_motion_repeatability(
    {positive, wrong_direction}).valid);
}

}  // namespace
