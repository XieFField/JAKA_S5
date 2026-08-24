#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "massage_motion/progressive_joint_checkpoints.hpp"

namespace massage_motion
{

namespace
{

moveit_msgs::msg::RobotTrajectory make_corner_trajectory()
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2"};
  const std::vector<std::vector<double>> positions = {
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  for (std::size_t index = 0; index < positions.size(); ++index)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions[index];
    point.time_from_start.sec = static_cast<std::int32_t>(index);
    trajectory.joint_trajectory.points.push_back(point);
  }
  return trajectory;
}

}  // namespace

TEST(ProgressiveJointCheckpointsTest, SamplesRequiredFractionsByJointPathLength)
{
  ProgressiveJointCheckpointConfig config;
  config.required_fractions = {0.25, 0.50, 0.75, 1.0};
  config.maximum_joint_step = 1.0;
  const auto result = generate_progressive_joint_checkpoints(
    make_corner_trajectory(), config);

  ASSERT_TRUE(result.valid) << result.message;
  ASSERT_EQ(result.checkpoints.size(), 4U);
  EXPECT_NEAR(result.joint_path_length, 2.0, 1.0e-12);
  EXPECT_EQ(result.joint_names, std::vector<std::string>({"joint_1", "joint_2"}));
  EXPECT_EQ(result.start_positions, std::vector<double>({0.0, 0.0}));
  EXPECT_EQ(result.checkpoints[0].positions, std::vector<double>({0.5, 0.0}));
  EXPECT_EQ(result.checkpoints[1].positions, std::vector<double>({1.0, 0.0}));
  EXPECT_EQ(result.checkpoints[2].positions, std::vector<double>({1.0, 0.5}));
  EXPECT_EQ(result.checkpoints[3].positions, std::vector<double>({1.0, 1.0}));
  EXPECT_TRUE(result.checkpoints[0].required);
  EXPECT_DOUBLE_EQ(result.checkpoints[2].source_time, 1.5);
}

TEST(ProgressiveJointCheckpointsTest, InsertsAdaptiveCheckpointsForStepLimit)
{
  ProgressiveJointCheckpointConfig config;
  config.required_fractions = {0.50, 1.0};
  config.maximum_joint_step = 0.26;
  const auto result = generate_progressive_joint_checkpoints(
    make_corner_trajectory(), config);

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_GT(result.checkpoints.size(), config.required_fractions.size());
  EXPECT_LE(result.maximum_generated_joint_step, 0.26 + 1.0e-12);
  EXPECT_TRUE(result.checkpoints.back().required);
  EXPECT_DOUBLE_EQ(result.checkpoints.back().fraction, 1.0);
  std::size_t required_count = 0U;
  for (const auto & checkpoint : result.checkpoints)
  {
    EXPECT_GT(checkpoint.fraction, 0.0);
    EXPECT_LE(checkpoint.maximum_joint_step_from_previous, 0.26 + 1.0e-12);
    required_count += checkpoint.required ? 1U : 0U;
  }
  EXPECT_EQ(required_count, 2U);
}

TEST(ProgressiveJointCheckpointsTest, RejectsInvalidFractionContracts)
{
  auto config = ProgressiveJointCheckpointConfig{};
  config.required_fractions = {0.50, 0.25, 1.0};
  EXPECT_FALSE(generate_progressive_joint_checkpoints(
    make_corner_trajectory(), config).valid);

  config.required_fractions = {0.25, 0.50};
  EXPECT_FALSE(generate_progressive_joint_checkpoints(
    make_corner_trajectory(), config).valid);

  config.required_fractions = {0.25, 1.0};
  config.maximum_joint_step = 0.0;
  EXPECT_FALSE(generate_progressive_joint_checkpoints(
    make_corner_trajectory(), config).valid);
}

TEST(ProgressiveJointCheckpointsTest, RejectsMalformedAndZeroLengthTrajectories)
{
  auto malformed = make_corner_trajectory();
  malformed.joint_trajectory.points[1].positions.pop_back();
  EXPECT_FALSE(generate_progressive_joint_checkpoints(malformed).valid);

  auto non_finite = make_corner_trajectory();
  non_finite.joint_trajectory.points[1].positions[0] =
    std::numeric_limits<double>::infinity();
  EXPECT_FALSE(generate_progressive_joint_checkpoints(non_finite).valid);

  auto decreasing_time = make_corner_trajectory();
  decreasing_time.joint_trajectory.points[2].time_from_start.sec = 0;
  EXPECT_FALSE(generate_progressive_joint_checkpoints(decreasing_time).valid);

  auto zero_length = make_corner_trajectory();
  for (auto & point : zero_length.joint_trajectory.points)
  {
    point.positions = {0.0, 0.0};
  }
  EXPECT_FALSE(generate_progressive_joint_checkpoints(zero_length).valid);
}

TEST(ProgressiveJointCheckpointsTest, FailsClosedWhenAdaptiveLimitIsTooSmall)
{
  ProgressiveJointCheckpointConfig config;
  config.required_fractions = {1.0};
  config.maximum_joint_step = 0.01;
  config.maximum_checkpoint_count = 2U;
  const auto result = generate_progressive_joint_checkpoints(
    make_corner_trajectory(), config);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.checkpoints.empty());
}

}  // namespace massage_motion
