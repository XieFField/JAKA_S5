#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "massage_motion/joint_target_interpolation.hpp"

TEST(JointTargetInterpolationTest, ComputesQuarterHalfAndFullTargets)
{
  const std::vector<double> start{0.0, 1.0, -2.0};
  const std::vector<double> goal{1.0, -1.0, 2.0};

  const auto quarter = massage_motion::interpolate_joint_target(
    start, goal, 0.25, 2.0);
  ASSERT_TRUE(quarter.valid) << quarter.message;
  ASSERT_EQ(quarter.target_positions.size(), 3U);
  EXPECT_NEAR(quarter.target_positions[0], 0.25, 1e-12);
  EXPECT_NEAR(quarter.target_positions[1], 0.5, 1e-12);
  EXPECT_NEAR(quarter.target_positions[2], -1.0, 1e-12);
  EXPECT_NEAR(quarter.maximum_joint_travel, 1.0, 1e-12);

  const auto half = massage_motion::interpolate_joint_target(
    start, goal, 0.5, 2.0);
  ASSERT_TRUE(half.valid) << half.message;
  EXPECT_EQ(half.target_positions, (std::vector<double>{0.5, 0.0, 0.0}));
  EXPECT_EQ(half.joint_travels, (std::vector<double>{0.5, -1.0, 2.0}));

  const auto full = massage_motion::interpolate_joint_target(
    start, goal, 1.0, 4.0);
  ASSERT_TRUE(full.valid) << full.message;
  EXPECT_EQ(full.target_positions, goal);
  EXPECT_NEAR(full.maximum_joint_travel, 4.0, 1e-12);
}

TEST(JointTargetInterpolationTest, EnforcesMaximumTravel)
{
  const auto accepted = massage_motion::interpolate_joint_target(
    {0.0, 0.0}, {0.4, -0.2}, 0.25, 0.1);
  ASSERT_TRUE(accepted.valid) << accepted.message;
  EXPECT_NEAR(accepted.maximum_joint_travel, 0.1, 1e-12);

  const auto rejected = massage_motion::interpolate_joint_target(
    {0.0, 0.0}, {0.4, -0.2}, 0.5, 0.1);
  EXPECT_FALSE(rejected.valid);
  EXPECT_TRUE(rejected.target_positions.empty());
  EXPECT_TRUE(rejected.joint_travels.empty());
}

TEST(JointTargetInterpolationTest, RejectsInvalidShapeRatioAndLimit)
{
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {}, {}, 0.25, 1.0).valid);
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {0.0}, {0.0, 1.0}, 0.25, 1.0).valid);
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {0.0}, {1.0}, 0.0, 1.0).valid);
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {0.0}, {1.0}, 1.01, 1.0).valid);
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {0.0}, {1.0}, std::numeric_limits<double>::quiet_NaN(), 1.0).valid);
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {0.0}, {1.0}, 0.25, 0.0).valid);
}

TEST(JointTargetInterpolationTest, RejectsNonFiniteInputsAndOverflow)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const double maximum = std::numeric_limits<double>::max();
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {infinity}, {0.0}, 0.25, 1.0).valid);
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {0.0}, {infinity}, 0.25, 1.0).valid);
  EXPECT_FALSE(massage_motion::interpolate_joint_target(
    {-maximum}, {maximum}, 0.5, maximum).valid);
}
