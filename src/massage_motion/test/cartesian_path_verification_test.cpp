#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "massage_motion/cartesian_path_verification.hpp"

namespace massage_motion
{

namespace
{

geometry_msgs::msg::Pose pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.position.z = z;
  result.orientation.w = 1.0;
  return result;
}

CartesianLineTraceConfig strict_config()
{
  CartesianLineTraceConfig config;
  config.maximum_endpoint_position_error = 0.002;
  config.maximum_endpoint_orientation_error = 0.01;
  config.maximum_transverse_error = 0.002;
  config.maximum_height_error = 0.002;
  config.maximum_orientation_error = 0.01;
  config.maximum_longitudinal_overshoot = 0.002;
  return config;
}

}  // namespace

TEST(CartesianPathVerificationTest, AcceptsWorldPositiveYLine)
{
  const auto metrics = evaluate_cartesian_line_trace(
    {pose(-0.47, 0.15, 0.23), pose(-0.47, 0.18, 0.23),
      pose(-0.47, 0.20, 0.23)},
    pose(-0.47, 0.15, 0.23), pose(-0.47, 0.20, 0.23), strict_config());

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_TRUE(metrics.accepted) << metrics.message;
  EXPECT_NEAR(metrics.expected_length, 0.05, 1.0e-12);
  EXPECT_NEAR(metrics.final_directed_progress, 0.05, 1.0e-12);
  EXPECT_DOUBLE_EQ(metrics.maximum_transverse_error, 0.0);
  EXPECT_DOUBLE_EQ(metrics.maximum_height_error, 0.0);
}

TEST(CartesianPathVerificationTest, RejectsTransverseAndHeightDeviation)
{
  const auto metrics = evaluate_cartesian_line_trace(
    {pose(0.0, 0.0, 0.2), pose(0.01, 0.02, 0.21),
      pose(0.0, 0.05, 0.2)},
    pose(0.0, 0.0, 0.2), pose(0.0, 0.05, 0.2), strict_config());

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_FALSE(metrics.accepted);
  EXPECT_NEAR(metrics.maximum_transverse_error, std::sqrt(0.0002), 1.0e-12);
  EXPECT_NEAR(metrics.maximum_height_error, 0.01, 1.0e-12);
}

TEST(CartesianPathVerificationTest, RejectsWrongDirectionAndEndpoint)
{
  const auto metrics = evaluate_cartesian_line_trace(
    {pose(0.0, 0.0, 0.2), pose(0.0, -0.01, 0.2)},
    pose(0.0, 0.0, 0.2), pose(0.0, 0.05, 0.2), strict_config());

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_FALSE(metrics.accepted);
  EXPECT_LT(metrics.final_directed_progress, 0.0);
}

TEST(CartesianPathVerificationTest, RejectsOrientationDrift)
{
  auto drifted = pose(0.0, 0.05, 0.2);
  drifted.orientation.z = std::sin(0.05);
  drifted.orientation.w = std::cos(0.05);
  const auto metrics = evaluate_cartesian_line_trace(
    {pose(0.0, 0.0, 0.2), drifted},
    pose(0.0, 0.0, 0.2), pose(0.0, 0.05, 0.2), strict_config());

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_FALSE(metrics.accepted);
  EXPECT_NEAR(metrics.maximum_orientation_error, 0.1, 1.0e-12);
}

TEST(CartesianPathVerificationTest, RejectsMalformedInputs)
{
  EXPECT_FALSE(evaluate_cartesian_line_trace(
      {}, pose(0.0, 0.0, 0.0), pose(0.0, 0.1, 0.0)).valid);
  EXPECT_FALSE(evaluate_cartesian_line_trace(
      {pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0)},
      pose(0.0, 0.0, 0.0), pose(0.0, 0.0, 0.0)).valid);

  auto invalid = pose(0.0, 0.0, 0.0);
  invalid.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(evaluate_cartesian_line_trace(
      {pose(0.0, 0.0, 0.0), invalid},
      pose(0.0, 0.0, 0.0), pose(0.0, 0.1, 0.0)).valid);
}

TEST(CartesianPathVerificationTest, CalculatesPoseError)
{
  auto actual = pose(0.003, 0.004, 0.0);
  actual.orientation.z = std::sin(0.1);
  actual.orientation.w = std::cos(0.1);
  const auto metrics = calculate_pose_error(pose(0.0, 0.0, 0.0), actual);

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_NEAR(metrics.translation, 0.005, 1.0e-12);
  EXPECT_NEAR(metrics.rotation, 0.2, 1.0e-12);
}

}  // namespace massage_motion
