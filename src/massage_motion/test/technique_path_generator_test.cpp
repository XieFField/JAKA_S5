#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "massage_motion/technique_path_generator.hpp"

namespace
{

massage_motion::PushPathRequest valid_request()
{
  massage_motion::PushPathRequest request;
  request.start_pose.header.frame_id = "world";
  request.start_pose.header.stamp.sec = 123;
  request.start_pose.header.stamp.nanosec = 456U;
  request.start_pose.pose.position.x = 0.4;
  request.start_pose.pose.position.y = -0.2;
  request.start_pose.pose.position.z = 0.3;
  request.start_pose.pose.orientation.x = 0.0;
  request.start_pose.pose.orientation.y = 0.0;
  request.start_pose.pose.orientation.z = std::sqrt(0.5);
  request.start_pose.pose.orientation.w = std::sqrt(0.5);
  request.direction_x = 3.0;
  request.direction_y = 4.0;
  request.length = 0.1;
  request.speed = 0.02;
  request.sample_period = 0.2;
  request.maximum_speed = 0.03;
  return request;
}

TEST(TechniquePathGeneratorTest, GeneratesBoundedWorldXyPush)
{
  const auto request = valid_request();
  const auto result =
    massage_motion::TechniquePathGenerator::generate_push(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.error, massage_motion::TechniquePathError::kNone);
  EXPECT_EQ(result.path.type, massage_motion::TechniquePathType::kPush);
  EXPECT_EQ(result.path.reference_frame, "world");
  EXPECT_EQ(result.path.reference_stamp, request.start_pose.header.stamp);
  EXPECT_DOUBLE_EQ(result.path.duration, 5.0);
  EXPECT_DOUBLE_EQ(result.path.nominal_speed, request.speed);
  ASSERT_EQ(result.path.points.size(), 26U);

  const auto & first = result.path.points.front();
  const auto & last = result.path.points.back();
  EXPECT_DOUBLE_EQ(first.time_from_start, 0.0);
  EXPECT_NEAR(last.time_from_start, 5.0, 1.0e-12);
  EXPECT_NEAR(last.pose.position.x, 0.46, 1.0e-12);
  EXPECT_NEAR(last.pose.position.y, -0.12, 1.0e-12);
  EXPECT_DOUBLE_EQ(last.pose.position.z, request.start_pose.pose.position.z);

  for (std::size_t index = 0; index < result.path.points.size(); ++index)
  {
    const auto & point = result.path.points[index];
    EXPECT_DOUBLE_EQ(point.pose.position.z, request.start_pose.pose.position.z);
    EXPECT_EQ(point.pose.orientation, request.start_pose.pose.orientation);
    if (index == 0U)
    {
      continue;
    }

    const auto & previous = result.path.points[index - 1U];
    const double dx = point.pose.position.x - previous.pose.position.x;
    const double dy = point.pose.position.y - previous.pose.position.y;
    const double dt = point.time_from_start - previous.time_from_start;
    ASSERT_GT(dt, 0.0);
    EXPECT_LE(dt, request.sample_period + 1.0e-12);
    EXPECT_NEAR(std::hypot(dx, dy) / dt, request.speed, 1.0e-12);
    EXPECT_LE(std::hypot(dx, dy) / dt, request.maximum_speed);
  }
}

TEST(TechniquePathGeneratorTest, IncludesExactEndpointForNonIntegralPeriod)
{
  auto request = valid_request();
  request.length = 0.095;
  request.sample_period = 0.3;

  const auto result =
    massage_motion::TechniquePathGenerator::generate_push(request);

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.path.points.size(), 17U);
  const auto & last = result.path.points.back();
  EXPECT_NEAR(last.time_from_start, 4.75, 1.0e-12);
  EXPECT_NEAR(last.pose.position.x, 0.457, 1.0e-12);
  EXPECT_NEAR(last.pose.position.y, -0.124, 1.0e-12);
}

TEST(TechniquePathGeneratorTest, SupportsNegativeDirections)
{
  auto request = valid_request();
  request.direction_x = -1.0;
  request.direction_y = 0.0;

  const auto result =
    massage_motion::TechniquePathGenerator::generate_push(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_NEAR(result.path.points.back().pose.position.x, 0.3, 1.0e-12);
  EXPECT_DOUBLE_EQ(
    result.path.points.back().pose.position.y,
    request.start_pose.pose.position.y);
}

TEST(TechniquePathGeneratorTest, RejectsInvalidFramePoseAndDirection)
{
  auto request = valid_request();
  request.start_pose.header.frame_id.clear();
  EXPECT_FALSE(massage_motion::TechniquePathGenerator::generate_push(request).success);

  request = valid_request();
  request.start_pose.pose.position.z =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(massage_motion::TechniquePathGenerator::generate_push(request).success);

  request = valid_request();
  request.start_pose.pose.orientation.w = 0.5;
  EXPECT_FALSE(massage_motion::TechniquePathGenerator::generate_push(request).success);

  request = valid_request();
  request.direction_x = 0.0;
  request.direction_y = 0.0;
  EXPECT_FALSE(massage_motion::TechniquePathGenerator::generate_push(request).success);
}

TEST(TechniquePathGeneratorTest, RejectsInvalidTimingAndSpeedLimit)
{
  auto request = valid_request();
  request.length = 0.0;
  EXPECT_FALSE(massage_motion::TechniquePathGenerator::generate_push(request).success);

  request = valid_request();
  request.speed = request.maximum_speed + 0.001;
  EXPECT_FALSE(massage_motion::TechniquePathGenerator::generate_push(request).success);

  request = valid_request();
  request.sample_period = -0.1;
  EXPECT_FALSE(massage_motion::TechniquePathGenerator::generate_push(request).success);
}

TEST(TechniquePathGeneratorTest, RejectsExcessivePointCount)
{
  auto request = valid_request();
  request.length = 100.0;
  request.speed = 0.001;
  request.maximum_speed = 0.001;
  request.sample_period = 0.001;

  const auto result =
    massage_motion::TechniquePathGenerator::generate_push(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, massage_motion::TechniquePathError::kPathTooLarge);
  EXPECT_TRUE(result.path.points.empty());
}

TEST(TechniquePathGeneratorTest, ProvidesStableDiagnosticNames)
{
  EXPECT_EQ(
    massage_motion::to_string(massage_motion::TechniquePathType::kPush),
    "push");
  EXPECT_EQ(
    massage_motion::to_string(massage_motion::TechniquePathError::kPathTooLarge),
    "path_too_large");
}

}  // namespace
