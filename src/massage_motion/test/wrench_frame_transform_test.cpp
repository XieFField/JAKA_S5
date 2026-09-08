#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "massage_motion/wrench_frame_transform.hpp"

namespace
{

massage_motion::WrenchFrameTransformRequest identity_request()
{
  massage_motion::WrenchFrameTransformRequest request;
  request.target_from_source_rotation.w = 1.0;
  return request;
}

TEST(WrenchFrameTransformTest, RotatesForceAndTorque)
{
  auto request = identity_request();
  const double half = std::sqrt(0.5);
  request.target_from_source_rotation.z = half;
  request.target_from_source_rotation.w = half;
  request.source_wrench = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  const auto result = massage_motion::transform_wrench_to_reference(request);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_NEAR(result.wrench[0], -2.0, 1.0e-12);
  EXPECT_NEAR(result.wrench[1], 1.0, 1.0e-12);
  EXPECT_NEAR(result.wrench[2], 3.0, 1.0e-12);
  EXPECT_NEAR(result.wrench[3], -5.0, 1.0e-12);
  EXPECT_NEAR(result.wrench[4], 4.0, 1.0e-12);
  EXPECT_NEAR(result.wrench[5], 6.0, 1.0e-12);
}

TEST(WrenchFrameTransformTest, ShiftsTorqueToRequestedReferencePoint)
{
  auto request = identity_request();
  request.source_wrench = {0.0, 10.0, 0.0, 1.0, 2.0, 3.0};
  request.source_origin_in_target.x = 0.2;
  request.reference_origin_in_target.x = 0.1;
  const auto result = massage_motion::transform_wrench_to_reference(request);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_DOUBLE_EQ(result.wrench[0], 0.0);
  EXPECT_DOUBLE_EQ(result.wrench[1], 10.0);
  EXPECT_DOUBLE_EQ(result.wrench[3], 1.0);
  EXPECT_DOUBLE_EQ(result.wrench[4], 2.0);
  EXPECT_NEAR(result.wrench[5], 4.0, 1.0e-12);
}

TEST(WrenchFrameTransformTest, DoesNotInventMomentAtSamePoint)
{
  auto request = identity_request();
  request.source_wrench = {2.0, -3.0, 4.0, 0.1, 0.2, 0.3};
  request.source_origin_in_target.x = 1.0;
  request.reference_origin_in_target.x = 1.0;
  const auto result = massage_motion::transform_wrench_to_reference(request);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_EQ(result.wrench, request.source_wrench);
}

TEST(WrenchFrameTransformTest, RejectsInvalidInputs)
{
  auto request = identity_request();
  request.source_wrench[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(massage_motion::transform_wrench_to_reference(request).valid);
  request = identity_request();
  request.target_from_source_rotation.w = 2.0;
  EXPECT_FALSE(massage_motion::transform_wrench_to_reference(request).valid);
  request = identity_request();
  request.reference_origin_in_target.z =
    std::numeric_limits<double>::infinity();
  EXPECT_FALSE(massage_motion::transform_wrench_to_reference(request).valid);
}

}  // namespace
