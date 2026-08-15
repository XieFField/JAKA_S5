#include <array>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "massage_jaka/jaka_compliance_guard.hpp"

TEST(JakaComplianceGuardTest, ComputesMaximumJointAndLinearDisplacement)
{
    const auto result = massage_jaka::calculate_compliance_displacement(
        {0.0, 0.1, -0.2},
        {0.01, 0.08, -0.19},
        {0.5, 0.2, 0.1},
        {0.503, 0.204, 0.1});

    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.maximum_joint_displacement, 0.02, 1e-12);
    EXPECT_NEAR(result.linear_displacement, 0.005, 1e-12);
}

TEST(JakaComplianceGuardTest, RejectsArrayMismatch)
{
    const auto result = massage_jaka::calculate_compliance_displacement(
        {0.0, 0.1}, {0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    EXPECT_FALSE(result.valid);
}

TEST(JakaComplianceGuardTest, RejectsNonFiniteInput)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(massage_jaka::calculate_compliance_displacement(
        {0.0}, {nan}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0})
        .valid);
    EXPECT_FALSE(massage_jaka::calculate_compliance_displacement(
        {0.0}, {0.0}, {0.0, 0.0, 0.0}, {nan, 0.0, 0.0})
        .valid);
}

TEST(JakaComplianceGuardTest, AcceptsSampleInsideAllLimits)
{
    const auto result = massage_jaka::evaluate_compliance_guard(
        {0.0, 0.0},
        {0.01, -0.01},
        {0.0, 0.0, 0.0},
        {0.001, 0.002, 0.0},
        {1.0, 0.0, 2.0, 0.0, 0.0, 0.0},
        {2.0, 0.0, 3.0, 0.0, 0.0, 0.0},
        0.02,
        0.01);

    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.limit_exceeded);
    EXPECT_EQ(
        result.violation,
        massage_jaka::ComplianceGuardViolation::kNone);
}

TEST(JakaComplianceGuardTest, DistinguishesJointAndTcpLimits)
{
    auto result = massage_jaka::evaluate_compliance_guard(
        {0.0}, {0.03}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {}, {}, 0.02, 0.01);
    EXPECT_TRUE(result.limit_exceeded);
    EXPECT_EQ(
        result.violation,
        massage_jaka::ComplianceGuardViolation::kJointDisplacement);

    result = massage_jaka::evaluate_compliance_guard(
        {0.0}, {0.0}, {0.0, 0.0, 0.0}, {0.011, 0.0, 0.0},
        {}, {}, 0.02, 0.01);
    EXPECT_TRUE(result.limit_exceeded);
    EXPECT_EQ(
        result.violation,
        massage_jaka::ComplianceGuardViolation::kLinearDisplacement);
}

TEST(JakaComplianceGuardTest, ChecksEveryConfiguredWrenchAxis)
{
    const auto result = massage_jaka::evaluate_compliance_guard(
        {0.0}, {0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.1, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        0.02,
        0.01);

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.limit_exceeded);
    EXPECT_EQ(result.violation, massage_jaka::ComplianceGuardViolation::kWrench);
    EXPECT_EQ(result.wrench_axis, 4U);
}

TEST(JakaComplianceGuardTest, RejectsInvalidLimitsAndWrenchSamples)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto result = massage_jaka::evaluate_compliance_guard(
        {0.0}, {0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {}, {}, 0.0, 0.01);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(
        result.violation,
        massage_jaka::ComplianceGuardViolation::kInvalidSample);

    result = massage_jaka::evaluate_compliance_guard(
        {0.0}, {0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {nan, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.02, 0.01);
    EXPECT_FALSE(result.valid);
}
