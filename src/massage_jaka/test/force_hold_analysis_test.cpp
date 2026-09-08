#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "massage_jaka/force_hold_analysis.hpp"

TEST(ForceHoldAnalysisTest, ProjectsOnlyExpectedContactDirection)
{
    const auto expected = massage_jaka::project_normal_force(0.8, 1.0, 0.5);
    const auto small_opposite =
        massage_jaka::project_normal_force(-0.1, 1.0, 0.5);
    const auto wrong_direction =
        massage_jaka::project_normal_force(-0.8, 1.0, 0.5);

    ASSERT_TRUE(expected.valid);
    EXPECT_DOUBLE_EQ(expected.normal_force, 0.8);
    ASSERT_TRUE(small_opposite.valid);
    EXPECT_DOUBLE_EQ(small_opposite.normal_force, 0.0);
    EXPECT_FALSE(wrong_direction.valid);
}

TEST(ForceHoldAnalysisTest, AcceptsStableForceNearTarget)
{
    std::vector<massage_jaka::ForceHoldSample> samples;
    for (std::size_t index = 0; index < 25U; ++index)
    {
        samples.push_back({0.15 * static_cast<double>(index),
            index % 2U == 0U ? 0.9 : 1.1});
    }

    const auto report = massage_jaka::analyze_force_hold(samples, {});

    EXPECT_TRUE(report.valid);
    EXPECT_TRUE(report.passed);
    EXPECT_EQ(report.sample_count, samples.size());
    EXPECT_GE(report.in_tolerance_ratio, 0.8);
    EXPECT_GE(report.duration, 2.5);
}

TEST(ForceHoldAnalysisTest, RejectsInsufficientSamples)
{
    const auto report = massage_jaka::analyze_force_hold(
        {{0.0, 1.0}, {0.02, 1.0}}, {});

    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.passed);
}

TEST(ForceHoldAnalysisTest, RejectsUnstableOrExcessiveForce)
{
    std::vector<massage_jaka::ForceHoldSample> samples;
    for (std::size_t index = 0; index < 20U; ++index)
    {
        samples.push_back({0.15 * static_cast<double>(index),
            index == 10U ? 5.1 : 0.4});
    }

    const auto report = massage_jaka::analyze_force_hold(samples, {});

    EXPECT_TRUE(report.valid);
    EXPECT_FALSE(report.passed);
    EXPECT_GT(report.maximum_force, 5.0);
}

TEST(ForceHoldAnalysisTest, RejectsNonFiniteSamples)
{
    std::vector<massage_jaka::ForceHoldSample> samples(20U, {0.0, 1.0});
    samples.back().normal_force = std::numeric_limits<double>::quiet_NaN();

    const auto report = massage_jaka::analyze_force_hold(samples, {});

    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.passed);
}

TEST(ForceHoldAnalysisTest, RejectsDuplicateOrReversedTimestamps)
{
    std::vector<massage_jaka::ForceHoldSample> samples;
    for (std::size_t index = 0; index < 20U; ++index)
    {
        samples.push_back({0.15 * static_cast<double>(index), 1.0});
    }
    samples[10].elapsed = samples[9].elapsed;

    const auto report = massage_jaka::analyze_force_hold(samples, {});

    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.passed);
}

TEST(ForceHoldAnalysisTest, RejectsExcessiveSampleGap)
{
    std::vector<massage_jaka::ForceHoldSample> samples;
    for (std::size_t index = 0; index < 20U; ++index)
    {
        samples.push_back({0.15 * static_cast<double>(index), 1.0});
    }
    samples[10].elapsed += 0.3;
    for (std::size_t index = 11U; index < samples.size(); ++index)
    {
        samples[index].elapsed += 0.3;
    }

    const auto report = massage_jaka::analyze_force_hold(samples, {});

    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.passed);
}

TEST(ForceHoldAnalysisTest, RejectsTooShortHoldWindow)
{
    std::vector<massage_jaka::ForceHoldSample> samples;
    for (std::size_t index = 0; index < 20U; ++index)
    {
        samples.push_back({0.05 * static_cast<double>(index), 1.0});
    }

    const auto report = massage_jaka::analyze_force_hold(samples, {});

    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.passed);
}
