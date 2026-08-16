#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "massage_motion/wrench_observation.hpp"

namespace massage_motion
{
namespace
{

void append_phase(
    std::vector<WrenchObservationSample> & samples,
    WrenchObservationPhase phase,
    double start_time,
    const WrenchVector & values)
{
    for (std::size_t index = 0; index < 6U; ++index)
    {
        WrenchObservationSample sample;
        sample.phase = phase;
        sample.receive_time = start_time + static_cast<double>(index) * 0.1;
        sample.stamp_nanoseconds = static_cast<std::int64_t>(
            sample.receive_time * 1e9);
        sample.frame_id = "Link_06";
        sample.values = values;
        samples.push_back(sample);
    }
}

TEST(WrenchObservationTest, ComputesLoadDeltaAndRecoveryOffset)
{
    std::vector<WrenchObservationSample> samples;
    append_phase(
        samples, WrenchObservationPhase::kBaseline, 0.0,
        WrenchVector{0.0, 0.0, 1.0, 0.0, 0.0, 0.1});
    append_phase(
        samples, WrenchObservationPhase::kLoad, 1.0,
        WrenchVector{2.0, 0.0, 1.0, 0.0, 0.3, 0.1});
    append_phase(
        samples, WrenchObservationPhase::kRecovery, 2.0,
        WrenchVector{0.1, 0.0, 1.0, 0.0, 0.0, 0.1});

    const auto report = analyze_wrench_observation(
        samples, "Link_06", 5.0, 0.2);
    ASSERT_TRUE(report.valid) << report.message;
    EXPECT_EQ(report.frame_id, "Link_06");
    EXPECT_NEAR(report.baseline.sample_rate, 10.0, 1e-9);
    EXPECT_NEAR(report.maximum_load_delta[0], 2.0, 1e-9);
    EXPECT_NEAR(report.maximum_load_delta[4], 0.3, 1e-9);
    EXPECT_NEAR(report.recovery_offset[0], 0.1, 1e-9);
}

TEST(WrenchObservationTest, RejectsFrameAndTimingFailures)
{
    std::vector<WrenchObservationSample> samples;
    append_phase(
        samples, WrenchObservationPhase::kBaseline, 0.0, WrenchVector{});
    append_phase(
        samples, WrenchObservationPhase::kLoad, 1.0, WrenchVector{});
    append_phase(
        samples, WrenchObservationPhase::kRecovery, 2.0, WrenchVector{});

    samples.front().frame_id = "wrong_frame";
    EXPECT_FALSE(analyze_wrench_observation(
        samples, "Link_06", 5.0, 0.2).valid);

    samples.front().frame_id = "Link_06";
    EXPECT_FALSE(analyze_wrench_observation(
        samples, "Link_06", 20.0, 0.2).valid);
}

TEST(WrenchObservationTest, RejectsOutOfOrderPhases)
{
    std::vector<WrenchObservationSample> samples;
    append_phase(
        samples, WrenchObservationPhase::kBaseline, 0.0, WrenchVector{});
    append_phase(
        samples, WrenchObservationPhase::kRecovery, 1.0, WrenchVector{});
    append_phase(
        samples, WrenchObservationPhase::kLoad, 2.0, WrenchVector{});

    const auto report = analyze_wrench_observation(
        samples, "Link_06", 5.0, 0.2);
    EXPECT_FALSE(report.valid);
    EXPECT_EQ(report.message, "FT 样本阶段顺序无效");
}

TEST(WrenchObservationTest, AnalyzesSinglePhaseForBaselineReuse)
{
    std::vector<WrenchObservationSample> samples;
    append_phase(
        samples, WrenchObservationPhase::kBaseline, 0.0,
        WrenchVector{0.1, 0.2, 0.3, 0.01, 0.02, 0.03});

    const auto report = analyze_wrench_phase(
        samples, "Link_06", 5.0, 0.2);
    ASSERT_TRUE(report.valid) << report.message;
    EXPECT_EQ(report.statistics.sample_count, 6U);
    EXPECT_NEAR(report.statistics.mean[2], 0.3, 1e-9);
    EXPECT_NEAR(report.statistics.sample_rate, 10.0, 1e-9);
}

}  // namespace
}  // namespace massage_motion
