#include <array>
#include <vector>

#include "gtest/gtest.h"

#include "massage_jaka/tool_response_analysis.hpp"

namespace
{

massage_jaka::ToolResponseSample sample(double force_z, double translation_z)
{
    massage_jaka::ToolResponseSample result;
    result.wrench[2] = force_z;
    result.joint_positions = std::vector<double>(6, 0.0);
    result.translation[2] = translation_z;
    return result;
}

const std::array<double, 6> kBaseline{};
const std::vector<double> kInitialJoints(6, 0.0);
const std::array<double, 3> kInitialTranslation{};
const std::array<double, 3> kToolZ{0.0, 0.0, 1.0};

}  // namespace

TEST(ToolResponseAnalysisTest, AcceptsBoundedMotionAlongAppliedForce)
{
    const std::vector<massage_jaka::ToolResponseSample> samples{
        sample(0.0, 0.0), sample(-2.0, -0.0004),
        sample(-3.0, -0.0010)};

    const auto report = massage_jaka::analyze_tool_response(
        samples, kBaseline, kInitialJoints, kInitialTranslation, kToolZ, {});

    EXPECT_TRUE(report.passed);
    EXPECT_TRUE(report.direction_correct);
    EXPECT_NEAR(report.directed_axis_displacement, 0.001, 1e-12);
}

TEST(ToolResponseAnalysisTest, RejectsForceWithoutMotion)
{
    const std::vector<massage_jaka::ToolResponseSample> samples{
        sample(0.0, 0.0), sample(-3.0, 0.0)};

    const auto report = massage_jaka::analyze_tool_response(
        samples, kBaseline, kInitialJoints, kInitialTranslation, kToolZ, {});

    EXPECT_FALSE(report.passed);
    EXPECT_TRUE(report.load_detected);
    EXPECT_FALSE(report.response_detected);
}

TEST(ToolResponseAnalysisTest, RejectsWrongDirectionAndExcessiveTransverseMotion)
{
    auto wrong_direction = sample(-2.0, 0.0004);
    auto transverse = sample(-3.0, -0.0004);
    transverse.translation[0] = 0.001;

    const auto report = massage_jaka::analyze_tool_response(
        {wrong_direction, transverse}, kBaseline, kInitialJoints,
        kInitialTranslation, kToolZ, {});

    EXPECT_FALSE(report.passed);
    EXPECT_GT(report.maximum_transverse_displacement, 0.0005);
}
