#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "massage_motion/wrench_processor.hpp"

namespace massage_motion
{
namespace
{

constexpr const char * kSensorFrame =
    "jaka_s5/ft_sensor_joint/massage_ft_sensor";

WrenchSample make_sample(
    std::int64_t stamp_nanoseconds,
    const WrenchVector & values,
    const std::string & frame_id = kSensorFrame)
{
    return {stamp_nanoseconds, frame_id, values};
}

WrenchProcessorParameters make_parameters()
{
    WrenchProcessorParameters parameters;
    parameters.expected_frame_id = kSensorFrame;
    parameters.calibration_sample_count = 2;
    parameters.filter_alpha = 0.25;
    parameters.max_absolute_raw_wrench =
        {50.0, 50.0, 50.0, 5.0, 5.0, 5.0};
    return parameters;
}

TEST(WrenchProcessorTest, RejectsInvalidConfiguration)
{
    auto parameters = make_parameters();
    parameters.filter_alpha = 0.0;

    EXPECT_THROW(WrenchProcessor processor(parameters), std::invalid_argument);
}

TEST(WrenchProcessorTest, CalibrationComputesMeanBaseline)
{
    WrenchProcessor processor(make_parameters());

    ASSERT_TRUE(processor.add_calibration_sample(
        make_sample(10, {0.0, -2.0, 0.0, 0.08, 0.0, 0.0})).valid);
    ASSERT_TRUE(processor.add_calibration_sample(
        make_sample(20, {0.0, -1.8, 0.2, 0.10, 0.0, 0.0})).valid);

    ASSERT_TRUE(processor.calibrated());
    EXPECT_DOUBLE_EQ(processor.baseline()[1], -1.9);
    EXPECT_DOUBLE_EQ(processor.baseline()[2], 0.1);
    EXPECT_DOUBLE_EQ(processor.baseline()[3], 0.09);
}

TEST(WrenchProcessorTest, SubtractsBaselineAndFiltersStepInput)
{
    WrenchProcessor processor(make_parameters());
    ASSERT_TRUE(processor.add_calibration_sample(make_sample(10, {})).valid);
    ASSERT_TRUE(processor.add_calibration_sample(make_sample(20, {})).valid);

    const auto first = processor.process(
        make_sample(30, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    ASSERT_TRUE(first.valid) << first.message;
    EXPECT_DOUBLE_EQ(first.filtered[2], 0.0);

    const auto step = processor.process(
        make_sample(40, {0.0, 0.0, 8.0, 0.0, 0.0, 0.0}));
    ASSERT_TRUE(step.valid) << step.message;
    EXPECT_DOUBLE_EQ(step.compensated[2], 8.0);
    EXPECT_DOUBLE_EQ(step.filtered[2], 2.0);
}

TEST(WrenchProcessorTest, RejectsWrongFrameAndNonFiniteValue)
{
    WrenchProcessor processor(make_parameters());

    const auto wrong_frame = processor.add_calibration_sample(
        make_sample(10, {}, "wrong_frame"));
    EXPECT_FALSE(wrong_frame.valid);
    EXPECT_EQ(wrong_frame.error, WrenchProcessingError::kFrameMismatch);

    WrenchVector invalid{};
    invalid[0] = std::numeric_limits<double>::quiet_NaN();
    const auto non_finite = processor.add_calibration_sample(
        make_sample(10, invalid));
    EXPECT_FALSE(non_finite.valid);
    EXPECT_EQ(non_finite.error, WrenchProcessingError::kInvalidSample);
}

TEST(WrenchProcessorTest, RejectsNonMonotonicTimestamp)
{
    WrenchProcessor processor(make_parameters());
    ASSERT_TRUE(processor.add_calibration_sample(make_sample(10, {})).valid);

    const auto repeated =
        processor.add_calibration_sample(make_sample(10, {}));

    EXPECT_FALSE(repeated.valid);
    EXPECT_EQ(
        repeated.error,
        WrenchProcessingError::kNonMonotonicTimestamp);
}

TEST(WrenchProcessorTest, RejectsRawWrenchAboveLimit)
{
    WrenchProcessor processor(make_parameters());
    ASSERT_TRUE(processor.add_calibration_sample(make_sample(10, {})).valid);
    ASSERT_TRUE(processor.add_calibration_sample(make_sample(20, {})).valid);

    const auto result = processor.process(
        make_sample(30, {0.0, 0.0, 51.0, 0.0, 0.0, 0.0}));

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, WrenchProcessingError::kLimitExceeded);
}

TEST(WrenchProcessorTest, ResetRequiresNewCalibration)
{
    WrenchProcessor processor(make_parameters());
    ASSERT_TRUE(processor.add_calibration_sample(make_sample(10, {})).valid);
    ASSERT_TRUE(processor.add_calibration_sample(make_sample(20, {})).valid);

    processor.reset();

    EXPECT_FALSE(processor.calibrated());
    const auto result = processor.process(make_sample(30, {}));
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, WrenchProcessingError::kNotCalibrated);
}

}  // namespace
}  // namespace massage_motion
