#include "massage_motion/wrench_processor.hpp"

#include <cmath>
#include <stdexcept>

namespace massage_motion
{

namespace
{

WrenchProcessingResult make_error(
    WrenchProcessingError error,
    const std::string & message)
{
    WrenchProcessingResult result;
    result.error = error;
    result.message = message;
    return result;
}

}  // namespace

WrenchProcessor::WrenchProcessor(
    const WrenchProcessorParameters & parameters)
    : parameters_(parameters)
{
    if (parameters_.expected_frame_id.empty())
    {
        throw std::invalid_argument("期望的力传感器 frame_id 不能为空");
    }

    if (parameters_.calibration_sample_count == 0)
    {
        throw std::invalid_argument("基线标定样本数量必须大于零");
    }

    if (!std::isfinite(parameters_.filter_alpha) ||
        parameters_.filter_alpha <= 0.0 ||
        parameters_.filter_alpha > 1.0)
    {
        throw std::invalid_argument("低通滤波 alpha 必须位于 (0, 1]");
    }

    for (const double limit : parameters_.max_absolute_raw_wrench)
    {
        if (!std::isfinite(limit) || limit <= 0.0)
        {
            throw std::invalid_argument("六维力绝对值限制必须是有限正数");
        }
    }
}

WrenchProcessingResult WrenchProcessor::add_calibration_sample(
    const WrenchSample & sample)
{
    if (calibrated_)
    {
        return make_error(
            WrenchProcessingError::kInvalidSample,
            "基线标定已经完成；如需重新标定，请先调用 reset()");
    }

    auto validation = validate_sample(sample);
    if (!validation.valid)
    {
        return validation;
    }

    for (std::size_t index = 0; index < kCartesianDof; ++index)
    {
        calibration_sum_[index] += sample.values[index];
    }

    ++calibration_samples_received_;
    last_stamp_nanoseconds_ = sample.stamp_nanoseconds;
    has_last_stamp_ = true;

    if (calibration_samples_received_ == parameters_.calibration_sample_count)
    {
        const double sample_count =
            static_cast<double>(calibration_samples_received_);

        for (std::size_t index = 0; index < kCartesianDof; ++index)
        {
            baseline_[index] = calibration_sum_[index] / sample_count;
        }

        calibrated_ = true;
    }

    WrenchProcessingResult result;
    result.valid = true;
    result.message = calibrated_ ? "基线标定完成" : "已接收基线样本";
    result.raw = sample.values;
    result.compensated = sample.values;
    result.filtered = sample.values;
    return result;
}

WrenchProcessingResult WrenchProcessor::process(const WrenchSample & sample)
{
    if (!calibrated_)
    {
        return make_error(
            WrenchProcessingError::kNotCalibrated,
            "处理六维力前必须先完成基线标定");
    }

    auto validation = validate_sample(sample);
    if (!validation.valid)
    {
        return validation;
    }

    WrenchProcessingResult result;
    result.raw = sample.values;

    for (std::size_t index = 0; index < kCartesianDof; ++index)
    {
        result.compensated[index] = sample.values[index] - baseline_[index];
    }

    if (!filter_initialized_)
    {
        filtered_ = result.compensated;
        filter_initialized_ = true;
    }
    else
    {
        for (std::size_t index = 0; index < kCartesianDof; ++index)
        {
            filtered_[index] =
                parameters_.filter_alpha * result.compensated[index] +
                (1.0 - parameters_.filter_alpha) * filtered_[index];
        }
    }

    last_stamp_nanoseconds_ = sample.stamp_nanoseconds;
    has_last_stamp_ = true;

    result.valid = true;
    result.message = "六维力处理成功";
    result.filtered = filtered_;
    return result;
}

void WrenchProcessor::reset()
{
    calibration_sum_ = {};
    baseline_ = {};
    filtered_ = {};
    calibration_samples_received_ = 0;
    last_stamp_nanoseconds_ = 0;
    has_last_stamp_ = false;
    calibrated_ = false;
    filter_initialized_ = false;
}

bool WrenchProcessor::calibrated() const
{
    return calibrated_;
}

std::size_t WrenchProcessor::calibration_samples_received() const
{
    return calibration_samples_received_;
}

const WrenchVector & WrenchProcessor::baseline() const
{
    return baseline_;
}

WrenchProcessingResult WrenchProcessor::validate_sample(
    const WrenchSample & sample) const
{
    if (sample.frame_id != parameters_.expected_frame_id)
    {
        return make_error(
            WrenchProcessingError::kFrameMismatch,
            "六维力样本 frame_id 与配置不一致");
    }

    if (sample.stamp_nanoseconds < 0)
    {
        return make_error(
            WrenchProcessingError::kInvalidSample,
            "六维力样本时间戳不能为负数");
    }

    if (has_last_stamp_ &&
        sample.stamp_nanoseconds <= last_stamp_nanoseconds_)
    {
        return make_error(
            WrenchProcessingError::kNonMonotonicTimestamp,
            "六维力样本时间戳必须严格递增");
    }

    for (std::size_t index = 0; index < kCartesianDof; ++index)
    {
        const double value = sample.values[index];
        if (!std::isfinite(value))
        {
            return make_error(
                WrenchProcessingError::kInvalidSample,
                "六维力样本包含非有限数值");
        }

        if (std::abs(value) > parameters_.max_absolute_raw_wrench[index])
        {
            return make_error(
                WrenchProcessingError::kLimitExceeded,
                "原始六维力超过绝对值限制");
        }
    }

    WrenchProcessingResult result;
    result.valid = true;
    return result;
}

}  // namespace massage_motion
