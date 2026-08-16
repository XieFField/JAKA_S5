#include "massage_motion/wrench_observation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace massage_motion
{

namespace
{

bool finite_vector(const WrenchVector & values)
{
    return std::all_of(
        values.begin(), values.end(),
        [](double value) {return std::isfinite(value);});
}

WrenchPhaseStatistics calculate_phase(
    const std::vector<const WrenchObservationSample *> & samples)
{
    WrenchPhaseStatistics result;
    result.sample_count = samples.size();
    if (samples.empty())
    {
        return result;
    }

    result.minimum = samples.front()->values;
    result.maximum = samples.front()->values;
    WrenchVector squared_difference_sum{};
    for (std::size_t sample_index = 0;
        sample_index < samples.size(); ++sample_index)
    {
        const auto & sample = *samples[sample_index];
        for (std::size_t axis = 0; axis < sample.values.size(); ++axis)
        {
            const double delta = sample.values[axis] - result.mean[axis];
            result.mean[axis] += delta /
                static_cast<double>(sample_index + 1U);
            const double updated_delta =
                sample.values[axis] - result.mean[axis];
            squared_difference_sum[axis] += delta * updated_delta;
            result.minimum[axis] = std::min(
                result.minimum[axis], sample.values[axis]);
            result.maximum[axis] = std::max(
                result.maximum[axis], sample.values[axis]);
        }
        if (sample_index > 0U)
        {
            result.maximum_gap = std::max(
                result.maximum_gap,
                sample.receive_time - samples[sample_index - 1U]->receive_time);
        }
    }

    if (samples.size() > 1U)
    {
        result.duration =
            samples.back()->receive_time - samples.front()->receive_time;
        if (result.duration > 0.0)
        {
            result.sample_rate =
                static_cast<double>(samples.size() - 1U) / result.duration;
        }
    }
    for (std::size_t axis = 0; axis < result.mean.size(); ++axis)
    {
        result.standard_deviation[axis] = std::sqrt(
            squared_difference_sum[axis] /
            static_cast<double>(samples.size()));
    }
    return result;
}

bool phase_passes_timing(
    const WrenchPhaseStatistics & statistics,
    double minimum_sample_rate,
    double maximum_sample_gap,
    const char * phase,
    std::string & error)
{
    if (statistics.sample_count < 2U || !(statistics.duration > 0.0))
    {
        error = std::string(phase) + " 阶段有效样本不足";
        return false;
    }
    if (minimum_sample_rate > 0.0 &&
        statistics.sample_rate < minimum_sample_rate)
    {
        error = std::string(phase) + " 阶段采样频率低于门限";
        return false;
    }
    if (maximum_sample_gap > 0.0 &&
        statistics.maximum_gap > maximum_sample_gap)
    {
        error = std::string(phase) + " 阶段最大采样间隔超过门限";
        return false;
    }
    return true;
}

}  // namespace

WrenchPhaseReport analyze_wrench_phase(
    const std::vector<WrenchObservationSample> & samples,
    const std::string & expected_frame_id,
    double minimum_sample_rate,
    double maximum_sample_gap)
{
    WrenchPhaseReport report;
    if (!std::isfinite(minimum_sample_rate) || minimum_sample_rate < 0.0 ||
        !std::isfinite(maximum_sample_gap) || maximum_sample_gap < 0.0)
    {
        report.message = "FT 阶段时间门限必须是有限非负数";
        return report;
    }
    if (samples.empty())
    {
        report.message = "FT 阶段样本不能为空";
        return report;
    }

    std::vector<const WrenchObservationSample *> sample_pointers;
    sample_pointers.reserve(samples.size());
    double previous_receive_time = -std::numeric_limits<double>::infinity();
    for (const auto & sample : samples)
    {
        if (sample.frame_id.empty() ||
            (!expected_frame_id.empty() &&
            sample.frame_id != expected_frame_id) ||
            !std::isfinite(sample.receive_time) ||
            sample.receive_time < previous_receive_time ||
            !finite_vector(sample.values))
        {
            report.message = "FT 阶段样本坐标系、时间顺序或六维数值无效";
            return report;
        }
        if (report.frame_id.empty())
        {
            report.frame_id = sample.frame_id;
        }
        else if (sample.frame_id != report.frame_id)
        {
            report.message = "FT 阶段坐标系发生变化";
            return report;
        }
        sample_pointers.push_back(&sample);
        previous_receive_time = sample.receive_time;
    }

    report.statistics = calculate_phase(sample_pointers);
    if (!phase_passes_timing(
            report.statistics, minimum_sample_rate, maximum_sample_gap,
            "single", report.message))
    {
        return report;
    }
    report.valid = true;
    report.message = "FT 单阶段观测数据有效";
    return report;
}

WrenchObservationReport analyze_wrench_observation(
    const std::vector<WrenchObservationSample> & samples,
    const std::string & expected_frame_id,
    double minimum_sample_rate,
    double maximum_sample_gap)
{
    WrenchObservationReport report;
    if (!std::isfinite(minimum_sample_rate) || minimum_sample_rate < 0.0 ||
        !std::isfinite(maximum_sample_gap) || maximum_sample_gap < 0.0)
    {
        report.message = "FT 观测时间门限必须是有限非负数";
        return report;
    }
    if (samples.empty())
    {
        report.message = "FT 观测样本不能为空";
        return report;
    }

    std::array<std::vector<const WrenchObservationSample *>, 3> phases;
    double previous_receive_time = -std::numeric_limits<double>::infinity();
    std::size_t previous_phase_index = 0U;
    bool first_sample = true;
    for (const auto & sample : samples)
    {
        if (sample.frame_id.empty() ||
            (!expected_frame_id.empty() &&
            sample.frame_id != expected_frame_id) ||
            !std::isfinite(sample.receive_time) ||
            sample.receive_time < previous_receive_time ||
            !finite_vector(sample.values))
        {
            report.message = "FT 样本坐标系、时间顺序或六维数值无效";
            return report;
        }
        if (report.frame_id.empty())
        {
            report.frame_id = sample.frame_id;
        }
        else if (sample.frame_id != report.frame_id)
        {
            report.message = "FT 观测期间坐标系发生变化";
            return report;
        }

        const auto phase_index = static_cast<std::size_t>(sample.phase);
        if (phase_index >= phases.size())
        {
            report.message = "FT 样本阶段无效";
            return report;
        }
        if (!first_sample && phase_index < previous_phase_index)
        {
            report.message = "FT 样本阶段顺序无效";
            return report;
        }
        phases[phase_index].push_back(&sample);
        previous_phase_index = phase_index;
        first_sample = false;
        previous_receive_time = sample.receive_time;
    }

    report.baseline = calculate_phase(phases[0]);
    report.load = calculate_phase(phases[1]);
    report.recovery = calculate_phase(phases[2]);
    if (!phase_passes_timing(
            report.baseline, minimum_sample_rate, maximum_sample_gap,
            "baseline", report.message) ||
        !phase_passes_timing(
            report.load, minimum_sample_rate, maximum_sample_gap,
            "load", report.message) ||
        !phase_passes_timing(
            report.recovery, minimum_sample_rate, maximum_sample_gap,
            "recovery", report.message))
    {
        return report;
    }

    for (const auto * sample : phases[1])
    {
        for (std::size_t axis = 0;
            axis < report.maximum_load_delta.size(); ++axis)
        {
            report.maximum_load_delta[axis] = std::max(
                report.maximum_load_delta[axis],
                std::abs(sample->values[axis] - report.baseline.mean[axis]));
        }
    }
    for (std::size_t axis = 0; axis < report.recovery_offset.size(); ++axis)
    {
        report.recovery_offset[axis] = std::abs(
            report.recovery.mean[axis] - report.baseline.mean[axis]);
    }

    report.valid = true;
    report.message = "FT 三阶段观测数据有效";
    return report;
}

const char * to_string(WrenchObservationPhase phase)
{
    switch (phase)
    {
        case WrenchObservationPhase::kBaseline:
            return "baseline";
        case WrenchObservationPhase::kLoad:
            return "load";
        case WrenchObservationPhase::kRecovery:
            return "recovery";
    }
    return "unknown";
}

}  // namespace massage_motion
