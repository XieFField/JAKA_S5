#include "massage_jaka/force_hold_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace massage_jaka
{

NormalForceProjection project_normal_force(
    double signed_force_delta,
    double expected_measured_force_sign,
    double opposite_force_rejection_threshold)
{
    NormalForceProjection result;
    if (!std::isfinite(signed_force_delta) ||
        !std::isfinite(expected_measured_force_sign) ||
        std::abs(std::abs(expected_measured_force_sign) - 1.0) > 1.0e-9 ||
        !std::isfinite(opposite_force_rejection_threshold) ||
        opposite_force_rejection_threshold <= 0.0)
    {
        result.message = "法向力投影参数无效";
        return result;
    }

    const double projected =
        expected_measured_force_sign * signed_force_delta;
    if (projected <= -opposite_force_rejection_threshold)
    {
        result.message = "检测到与预期接触方向相反的法向力";
        return result;
    }
    result.valid = true;
    result.normal_force = std::max(0.0, projected);
    result.message = "法向力方向有效";
    return result;
}

ForceHoldReport analyze_force_hold(
    const std::vector<ForceHoldSample> & samples,
    const ForceHoldAcceptance & acceptance)
{
    ForceHoldReport report;
    const bool config_valid =
        std::isfinite(acceptance.target_force) && acceptance.target_force > 0.0 &&
        std::isfinite(acceptance.tolerance) && acceptance.tolerance > 0.0 &&
        std::isfinite(acceptance.maximum_force) &&
        acceptance.maximum_force > acceptance.target_force &&
        std::isfinite(acceptance.minimum_in_tolerance_ratio) &&
        acceptance.minimum_in_tolerance_ratio >= 0.0 &&
        acceptance.minimum_in_tolerance_ratio <= 1.0 &&
        std::isfinite(acceptance.maximum_standard_deviation) &&
        acceptance.maximum_standard_deviation >= 0.0 &&
        std::isfinite(acceptance.minimum_duration) &&
        acceptance.minimum_duration > 0.0 &&
        std::isfinite(acceptance.maximum_sample_gap) &&
        acceptance.maximum_sample_gap > 0.0 &&
        std::isfinite(acceptance.maximum_mean_absolute_error) &&
        acceptance.maximum_mean_absolute_error >= 0.0 &&
        std::isfinite(acceptance.maximum_terminal_error) &&
        acceptance.maximum_terminal_error >= 0.0 &&
        acceptance.minimum_samples > 0U;
    if (!config_valid)
    {
        report.message = "恒力保持验收配置无效";
        return report;
    }
    if (samples.size() < acceptance.minimum_samples)
    {
        report.message = "恒力保持样本不足";
        return report;
    }
    double previous_elapsed = -1.0;
    for (const auto & sample : samples)
    {
        if (!std::isfinite(sample.elapsed) || sample.elapsed < 0.0 ||
            !std::isfinite(sample.normal_force) || sample.normal_force < 0.0)
        {
            report.message = "恒力保持样本包含无效数值";
            return report;
        }
        if (previous_elapsed >= 0.0)
        {
            if (sample.elapsed <= previous_elapsed)
            {
                report.message = "恒力保持样本时间戳未严格递增";
                return report;
            }
            report.maximum_sample_gap = std::max(
                report.maximum_sample_gap, sample.elapsed - previous_elapsed);
        }
        previous_elapsed = sample.elapsed;
    }

    report.duration = samples.back().elapsed - samples.front().elapsed;
    if (report.duration < acceptance.minimum_duration)
    {
        report.message = "恒力保持有效时长不足";
        return report;
    }
    if (report.maximum_sample_gap > acceptance.maximum_sample_gap)
    {
        report.message = "恒力保持反馈样本间隔超过上限";
        return report;
    }

    report.valid = true;
    report.sample_count = samples.size();
    report.minimum_force = std::numeric_limits<double>::infinity();
    double sum = 0.0;
    double absolute_error_sum = 0.0;
    std::size_t in_tolerance = 0U;
    for (const auto & sample : samples)
    {
        const double error = std::abs(sample.normal_force - acceptance.target_force);
        sum += sample.normal_force;
        absolute_error_sum += error;
        report.minimum_force = std::min(report.minimum_force, sample.normal_force);
        report.maximum_force = std::max(report.maximum_force, sample.normal_force);
        report.maximum_absolute_error = std::max(report.maximum_absolute_error, error);
        if (error <= acceptance.tolerance)
        {
            ++in_tolerance;
        }
    }
    report.mean_force = sum / static_cast<double>(samples.size());
    report.mean_absolute_error =
        absolute_error_sum / static_cast<double>(samples.size());
    report.terminal_absolute_error = std::abs(
        samples.back().normal_force - acceptance.target_force);
    double squared_error_sum = 0.0;
    for (const auto & sample : samples)
    {
        const double delta = sample.normal_force - report.mean_force;
        squared_error_sum += delta * delta;
    }
    report.standard_deviation = std::sqrt(
        squared_error_sum / static_cast<double>(samples.size()));
    report.in_tolerance_ratio =
        static_cast<double>(in_tolerance) / static_cast<double>(samples.size());
    report.passed =
        report.maximum_force < acceptance.maximum_force &&
        report.in_tolerance_ratio >= acceptance.minimum_in_tolerance_ratio &&
        report.standard_deviation <= acceptance.maximum_standard_deviation &&
        report.mean_absolute_error <= acceptance.maximum_mean_absolute_error &&
        report.terminal_absolute_error <= acceptance.maximum_terminal_error;
    report.message = report.passed ?
        "恒力保持统计通过" : "恒力保持统计未通过";
    return report;
}

}  // namespace massage_jaka
