#ifndef MASSAGE_JAKA__FORCE_HOLD_ANALYSIS_HPP_
#define MASSAGE_JAKA__FORCE_HOLD_ANALYSIS_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace massage_jaka
{

struct ForceHoldSample
{
    double elapsed{0.0};
    double normal_force{0.0};
};

struct NormalForceProjection
{
    bool valid{false};
    double normal_force{0.0};
    std::string message;
};

NormalForceProjection project_normal_force(
    double signed_force_delta,
    double expected_measured_force_sign,
    double opposite_force_rejection_threshold);

struct ForceHoldAcceptance
{
    double target_force{1.0};
    double tolerance{0.3};
    double maximum_force{5.0};
    double minimum_in_tolerance_ratio{0.8};
    double maximum_standard_deviation{0.25};
    double minimum_duration{2.5};
    double maximum_sample_gap{0.2};
    double maximum_mean_absolute_error{0.25};
    double maximum_terminal_error{0.3};
    std::size_t minimum_samples{20U};
};

struct ForceHoldReport
{
    bool valid{false};
    bool passed{false};
    std::string message;
    std::size_t sample_count{0U};
    double mean_force{0.0};
    double standard_deviation{0.0};
    double minimum_force{0.0};
    double maximum_force{0.0};
    double maximum_absolute_error{0.0};
    double mean_absolute_error{0.0};
    double terminal_absolute_error{0.0};
    double duration{0.0};
    double maximum_sample_gap{0.0};
    double in_tolerance_ratio{0.0};
};

ForceHoldReport analyze_force_hold(
    const std::vector<ForceHoldSample> & samples,
    const ForceHoldAcceptance & acceptance);

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__FORCE_HOLD_ANALYSIS_HPP_
