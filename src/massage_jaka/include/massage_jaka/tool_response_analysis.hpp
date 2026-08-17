#ifndef MASSAGE_JAKA__TOOL_RESPONSE_ANALYSIS_HPP_
#define MASSAGE_JAKA__TOOL_RESPONSE_ANALYSIS_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace massage_jaka
{

struct ToolResponseSample
{
    double time{0.0};
    std::array<double, 6> wrench{};
    std::vector<double> joint_positions;
    std::array<double, 3> translation{};
};

struct ToolResponseLimits
{
    std::size_t axis{2};
    double minimum_force_delta{0.5};
    double minimum_axis_displacement{0.0002};
    double maximum_linear_displacement{0.002};
    double maximum_transverse_displacement{0.0005};
    double maximum_joint_displacement{0.005};
};

struct ToolResponseReport
{
    bool passed{false};
    bool load_detected{false};
    bool response_detected{false};
    bool direction_correct{false};
    double peak_force_delta{0.0};
    double directed_axis_displacement{0.0};
    double maximum_axis_displacement{0.0};
    double maximum_transverse_displacement{0.0};
    double maximum_linear_displacement{0.0};
    double maximum_joint_displacement{0.0};
    std::string message;
};

ToolResponseReport analyze_tool_response(
    const std::vector<ToolResponseSample> & samples,
    const std::array<double, 6> & baseline_wrench,
    const std::vector<double> & initial_joint_positions,
    const std::array<double, 3> & initial_translation,
    const std::array<double, 3> & response_axis,
    const ToolResponseLimits & limits);

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__TOOL_RESPONSE_ANALYSIS_HPP_
