#include "massage_jaka/tool_response_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace massage_jaka
{
namespace
{

double norm(const std::array<double, 3> & vector)
{
    return std::sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] +
        vector[2] * vector[2]);
}

}  // namespace

ToolResponseReport analyze_tool_response(
    const std::vector<ToolResponseSample> & samples,
    const std::array<double, 6> & baseline_wrench,
    const std::vector<double> & initial_joint_positions,
    const std::array<double, 3> & initial_translation,
    const std::array<double, 3> & response_axis,
    const ToolResponseLimits & limits)
{
    ToolResponseReport report;
    const double axis_norm = norm(response_axis);
    if (samples.empty() || limits.axis >= 3U || axis_norm <= 0.0 ||
        initial_joint_positions.empty())
    {
        report.message = "响应分析输入无效";
        return report;
    }

    const std::array<double, 3> unit_axis{
        response_axis[0] / axis_norm,
        response_axis[1] / axis_norm,
        response_axis[2] / axis_norm};

    for (const auto & sample : samples)
    {
        if (sample.joint_positions.size() != initial_joint_positions.size())
        {
            report.message = "响应样本的关节数量不一致";
            return report;
        }
        const double force_delta =
            sample.wrench[limits.axis] - baseline_wrench[limits.axis];
        if (std::abs(force_delta) > std::abs(report.peak_force_delta))
        {
            report.peak_force_delta = force_delta;
        }

        const std::array<double, 3> displacement{
            sample.translation[0] - initial_translation[0],
            sample.translation[1] - initial_translation[1],
            sample.translation[2] - initial_translation[2]};
        const double linear = norm(displacement);
        const double projection =
            displacement[0] * unit_axis[0] +
            displacement[1] * unit_axis[1] +
            displacement[2] * unit_axis[2];
        const double transverse = std::sqrt(std::max(
            0.0, linear * linear - projection * projection));
        report.maximum_axis_displacement = std::max(
            report.maximum_axis_displacement, std::abs(projection));
        report.maximum_transverse_displacement = std::max(
            report.maximum_transverse_displacement, transverse);
        report.maximum_linear_displacement = std::max(
            report.maximum_linear_displacement, linear);

        for (std::size_t index = 0;
            index < initial_joint_positions.size(); ++index)
        {
            report.maximum_joint_displacement = std::max(
                report.maximum_joint_displacement,
                std::abs(
                    sample.joint_positions[index] -
                    initial_joint_positions[index]));
        }
    }

    const double force_direction = report.peak_force_delta >= 0.0 ? 1.0 : -1.0;
    for (const auto & sample : samples)
    {
        const std::array<double, 3> displacement{
            sample.translation[0] - initial_translation[0],
            sample.translation[1] - initial_translation[1],
            sample.translation[2] - initial_translation[2]};
        const double projection =
            displacement[0] * unit_axis[0] +
            displacement[1] * unit_axis[1] +
            displacement[2] * unit_axis[2];
        report.directed_axis_displacement = std::max(
            report.directed_axis_displacement,
            force_direction * projection);
    }

    report.load_detected =
        std::abs(report.peak_force_delta) >= limits.minimum_force_delta;
    report.direction_correct = report.directed_axis_displacement > 0.0;
    report.response_detected = report.direction_correct &&
        report.directed_axis_displacement >=
        limits.minimum_axis_displacement;

    std::ostringstream message;
    if (!report.load_detected)
    {
        message << "未检测到足够的测试外力; ";
    }
    if (report.load_detected && !report.response_detected)
    {
        message << "检测到外力但未产生同方向最小位移; ";
    }
    if (report.maximum_linear_displacement >
        limits.maximum_linear_displacement)
    {
        message << "TCP 总位移超过上限; ";
    }
    if (report.maximum_transverse_displacement >
        limits.maximum_transverse_displacement)
    {
        message << "TCP 横向位移超过上限; ";
    }
    if (report.maximum_joint_displacement >
        limits.maximum_joint_displacement)
    {
        message << "关节位移超过上限; ";
    }

    report.passed = report.load_detected && report.response_detected &&
        report.maximum_linear_displacement <=
        limits.maximum_linear_displacement &&
        report.maximum_transverse_displacement <=
        limits.maximum_transverse_displacement &&
        report.maximum_joint_displacement <=
        limits.maximum_joint_displacement;
    report.message = report.passed ? "工具柔顺响应满足全部验收条件" :
        message.str();
    return report;
}

}  // namespace massage_jaka
