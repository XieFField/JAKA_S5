#ifndef MASSAGE_JAKA__JAKA_COMPLIANCE_GUARD_HPP_
#define MASSAGE_JAKA__JAKA_COMPLIANCE_GUARD_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace massage_jaka
{

struct ComplianceDisplacementResult
{
    bool valid{false};
    double maximum_joint_displacement{0.0};
    double linear_displacement{0.0};
    std::string message;
};

enum class ComplianceGuardViolation
{
    kNone = 0,
    kInvalidSample,
    kJointDisplacement,
    kLinearDisplacement,
    kWrench
};

struct ComplianceGuardResult
{
    bool valid{false};
    bool limit_exceeded{false};
    ComplianceGuardViolation violation{ComplianceGuardViolation::kNone};
    std::size_t wrench_axis{0};
    ComplianceDisplacementResult displacement;
    std::string message;
};

ComplianceDisplacementResult calculate_compliance_displacement(
    const std::vector<double> & initial_joints,
    const std::vector<double> & current_joints,
    const std::array<double, 3> & initial_translation,
    const std::array<double, 3> & current_translation);

// 对单次真机反馈执行后端无关的保护判定，便于在无真机环境中完整测试。
ComplianceGuardResult evaluate_compliance_guard(
    const std::vector<double> & initial_joints,
    const std::vector<double> & current_joints,
    const std::array<double, 3> & initial_translation,
    const std::array<double, 3> & current_translation,
    const std::array<double, 6> & wrench,
    const std::array<double, 6> & maximum_absolute_wrench,
    double maximum_joint_displacement,
    double maximum_linear_displacement);

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__JAKA_COMPLIANCE_GUARD_HPP_
