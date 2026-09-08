#ifndef MASSAGE_MOTION__COMPLIANCE_TYPES_HPP_
#define MASSAGE_MOTION__COMPLIANCE_TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace massage_motion
{

// 柔顺控制的六个自由度顺序固定为 X、Y、Z、RX、RY、RZ。
constexpr std::size_t kCartesianDof = 6;

enum class ComplianceStatus : std::int32_t
{
    kIdle = 0,
    kActive,
    kStopped,
    kFault
};

enum class ComplianceError : std::int32_t
{
    kNone = 0,
    kInvalidRequest,
    kBackendUnavailable,
    kFeedbackUnavailable,
    kAlreadyActive,
    kControlFailed,
    kLimitExceeded,
    kTimeout
};

// 该请求只描述所有柔顺后端都需要理解的公共语义。
// 具体的 Gazebo 控制器参数和 JAKA SDK 参数留在各自的适配器中。
struct ComplianceRequest
{
    std::string request_id;
    std::array<bool, kCartesianDof> enabled_axes{};
    std::array<double, kCartesianDof> target_wrench{};
    std::array<double, kCartesianDof> max_absolute_wrench{};
    double max_joint_displacement{0.0};
    double max_linear_displacement{0.0};
    double timeout{0.0};
};

struct ComplianceResult
{
    bool success{false};
    ComplianceError error{ComplianceError::kNone};
    std::int32_t backend_error_code{0};
    std::string message;
    ComplianceStatus status{ComplianceStatus::kIdle};
    // 后端在本次运行中观测到的六轴绝对峰值，便于故障复盘和任务层汇总。
    std::array<double, kCartesianDof> peak_absolute_wrench{};
    double peak_joint_displacement{0.0};
    double peak_selected_axis_translation{0.0};
};

struct ComplianceValidationResult
{
    bool valid{false};
    ComplianceError error{ComplianceError::kNone};
    std::string message;
};

struct ComplianceCapabilities
{
    // Can consume a time-varying nominal trajectory while retaining exclusive
    // ownership of the compliant contact phase.
    bool reference_tracking{false};
    // Can configure and maintain a force target internally.
    bool force_target_management{false};
};

// 柔顺阶段的名义关节参考。控制器负责按 joint_names 映射到自身关节顺序。
struct ComplianceReference
{
    std::vector<std::string> joint_names;
    std::vector<double> positions;
    std::vector<double> velocities;
    double time_from_start{0.0};
};

// 后端无关的柔顺反馈。wrench 顺序固定为 X、Y、Z、RX、RY、RZ。
struct ComplianceFeedback
{
    ComplianceStatus status{ComplianceStatus::kIdle};
    std::array<double, kCartesianDof> wrench{};
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::int64_t wrench_stamp_nanoseconds{0};
    double age{0.0};
    bool stale{true};
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__COMPLIANCE_TYPES_HPP_
