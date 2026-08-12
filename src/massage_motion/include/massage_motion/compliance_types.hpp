#ifndef MASSAGE_MOTION__COMPLIANCE_TYPES_HPP_
#define MASSAGE_MOTION__COMPLIANCE_TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

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
    kAlreadyActive,
    kControlFailed,
    kLimitExceeded
};

// 该请求只描述所有柔顺后端都需要理解的公共语义。
// 具体的 Gazebo 控制器参数和 JAKA SDK 参数留在各自的适配器中。
struct ComplianceRequest
{
    std::string request_id;
    std::array<bool, kCartesianDof> enabled_axes{};
    std::array<double, kCartesianDof> target_wrench{};
    std::array<double, kCartesianDof> max_absolute_wrench{};
    double timeout{0.0};
};

struct ComplianceResult
{
    bool success{false};
    ComplianceError error{ComplianceError::kNone};
    std::int32_t backend_error_code{0};
    std::string message;
    ComplianceStatus status{ComplianceStatus::kIdle};
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__COMPLIANCE_TYPES_HPP_
