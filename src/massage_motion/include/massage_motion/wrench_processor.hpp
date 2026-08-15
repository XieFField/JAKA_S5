#ifndef MASSAGE_MOTION__WRENCH_PROCESSOR_HPP_
#define MASSAGE_MOTION__WRENCH_PROCESSOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "massage_motion/compliance_types.hpp"

namespace massage_motion
{

using WrenchVector = std::array<double, kCartesianDof>;

enum class WrenchProcessingError : std::int32_t
{
    kNone = 0,
    kInvalidSample,
    kFrameMismatch,
    kNonMonotonicTimestamp,
    kNotCalibrated,
    kLimitExceeded
};

// 与 ROS 消息无关的六维力样本，顺序固定为 Fx、Fy、Fz、Tx、Ty、Tz。
struct WrenchSample
{
    std::int64_t stamp_nanoseconds{0};
    std::string frame_id;
    WrenchVector values{};
};

struct WrenchProcessorParameters
{
    std::string expected_frame_id;
    std::size_t calibration_sample_count{100};

    // 一阶低通中当前样本的权重，必须位于 (0, 1]。
    // 该参数越小，输出越平滑，但相位滞后越明显。
    double filter_alpha{0.2};

    // 对原始六维力进行绝对值保护。这里的默认值仅表示未配置业务限制，
    // 真机和具体业务必须提供经过审批的更小阈值。
    WrenchVector max_absolute_raw_wrench{
        1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6};
};

struct WrenchProcessingResult
{
    bool valid{false};
    WrenchProcessingError error{WrenchProcessingError::kNone};
    std::string message;
    WrenchVector raw{};
    WrenchVector compensated{};
    WrenchVector filtered{};
};

// 固定姿态下的六维力预处理器。
//
// 标定阶段计算的 baseline 同时包含传感器零偏和当前姿态下的工具重力。
// 因此它只能用于本实验的固定姿态；机械臂姿态变化后，需要根据 TF 和
// 工具质量/质心做随姿态变化的重力补偿，不能继续套用同一组 baseline。
class WrenchProcessor
{
public:
    explicit WrenchProcessor(const WrenchProcessorParameters & parameters);

    WrenchProcessingResult add_calibration_sample(
        const WrenchSample & sample);

    WrenchProcessingResult process(const WrenchSample & sample);

    void reset();

    bool calibrated() const;
    std::size_t calibration_samples_received() const;
    const WrenchVector & baseline() const;

private:
    WrenchProcessingResult validate_sample(
        const WrenchSample & sample) const;

    WrenchProcessorParameters parameters_;
    WrenchVector calibration_sum_{};
    WrenchVector baseline_{};
    WrenchVector filtered_{};
    std::size_t calibration_samples_received_{0};
    std::int64_t last_stamp_nanoseconds_{0};
    bool has_last_stamp_{false};
    bool calibrated_{false};
    bool filter_initialized_{false};
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__WRENCH_PROCESSOR_HPP_
