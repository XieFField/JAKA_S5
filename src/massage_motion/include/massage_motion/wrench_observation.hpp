#ifndef MASSAGE_MOTION__WRENCH_OBSERVATION_HPP_
#define MASSAGE_MOTION__WRENCH_OBSERVATION_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "massage_motion/wrench_processor.hpp"

namespace massage_motion
{

// FT 被动观测的三个固定阶段。该类型只描述数据，不执行任何控制。
enum class WrenchObservationPhase
{
    kBaseline,
    kLoad,
    kRecovery,
};

struct WrenchObservationSample
{
    WrenchObservationPhase phase{WrenchObservationPhase::kBaseline};
    std::int64_t stamp_nanoseconds{0};
    double receive_time{0.0};
    std::string frame_id;
    WrenchVector values{};
};

struct WrenchPhaseStatistics
{
    std::size_t sample_count{0};
    double duration{0.0};
    double sample_rate{0.0};
    double maximum_gap{0.0};
    WrenchVector mean{};
    WrenchVector standard_deviation{};
    WrenchVector minimum{};
    WrenchVector maximum{};
};

struct WrenchObservationReport
{
    bool valid{false};
    std::string message;
    std::string frame_id;
    WrenchPhaseStatistics baseline;
    WrenchPhaseStatistics load;
    WrenchPhaseStatistics recovery;
    // 加载阶段相对基线均值的各轴最大绝对变化。
    WrenchVector maximum_load_delta{};
    // 卸载恢复阶段均值相对基线均值的各轴绝对偏差。
    WrenchVector recovery_offset{};
};

struct WrenchPhaseReport
{
    bool valid{false};
    std::string message;
    std::string frame_id;
    WrenchPhaseStatistics statistics;
};

// 分析单个连续阶段，供柔顺启用前的基线检查等场景复用。
WrenchPhaseReport analyze_wrench_phase(
    const std::vector<WrenchObservationSample> & samples,
    const std::string & expected_frame_id,
    double minimum_sample_rate,
    double maximum_sample_gap);

// 对已经分段的 FT 样本做纯数据分析。minimum_sample_rate 和
// maximum_sample_gap 取 0 时关闭对应门禁。
WrenchObservationReport analyze_wrench_observation(
    const std::vector<WrenchObservationSample> & samples,
    const std::string & expected_frame_id,
    double minimum_sample_rate,
    double maximum_sample_gap);

const char * to_string(WrenchObservationPhase phase);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__WRENCH_OBSERVATION_HPP_
