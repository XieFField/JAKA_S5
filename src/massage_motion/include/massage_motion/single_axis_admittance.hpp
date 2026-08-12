#ifndef MASSAGE_MOTION__SINGLE_AXIS_ADMITTANCE_HPP_
#define MASSAGE_MOTION__SINGLE_AXIS_ADMITTANCE_HPP_

#include <string>

namespace massage_motion
{

// 单轴导纳模型：
//   mass * acceleration + damping * velocity + stiffness * position
//       = external_force
struct SingleAxisAdmittanceParameters
{
    double mass{1.0};
    double damping{0.0};
    double stiffness{0.0};
};

struct SingleAxisAdmittanceState
{
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

struct SingleAxisAdmittanceUpdate
{
    bool valid{false};
    std::string message;
    SingleAxisAdmittanceState state;
};

// 这是纯算法类，不读取传感器，也不向机器人发送命令。
// 后续仿真和真机适配器负责提供力误差与固定周期 dt。
class SingleAxisAdmittance
{
public:
    explicit SingleAxisAdmittance(
        const SingleAxisAdmittanceParameters & parameters);

    SingleAxisAdmittanceUpdate update(
        double external_force,
        double dt);

    void reset();

    const SingleAxisAdmittanceState & state() const;

private:
    SingleAxisAdmittanceParameters parameters_;
    SingleAxisAdmittanceState state_;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__SINGLE_AXIS_ADMITTANCE_HPP_
