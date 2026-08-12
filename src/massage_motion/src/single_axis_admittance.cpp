#include "massage_motion/single_axis_admittance.hpp"

#include <cmath>
#include <stdexcept>

namespace massage_motion
{

SingleAxisAdmittance::SingleAxisAdmittance(
    const SingleAxisAdmittanceParameters & parameters)
    : parameters_(parameters)
{
    if (!std::isfinite(parameters_.mass) || parameters_.mass <= 0.0)
    {
        throw std::invalid_argument("导纳质量参数必须是有限正数");
    }

    if (!std::isfinite(parameters_.damping) || parameters_.damping < 0.0)
    {
        throw std::invalid_argument("导纳阻尼参数必须是有限非负数");
    }

    if (!std::isfinite(parameters_.stiffness) || parameters_.stiffness < 0.0)
    {
        throw std::invalid_argument("导纳刚度参数必须是有限非负数");
    }
}

SingleAxisAdmittanceUpdate SingleAxisAdmittance::update(
    double external_force,
    double dt)
{
    if (!std::isfinite(external_force))
    {
        return {false, "外力输入必须是有限数值", state_};
    }

    if (!std::isfinite(dt) || dt <= 0.0)
    {
        return {false, "控制周期 dt 必须是有限正数", state_};
    }

    // 半隐式欧拉积分：先更新速度，再使用新速度更新位置。
    // 它比完全显式欧拉在当前教学实验中更稳定，后续仍需按真实控制周期调参。
    const double acceleration =
        (external_force -
        parameters_.damping * state_.velocity -
        parameters_.stiffness * state_.position) /
        parameters_.mass;

    const double velocity = state_.velocity + acceleration * dt;
    const double position = state_.position + velocity * dt;

    if (!std::isfinite(acceleration) ||
        !std::isfinite(velocity) ||
        !std::isfinite(position))
    {
        return {false, "导纳计算产生了非有限状态", state_};
    }

    state_ = {position, velocity, acceleration};
    return {true, "更新成功", state_};
}

void SingleAxisAdmittance::reset()
{
    state_ = {};
}

const SingleAxisAdmittanceState & SingleAxisAdmittance::state() const
{
    return state_;
}

}  // namespace massage_motion
