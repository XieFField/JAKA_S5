#include "massage_motion/single_axis_admittance.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace massage_motion;
int main(int argc,char ** argv)
{
    SingleAxisAdmittanceParameters parameters;
    parameters.mass = 2.0;
    parameters.damping = 13.0;
    parameters.stiffness = 20.0;

    SingleAxisAdmittance sing_axis_admittance(parameters);

    double dt = 0.01;
    int num_steps = 400;

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("admittance_response_demo");

    for (int step = 0; step < num_steps; ++step)
    {
        // 本次输入对应的时间区间是 [current_time, current_time + dt)。
        const double current_time =
            static_cast<double>(step) * dt;

        double force_error = 0.0;

        if (current_time < 1.0)
        {
            // 第一阶段：0~1 s，没有力误差。
            force_error = 0.0;
        }
        else if (current_time < 2.0)
        {
            // 第二阶段：1~2 s，施加恒定的模拟力误差。
            force_error = 10.0;
        }
        else
        {
            // 第三阶段：2~4 s，撤去力误差，观察系统回弹。
            force_error = 0.0;
        }

        const auto result =
            sing_axis_admittance.update(force_error, dt);

        if (!result.valid)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "导纳计算失败: step=%d, time=%.2f, message=%s",
                step,
                current_time,
                result.message.c_str());

            rclcpp::shutdown();
            return 1;
        }

        const double state_time = current_time + dt;

        RCLCPP_INFO(
            node->get_logger(),
            "t=%.2f s, force_error=%.2f N, "
            "position=%.6f m, velocity=%.6f m/s, "
            "acceleration=%.6f m/s^2",
            state_time,
            force_error,
            result.state.position,
            result.state.velocity,
            result.state.acceleration);
    }

    rclcpp::shutdown();
    return 0;
}
