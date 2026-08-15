#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "massage_motion/single_axis_admittance.hpp"
#include "massage_motion/wrench_processor.hpp"

namespace massage_motion
{

class ForceAdmittanceDemo : public rclcpp::Node
{
public:
    ForceAdmittanceDemo()
        : Node("force_admittance_demo"),
          wrench_processor_(make_wrench_parameters()),
          admittance_(make_admittance_parameters())
    {
        subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/massage/ft_sensor/wrench_raw",
            rclcpp::SensorDataQoS(),
            std::bind(
                &ForceAdmittanceDemo::wrench_callback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "实验四启动：先用 100 个样本标定固定姿态基线；本节点只计算虚拟位移，不控制机械臂");
    }

private:
    static WrenchProcessorParameters make_wrench_parameters()
    {
        WrenchProcessorParameters parameters;
        parameters.expected_frame_id =
            "jaka_s5/ft_sensor_joint/massage_ft_sensor";
        parameters.calibration_sample_count = 100;
        parameters.filter_alpha = 0.2;
        parameters.max_absolute_raw_wrench = {
            50.0, 50.0, 50.0,
            5.0, 5.0, 5.0};
        return parameters;
    }

    static SingleAxisAdmittanceParameters make_admittance_parameters()
    {
        SingleAxisAdmittanceParameters parameters;
        parameters.mass = 2.0;
        parameters.damping = 13.0;
        parameters.stiffness = 20.0;
        return parameters;
    }

    static WrenchSample to_wrench_sample(
        const geometry_msgs::msg::WrenchStamped & message)
    {
        WrenchSample sample;
        sample.stamp_nanoseconds =
            rclcpp::Time(message.header.stamp).nanoseconds();
        sample.frame_id = message.header.frame_id;
        sample.values = {
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z};
        return sample;
    }

    void wrench_callback(
        const geometry_msgs::msg::WrenchStamped::SharedPtr message)
    {
        const WrenchSample sample = to_wrench_sample(*message);

        if (!wrench_processor_.calibrated())
        {
            const auto calibration =
                wrench_processor_.add_calibration_sample(sample);

            if (!calibration.valid)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "基线样本无效: %s",
                    calibration.message.c_str());
                return;
            }

            if (wrench_processor_.calibrated())
            {
                RCLCPP_INFO(
                    get_logger(),
                    "基线标定完成，下一帧开始计算传感器时间戳 dt");
            }
            return;
        }

        const auto wrench_result = wrench_processor_.process(sample);
        if (!wrench_result.valid)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "六维力处理失败: error=%d, message=%s",
                static_cast<int>(wrench_result.error),
                wrench_result.message.c_str());
            reset_dynamic_state();
            return;
        }

        // 第一帧只能建立时间基准，还不能计算 dt。
        if (!last_processed_stamp_nanoseconds_.has_value())
        {
            last_processed_stamp_nanoseconds_ = sample.stamp_nanoseconds;
            return;
        }

        const double dt = static_cast<double>(
            sample.stamp_nanoseconds -
            last_processed_stamp_nanoseconds_.value()) * 1.0e-9;
        last_processed_stamp_nanoseconds_ = sample.stamp_nanoseconds;

        // 时间间隔过大时，继续积分会造成虚拟状态跳变，因此先复位。
        if (!std::isfinite(dt) || dt <= 0.0 || dt > max_dt_seconds_)
        {
            RCLCPP_WARN(
                get_logger(),
                "控制周期异常: dt=%.6f s，导纳动态状态已复位",
                dt);
            reset_dynamic_state();
            return;
        }

        // TODO(练习 1)：从 wrench_result.filtered 中取出局部传感器 Z 轴测量力。
        const double measured_force_z = wrench_result.filtered[2];

        // TODO(练习 2)：按“目标力 - 测量力”计算带符号的力误差。
        const double force_error = target_force_z_newtons_ - measured_force_z;

        // TODO(练习 3)：把上面的力误差和传感器时间戳 dt 输入单轴导纳模型。
        const auto update = admittance_.update(force_error, dt);

        if (!update.valid)
        {
            RCLCPP_ERROR(
                get_logger(),
                "导纳更新失败: %s",
                update.message.c_str());
            reset_dynamic_state();
            return;
        }

        // 这里限制的是尚未发送给机器人的虚拟状态，只用于验证保护逻辑。
        if (std::abs(update.state.position) > max_virtual_position_meters_ ||
            std::abs(update.state.velocity) > max_virtual_velocity_mps_)
        {
            RCLCPP_ERROR(
                get_logger(),
                "虚拟状态越界: position=%.6f m, velocity=%.6f m/s，已复位",
                update.state.position,
                update.state.velocity);
            reset_dynamic_state();
            return;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 200,
            "target=%.3f N, measured=%.3f N, error=%.3f N, "
            "dt=%.4f s, x=%.6f m, v=%.6f m/s, a=%.6f m/s^2",
            target_force_z_newtons_,
            measured_force_z,
            force_error,
            dt,
            update.state.position,
            update.state.velocity,
            update.state.acceleration);
    }

    void reset_dynamic_state()
    {
        admittance_.reset();
        last_processed_stamp_nanoseconds_.reset();
    }

    WrenchProcessor wrench_processor_;
    SingleAxisAdmittance admittance_;
    std::optional<std::int64_t> last_processed_stamp_nanoseconds_;

    // 实验四先以标定后的 Fz=0 N 为参考：施加局部 -10 N 后，按
    // “目标力 - 测量力”会得到 +10 N，可直接对照此前的合成力实验。
    // 非零接触力目标留到实验五的闭环中使用。
    const double target_force_z_newtons_{0.0};
    const double max_dt_seconds_{0.05};
    const double max_virtual_position_meters_{0.05};
    const double max_virtual_velocity_mps_{0.5};

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
        subscription_;
};

}  // namespace massage_motion

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<massage_motion::ForceAdmittanceDemo>());
    rclcpp::shutdown();
    return 0;
}
