#include <cstdint>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "massage_motion/wrench_processor.hpp"

using namespace massage_motion;
using WrenchStamped = geometry_msgs::msg::WrenchStamped;

class WrenchProcessingDemo : public rclcpp::Node
{
public:
    WrenchProcessingDemo()
        : Node("wrench_processing_demo"),
          processor_(make_processor_parameters())
    {
        subscription_ =
            create_subscription<geometry_msgs::msg::WrenchStamped>(
                "/massage/ft_sensor/wrench_raw",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &WrenchProcessingDemo::wrench_callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "开始监听 /massage/ft_sensor/wrench_raw");
    }

private:
    static massage_motion::WrenchProcessorParameters
    make_processor_parameters()
    {
        massage_motion::WrenchProcessorParameters parameters;

        parameters.expected_frame_id =
            "jaka_s5/ft_sensor_joint/massage_ft_sensor";

        parameters.calibration_sample_count = 100;
        parameters.filter_alpha = 0.2;

        parameters.max_absolute_raw_wrench = {
            50.0, 50.0, 50.0,
            5.0, 5.0, 5.0
        };

        return parameters;
    }

    void wrench_callback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr message)
    {
        // 1. 将 ROS 消息转换为与 ROS 无关的 WrenchSample。
        WrenchSample sample;

        sample.stamp_nanoseconds =
            rclcpp::Time(message->header.stamp).nanoseconds();

        sample.frame_id = message->header.frame_id;

        sample.values = {
            message->wrench.force.x,
            message->wrench.force.y,
            message->wrench.force.z,
            message->wrench.torque.x,
            message->wrench.torque.y,
            message->wrench.torque.z
        };

        // 2. 前 100 个有效样本用于计算固定姿态基线。
        if (!processor_.calibrated())
        {
            const auto result =
                processor_.add_calibration_sample(sample);

            if (!result.valid)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "基线样本无效: %s",
                    result.message.c_str());
                return;
            }

            const std::size_t received =
                processor_.calibration_samples_received();

            if (processor_.calibrated())
            {
                const auto & baseline = processor_.baseline();

                RCLCPP_INFO(
                    get_logger(),
                    "基线标定完成: "
                    "Fx=%.6f, Fy=%.6f, Fz=%.6f, "
                    "Tx=%.6f, Ty=%.6f, Tz=%.6f",
                    baseline[0],
                    baseline[1],
                    baseline[2],
                    baseline[3],
                    baseline[4],
                    baseline[5]);
            }
            else if (received % 20 == 0)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "正在标定基线: %zu / 100",
                    received);
            }

            // 标定样本不再进入正常处理。
            return;
        }

        // 3. 标定完成后，执行基线扣除、限幅和低通滤波。
        const auto result = processor_.process(sample);

        if (!result.valid)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "六维力处理失败: error=%d, message=%s",
                static_cast<int>(result.error),
                result.message.c_str());
            return;
        }

        // 100 Hz 数据不应每帧都打印，否则日志会影响实时性。
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "Fz: raw=%.6f N, compensated=%.6f N, filtered=%.6f N",
            result.raw[2],
            result.compensated[2],
            result.filtered[2]);
    }
    massage_motion::WrenchProcessor processor_;

    rclcpp::Subscription<
        geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WrenchProcessingDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}