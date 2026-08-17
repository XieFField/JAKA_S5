#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "jaka_msgs/msg/robot_msg.hpp"
#include "jaka_msgs/srv/set_torque_sensor_soft_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "massage_jaka/jaka_tool_drive_client.hpp"
#include "massage_jaka/tool_response_analysis.hpp"
#include "massage_motion/wrench_subscriber.hpp"

namespace
{

std::atomic<bool> interruption_requested{false};

void handle_signal(int)
{
    interruption_requested.store(true);
}

bool finite_positive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

std::string default_csv_path()
{
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return "/tmp/massage_tool_drive_" +
        std::to_string(milliseconds) + ".csv";
}

std::array<double, 3> response_axis(
    const geometry_msgs::msg::TransformStamped & transform,
    std::size_t axis,
    std::int32_t frame)
{
    std::array<double, 3> result{};
    if (frame == 1)
    {
        result[axis] = 1.0;
        return result;
    }

    const auto & q = transform.transform.rotation;
    const std::array<std::array<double, 3>, 3> rotation{{
        {{1.0 - 2.0 * (q.y * q.y + q.z * q.z),
          2.0 * (q.x * q.y - q.z * q.w),
          2.0 * (q.x * q.z + q.y * q.w)}},
        {{2.0 * (q.x * q.y + q.z * q.w),
          1.0 - 2.0 * (q.x * q.x + q.z * q.z),
          2.0 * (q.y * q.z - q.x * q.w)}},
        {{2.0 * (q.x * q.z - q.y * q.w),
          2.0 * (q.y * q.z + q.x * q.w),
          1.0 - 2.0 * (q.x * q.x + q.y * q.y)}}}};
    for (std::size_t row = 0; row < 3U; ++row)
    {
        result[row] = rotation[row][axis];
    }
    return result;
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(
        argc, argv, rclcpp::InitOptions(),
        rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    auto node = std::make_shared<rclcpp::Node>(
        "real_tool_drive_smoke_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    int exit_code = 1;
    bool tool_drive_may_be_active = false;
    std::shared_ptr<massage_jaka::JakaToolDriveClient> tool_drive;
    std::ofstream csv;
    std::string csv_path;
    try
    {
        bool activate = false;
        bool parameters_confirmed = false;
        std::int64_t axis = 2;
        std::int64_t frame = 0;
        double rebound = 0.0;
        double rigidity = 0.0;
        bool preserve_existing_axis_parameters = true;
        bool preserve_existing_sensitivity = true;
        bool preserve_existing_warning_range = true;
        std::int64_t sensitivity_level = 1;
        std::int64_t warning_range = 1;
        double baseline_duration = 2.0;
        double settle_duration = 1.0;
        double load_duration = 4.0;
        double recovery_duration = 2.0;
        double readiness_timeout = 5.0;
        double feedback_timeout = 0.5;
        double state_timeout = 0.5;
        double log_period = 0.5;
        double minimum_force_delta = 0.5;
        double minimum_axis_displacement = 0.0002;
        double maximum_force = 5.0;
        double maximum_torque = 1.0;
        double maximum_linear_displacement = 0.002;
        double maximum_transverse_displacement = 0.0005;
        double maximum_joint_displacement = 0.005;
        std::string wrench_frame{"Link_06"};
        std::string base_frame{"world"};
        std::string tool_frame{"massage_tool_tip"};
        node->get_parameter_or("activate", activate, false);
        node->get_parameter_or("parameters_confirmed", parameters_confirmed, false);
        node->get_parameter_or("axis", axis, std::int64_t{2});
        node->get_parameter_or("frame", frame, std::int64_t{0});
        node->get_parameter_or("rebound", rebound, 0.0);
        node->get_parameter_or("rigidity", rigidity, 0.0);
        node->get_parameter_or(
            "preserve_existing_axis_parameters",
            preserve_existing_axis_parameters, true);
        node->get_parameter_or(
            "preserve_existing_sensitivity",
            preserve_existing_sensitivity, true);
        node->get_parameter_or(
            "preserve_existing_warning_range",
            preserve_existing_warning_range, true);
        node->get_parameter_or(
            "sensitivity_level", sensitivity_level, std::int64_t{1});
        node->get_parameter_or(
            "warning_range", warning_range, std::int64_t{1});
        node->get_parameter_or("baseline_duration", baseline_duration, 2.0);
        node->get_parameter_or("settle_duration", settle_duration, 1.0);
        node->get_parameter_or("load_duration", load_duration, 4.0);
        node->get_parameter_or("recovery_duration", recovery_duration, 2.0);
        node->get_parameter_or("readiness_timeout", readiness_timeout, 5.0);
        node->get_parameter_or("feedback_timeout", feedback_timeout, 0.5);
        node->get_parameter_or("state_timeout", state_timeout, 0.5);
        node->get_parameter_or("log_period", log_period, 0.5);
        node->get_parameter_or("minimum_force_delta", minimum_force_delta, 0.5);
        node->get_parameter_or(
            "minimum_axis_displacement", minimum_axis_displacement, 0.0002);
        node->get_parameter_or("maximum_force", maximum_force, 5.0);
        node->get_parameter_or("maximum_torque", maximum_torque, 1.0);
        node->get_parameter_or(
            "maximum_linear_displacement", maximum_linear_displacement, 0.002);
        node->get_parameter_or(
            "maximum_transverse_displacement",
            maximum_transverse_displacement, 0.0005);
        node->get_parameter_or(
            "maximum_joint_displacement", maximum_joint_displacement, 0.005);
        node->get_parameter_or("wrench_frame", wrench_frame, std::string{"Link_06"});
        node->get_parameter_or("base_frame", base_frame, std::string{"world"});
        node->get_parameter_or(
            "tool_frame", tool_frame, std::string{"massage_tool_tip"});
        node->get_parameter_or("csv_path", csv_path, std::string{});

        if (axis < 0 || axis >= 3 || frame != 0 ||
            sensitivity_level < 0 || sensitivity_level > 5 ||
            warning_range < 1 || warning_range > 5 ||
            !std::isfinite(rebound) || rebound < 0.0 ||
            !std::isfinite(rigidity) || rigidity < 0.0 ||
            !finite_positive(baseline_duration) ||
            !finite_positive(settle_duration) ||
            !finite_positive(load_duration) ||
            !finite_positive(recovery_duration) ||
            !finite_positive(readiness_timeout) ||
            !finite_positive(feedback_timeout) ||
            !finite_positive(state_timeout) || !finite_positive(log_period) ||
            !finite_positive(minimum_force_delta) ||
            !finite_positive(minimum_axis_displacement) ||
            !finite_positive(maximum_force) ||
            !finite_positive(maximum_torque) ||
            !finite_positive(maximum_linear_displacement) ||
            !finite_positive(maximum_transverse_displacement) ||
            !finite_positive(maximum_joint_displacement) ||
            minimum_axis_displacement >= maximum_linear_displacement ||
            wrench_frame.empty() || base_frame.empty() || tool_frame.empty())
        {
            throw std::invalid_argument("tool-drive 冒烟参数无效");
        }
        if (activate && !parameters_confirmed)
        {
            throw std::invalid_argument(
                "activate=true 时必须显式设置 parameters_confirmed=true");
        }

        if (csv_path.empty())
        {
            csv_path = default_csv_path();
        }
        csv.open(csv_path, std::ios::out | std::ios::trunc);
        if (!csv)
        {
            throw std::runtime_error("无法创建 tool-drive CSV: " + csv_path);
        }
        csv << "phase,time_s,fx_n,fy_n,fz_n,tx_nm,ty_nm,tz_nm,"
               "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,"
               "tcp_x_m,tcp_y_m,tcp_z_m,axis_displacement_m,"
               "transverse_displacement_m\n";
        csv << std::setprecision(17);

        massage_motion::WrenchSubscriberConfig wrench_config;
        wrench_config.topic_name = "/jaka_driver/wrench";
        wrench_config.expected_frame_id = wrench_frame;
        wrench_config.stale_timeout = feedback_timeout;
        auto wrench_subscriber =
            std::make_shared<massage_motion::WrenchSubscriber>(
                node, wrench_config);

        std::mutex state_mutex;
        std::vector<double> joint_positions;
        std::chrono::steady_clock::time_point joint_time{};
        jaka_msgs::msg::RobotMsg robot_state;
        std::chrono::steady_clock::time_point robot_time{};
        bool has_robot_state = false;
        auto joint_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [&](const sensor_msgs::msg::JointState::SharedPtr message)
            {
                if (message->position.size() == 6U)
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    joint_positions = message->position;
                    joint_time = std::chrono::steady_clock::now();
                }
            });
        auto robot_subscription = node->create_subscription<jaka_msgs::msg::RobotMsg>(
            "/jaka_driver/robot_states", rclcpp::SensorDataQoS(),
            [&](const jaka_msgs::msg::RobotMsg::SharedPtr message)
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                robot_state = *message;
                robot_time = std::chrono::steady_clock::now();
                has_robot_state = true;
            });

        tf2_ros::Buffer tf_buffer(node->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);
        bool readiness_passed = false;
        const auto readiness_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(readiness_timeout);
        while (!interruption_requested.load() &&
            std::chrono::steady_clock::now() < readiness_deadline)
        {
            const auto wrench = wrench_subscriber->latest();
            bool local_ready = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                const auto now = std::chrono::steady_clock::now();
                local_ready = joint_positions.size() == 6U && has_robot_state &&
                    std::chrono::duration<double>(now - joint_time).count() <=
                    state_timeout &&
                    std::chrono::duration<double>(now - robot_time).count() <=
                    state_timeout && robot_state.motion_state == 0 &&
                    robot_state.power_state == 1 && robot_state.servo_state == 1 &&
                    robot_state.collision_state == 0;
            }
            if (wrench.received && !wrench.stale && local_ready &&
                tf_buffer.canTransform(base_frame, tool_frame, tf2::TimePointZero))
            {
                readiness_passed = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        const auto wrench_ready = wrench_subscriber->latest();
        std::vector<double> initial_joints;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            initial_joints = joint_positions;
        }
        if (interruption_requested.load() || !readiness_passed ||
            wrench_ready.stale ||
            initial_joints.size() != 6U ||
            !tf_buffer.canTransform(base_frame, tool_frame, tf2::TimePointZero))
        {
            throw std::runtime_error("tool-drive 真机就绪检查失败");
        }
        const auto initial_transform = tf_buffer.lookupTransform(
            base_frame, tool_frame, tf2::TimePointZero);
        const std::array<double, 3> initial_translation{
            initial_transform.transform.translation.x,
            initial_transform.transform.translation.y,
            initial_transform.transform.translation.z};
        const auto axis_world = response_axis(
            initial_transform, static_cast<std::size_t>(axis),
            static_cast<std::int32_t>(frame));

        RCLCPP_INFO(
            node->get_logger(),
            "就绪检查通过: axis=%ld, frame=%s, 响应轴(world)=[%.5f %.5f %.5f]",
            static_cast<long>(axis), frame == 0 ? "tool" : "world",
            axis_world[0], axis_world[1], axis_world[2]);
        RCLCPP_INFO(
            node->get_logger(),
            "阶段 1/4 基线采集 %.1f 秒：不要接触机械臂",
            baseline_duration);

        std::array<double, 6> baseline_sum{};
        std::size_t baseline_count = 0;
        std::int64_t last_baseline_stamp =
            std::numeric_limits<std::int64_t>::min();
        const auto baseline_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(baseline_duration);
        while (!interruption_requested.load() &&
            std::chrono::steady_clock::now() < baseline_deadline)
        {
            const auto wrench = wrench_subscriber->latest();
            if (!wrench.stale &&
                wrench.sample.stamp_nanoseconds != last_baseline_stamp)
            {
                last_baseline_stamp = wrench.sample.stamp_nanoseconds;
                for (std::size_t index = 0; index < 6U; ++index)
                {
                    baseline_sum[index] += wrench.sample.values[index];
                }
                ++baseline_count;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (baseline_count == 0U)
        {
            throw std::runtime_error("FT 基线没有有效样本");
        }
        std::array<double, 6> baseline_wrench{};
        for (std::size_t index = 0; index < 6U; ++index)
        {
            baseline_wrench[index] =
                baseline_sum[index] / static_cast<double>(baseline_count);
        }
        RCLCPP_INFO(
            node->get_logger(),
            "基线完成: samples=%zu, selected_axis=%.4f",
            baseline_count, baseline_wrench[static_cast<std::size_t>(axis)]);

        massage_jaka::JakaToolDriveConfig tool_config;
        tool_config.axis = static_cast<std::int32_t>(axis);
        tool_config.frame = static_cast<std::int32_t>(frame);
        tool_config.rebound = rebound;
        tool_config.rigidity = rigidity;
        tool_config.preserve_existing_axis_parameters =
            preserve_existing_axis_parameters;
        tool_config.preserve_existing_sensitivity =
            preserve_existing_sensitivity;
        tool_config.preserve_existing_warning_range =
            preserve_existing_warning_range;
        tool_config.sensitivity_level =
            static_cast<std::int32_t>(sensitivity_level);
        tool_config.warning_range = static_cast<std::int32_t>(warning_range);
        tool_drive = std::make_shared<massage_jaka::JakaToolDriveClient>(
            node, tool_config);

        auto soft_limit_client =
            node->create_client<jaka_msgs::srv::SetTorqueSensorSoftLimit>(
                "/jaka_driver/set_ft_soft_limit");
        if (!soft_limit_client->wait_for_service(std::chrono::seconds(3)))
        {
            throw std::runtime_error("set_ft_soft_limit 服务不可用");
        }
        auto limit_request = std::make_shared<
            jaka_msgs::srv::SetTorqueSensorSoftLimit::Request>();
        limit_request->limits = {
            maximum_force, maximum_force, maximum_force,
            maximum_torque, maximum_torque, maximum_torque};
        auto limit_future = soft_limit_client->async_send_request(limit_request);
        if (limit_future.wait_for(std::chrono::seconds(3)) !=
            std::future_status::ready || !limit_future.get()->success)
        {
            throw std::runtime_error("FT 软限幅写入失败");
        }

        std::string operation_message;
        if (!tool_drive->configure(operation_message))
        {
            throw std::runtime_error("tool drive 配置失败: " + operation_message);
        }
        RCLCPP_INFO(node->get_logger(), "%s", operation_message.c_str());
        const auto configured_state = tool_drive->state();
        if (!configured_state.success)
        {
            throw std::runtime_error(
                "tool drive 配置后状态读取失败: " +
                configured_state.message);
        }
        RCLCPP_INFO(
            node->get_logger(),
            "tool drive 读回: enabled=%s, owner=%s, frame=%d, axis=%ld, "
            "option=%d, rebound=%.6f, rigidity=%.6f, sensitivity=%d, "
            "warning_range=%d, warning_state=%d",
            configured_state.enabled ? "true" : "false",
            configured_state.control_owner.c_str(), configured_state.frame,
            static_cast<long>(axis),
            configured_state.axis_options[static_cast<std::size_t>(axis)],
            configured_state.rebound[static_cast<std::size_t>(axis)],
            configured_state.rigidity[static_cast<std::size_t>(axis)],
            configured_state.sensitivity_level,
            configured_state.warning_range,
            configured_state.warning_state);
        if (!activate)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "只读/配置验证完成；activate=false，未启用 tool drive");
            exit_code = 0;
        }
        else
        {
            if (configured_state.sensitivity_level == 0)
            {
                throw std::runtime_error(
                    "fusion-drive sensitivity=0，tool-drive 功能处于关闭状态；"
                    "请显式设置 preserve_existing_sensitivity:=false 和 "
                    "sensitivity_level:=1 后再启用");
            }
            if (!tool_drive->start(operation_message))
            {
                throw std::runtime_error("tool drive 启用失败: " + operation_message);
            }
            tool_drive_may_be_active = true;
            RCLCPP_INFO(node->get_logger(), "%s", operation_message.c_str());

            std::vector<massage_jaka::ToolResponseSample> response_samples;
            const auto experiment_started = std::chrono::steady_clock::now();
            auto next_log = experiment_started;
            std::int64_t last_response_stamp =
                std::numeric_limits<std::int64_t>::min();
            std::string guard_error;

            const auto collect_phase = [&](
                const char * phase, double duration, const char * instruction)
                {
                    RCLCPP_INFO(
                        node->get_logger(), "%s %.1f 秒：%s",
                        phase, duration, instruction);
                    const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(duration);
                    while (!interruption_requested.load() &&
                        guard_error.empty() &&
                        std::chrono::steady_clock::now() < deadline)
                    {
                        const auto wrench = wrench_subscriber->latest();
                        if (wrench.stale)
                        {
                            guard_error = "FT 反馈过期";
                            break;
                        }
                        if (wrench.sample.stamp_nanoseconds ==
                            last_response_stamp)
                        {
                            std::this_thread::sleep_for(
                                std::chrono::milliseconds(20));
                            continue;
                        }
                        last_response_stamp = wrench.sample.stamp_nanoseconds;
                        std::vector<double> current_joints;
                        bool current_robot_ready = false;
                        {
                            std::lock_guard<std::mutex> lock(state_mutex);
                            current_joints = joint_positions;
                            const auto now = std::chrono::steady_clock::now();
                            current_robot_ready = has_robot_state &&
                                std::chrono::duration<double>(
                                    now - joint_time).count() <= state_timeout &&
                                std::chrono::duration<double>(
                                    now - robot_time).count() <= state_timeout &&
                                robot_state.power_state == 1 &&
                                robot_state.servo_state == 1 &&
                                robot_state.collision_state == 0;
                        }
                        if (!current_robot_ready || current_joints.size() != 6U)
                        {
                            guard_error = "机器人或关节状态失效";
                            break;
                        }
                        geometry_msgs::msg::TransformStamped transform;
                        try
                        {
                            transform = tf_buffer.lookupTransform(
                                base_frame, tool_frame, tf2::TimePointZero);
                        }
                        catch (const tf2::TransformException & exception)
                        {
                            guard_error = std::string("TF 读取失败: ") +
                                exception.what();
                            break;
                        }
                        massage_jaka::ToolResponseSample sample;
                        sample.time = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() -
                            experiment_started).count();
                        sample.wrench = wrench.sample.values;
                        sample.joint_positions = current_joints;
                        sample.translation = {
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z};
                        response_samples.push_back(sample);

                        const std::array<double, 3> displacement{
                            sample.translation[0] - initial_translation[0],
                            sample.translation[1] - initial_translation[1],
                            sample.translation[2] - initial_translation[2]};
                        const double projection =
                            displacement[0] * axis_world[0] +
                            displacement[1] * axis_world[1] +
                            displacement[2] * axis_world[2];
                        const double linear = std::sqrt(
                            displacement[0] * displacement[0] +
                            displacement[1] * displacement[1] +
                            displacement[2] * displacement[2]);
                        const double transverse = std::sqrt(std::max(
                            0.0, linear * linear - projection * projection));
                        double max_joint = 0.0;
                        if (current_joints.size() == initial_joints.size())
                        {
                            for (std::size_t index = 0;
                                index < current_joints.size(); ++index)
                            {
                                max_joint = std::max(
                                    max_joint,
                                    std::abs(
                                        current_joints[index] -
                                        initial_joints[index]));
                            }
                        }
                        const double axis_force =
                            sample.wrench[static_cast<std::size_t>(axis)] -
                            baseline_wrench[static_cast<std::size_t>(axis)];
                        csv << phase << ',' << sample.time;
                        for (const double value : sample.wrench)
                        {
                            csv << ',' << value;
                        }
                        for (const double value : sample.joint_positions)
                        {
                            csv << ',' << value;
                        }
                        csv << ',' << sample.translation[0]
                            << ',' << sample.translation[1]
                            << ',' << sample.translation[2]
                            << ',' << projection << ',' << transverse << '\n';
                        csv.flush();

                        if (std::chrono::steady_clock::now() >= next_log)
                        {
                            const auto drive_state = tool_drive->state();
                            if (!drive_state.success || !drive_state.enabled ||
                                drive_state.control_owner != "tool_drive")
                            {
                                guard_error =
                                    "tool drive 启用状态或控制权失效: " +
                                    drive_state.message;
                            }
                            else if (drive_state.warning_state != 0)
                            {
                                guard_error =
                                    "tool drive 报告运动告警，state=" +
                                    std::to_string(drive_state.warning_state);
                            }
                            RCLCPP_INFO(
                                node->get_logger(),
                                "%s: dF=%.3f, axis=%.3f mm, transverse=%.3f mm, "
                                "linear=%.3f mm, max_joint=%.6f rad",
                                phase, axis_force, projection * 1000.0,
                                transverse * 1000.0, linear * 1000.0, max_joint);
                            next_log = std::chrono::steady_clock::now() +
                                std::chrono::duration_cast<
                                    std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(log_period));
                        }

                        const bool wrench_over_limit =
                            std::abs(sample.wrench[0]) > maximum_force ||
                            std::abs(sample.wrench[1]) > maximum_force ||
                            std::abs(sample.wrench[2]) > maximum_force ||
                            std::abs(sample.wrench[3]) > maximum_torque ||
                            std::abs(sample.wrench[4]) > maximum_torque ||
                            std::abs(sample.wrench[5]) > maximum_torque;
                        if (wrench_over_limit)
                        {
                            guard_error = "六维力超过软件限幅";
                        }
                        else if (linear > maximum_linear_displacement)
                        {
                            guard_error = "TCP 总位移超过上限";
                        }
                        else if (transverse > maximum_transverse_displacement)
                        {
                            guard_error = "TCP 横向位移超过上限";
                        }
                        else if (max_joint > maximum_joint_displacement)
                        {
                            guard_error = "关节位移超过上限";
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    }
                };

            collect_phase(
                "阶段 2/4 稳定", settle_duration,
                "tool drive 已启用，暂时不要接触");
            if (guard_error.empty())
            {
                collect_phase(
                    "阶段 3/4 施力", load_duration,
                    "现在沿选定轴缓慢施力，使轴向力变化保持在 1~4 N");
            }
            if (guard_error.empty())
            {
                collect_phase(
                    "阶段 4/4 松手", recovery_duration,
                    "完全松手，不再接触，等待程序自动判定");
            }

            if (!tool_drive->stop(operation_message))
            {
                throw std::runtime_error("tool drive 关闭失败: " + operation_message);
            }
            tool_drive_may_be_active = false;
            RCLCPP_INFO(node->get_logger(), "%s", operation_message.c_str());

            if (interruption_requested.load())
            {
                RCLCPP_WARN(
                    node->get_logger(),
                    "收到中断，tool drive 已关闭并完成控制权读回");
                exit_code = 130;
            }

            massage_jaka::ToolResponseLimits limits;
            limits.axis = static_cast<std::size_t>(axis);
            limits.minimum_force_delta = minimum_force_delta;
            limits.minimum_axis_displacement = minimum_axis_displacement;
            limits.maximum_linear_displacement = maximum_linear_displacement;
            limits.maximum_transverse_displacement =
                maximum_transverse_displacement;
            limits.maximum_joint_displacement = maximum_joint_displacement;
            const auto report = massage_jaka::analyze_tool_response(
                response_samples, baseline_wrench, initial_joints,
                initial_translation, axis_world, limits);
            RCLCPP_INFO(
                node->get_logger(),
                "量化结果: peak_dF=%.4f N, directed_axis=%.4f mm, "
                "max_axis=%.4f mm, transverse=%.4f mm, linear=%.4f mm, "
                "max_joint=%.7f rad",
                report.peak_force_delta,
                report.directed_axis_displacement * 1000.0,
                report.maximum_axis_displacement * 1000.0,
                report.maximum_transverse_displacement * 1000.0,
                report.maximum_linear_displacement * 1000.0,
                report.maximum_joint_displacement);
            if (interruption_requested.load())
            {
                exit_code = 130;
            }
            else if (!guard_error.empty())
            {
                RCLCPP_ERROR(
                    node->get_logger(), "TOOL DRIVE TEST: FAIL: %s",
                    guard_error.c_str());
                exit_code = 4;
            }
            else if (!report.passed)
            {
                RCLCPP_ERROR(
                    node->get_logger(), "TOOL DRIVE TEST: FAIL: %s",
                    report.message.c_str());
                exit_code = 4;
            }
            else
            {
                RCLCPP_INFO(
                    node->get_logger(), "TOOL DRIVE TEST: PASS: %s",
                    report.message.c_str());
                exit_code = 0;
            }
        }
        (void)joint_subscription;
        (void)robot_subscription;
        (void)tf_listener;
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(
            node->get_logger(), "真机 tool-drive 冒烟异常: %s",
            exception.what());
        exit_code = interruption_requested.load() ? 130 : 2;
    }

    if (tool_drive_may_be_active && tool_drive)
    {
        std::string message;
        if (!tool_drive->stop(message))
        {
            RCLCPP_ERROR(
                node->get_logger(), "退出清理未能关闭 tool drive: %s",
                message.c_str());
            exit_code = 5;
        }
    }
    if (csv)
    {
        csv.flush();
        RCLCPP_INFO(
            node->get_logger(), "tool-drive 遥测 CSV: %s", csv_path.c_str());
    }
    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
