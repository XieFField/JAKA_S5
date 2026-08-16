#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "massage_motion/wrench_observation.hpp"

namespace
{

using massage_motion::WrenchObservationPhase;
using massage_motion::WrenchObservationReport;
using massage_motion::WrenchObservationSample;
using massage_motion::WrenchPhaseStatistics;
using massage_motion::WrenchVector;

WrenchVector wrench_values(const geometry_msgs::msg::Wrench & wrench)
{
    return {
        wrench.force.x,
        wrench.force.y,
        wrench.force.z,
        wrench.torque.x,
        wrench.torque.y,
        wrench.torque.z};
}

bool finite_values(const WrenchVector & values)
{
    return std::all_of(
        values.begin(), values.end(),
        [](double value) {return std::isfinite(value);});
}

std::string default_csv_path()
{
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return "/tmp/massage_ft_observation_" +
        std::to_string(milliseconds) + ".csv";
}

void write_csv(
    const std::string & path,
    const std::vector<WrenchObservationSample> & samples)
{
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output)
    {
        throw std::runtime_error("无法创建 FT CSV 文件: " + path);
    }
    output << "phase,receive_time_s,stamp_ns,frame_id,fx_n,fy_n,fz_n,"
              "tx_nm,ty_nm,tz_nm\n";
    output << std::setprecision(17);
    for (const auto & sample : samples)
    {
        output << massage_motion::to_string(sample.phase) << ','
               << sample.receive_time << ','
               << sample.stamp_nanoseconds << ','
               << std::quoted(sample.frame_id);
        for (const double value : sample.values)
        {
            output << ',' << value;
        }
        output << '\n';
    }
    if (!output)
    {
        throw std::runtime_error("写入 FT CSV 文件失败: " + path);
    }
}

void log_phase(
    const rclcpp::Logger & logger,
    const char * name,
    const WrenchPhaseStatistics & statistics)
{
    RCLCPP_INFO(
        logger,
        "%s: samples=%zu, rate=%.3f Hz, max_gap=%.4f s, "
        "mean=[%.4f %.4f %.4f N; %.4f %.4f %.4f N*m], "
        "std=[%.4f %.4f %.4f; %.4f %.4f %.4f]",
        name, statistics.sample_count, statistics.sample_rate,
        statistics.maximum_gap,
        statistics.mean[0], statistics.mean[1], statistics.mean[2],
        statistics.mean[3], statistics.mean[4], statistics.mean[5],
        statistics.standard_deviation[0],
        statistics.standard_deviation[1],
        statistics.standard_deviation[2],
        statistics.standard_deviation[3],
        statistics.standard_deviation[4],
        statistics.standard_deviation[5]);
}

double maximum_force_delta(const WrenchObservationReport & report)
{
    return std::max({
        report.maximum_load_delta[0],
        report.maximum_load_delta[1],
        report.maximum_load_delta[2]});
}

double maximum_torque_delta(const WrenchObservationReport & report)
{
    return std::max({
        report.maximum_load_delta[3],
        report.maximum_load_delta[4],
        report.maximum_load_delta[5]});
}

double maximum_force_recovery_offset(const WrenchObservationReport & report)
{
    return std::max({
        report.recovery_offset[0],
        report.recovery_offset[1],
        report.recovery_offset[2]});
}

double maximum_torque_recovery_offset(const WrenchObservationReport & report)
{
    return std::max({
        report.recovery_offset[3],
        report.recovery_offset[4],
        report.recovery_offset[5]});
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "real_ft_observation_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    std::string wrench_topic{"/jaka_driver/wrench"};
    std::string expected_frame{"Link_06"};
    std::string csv_path;
    std::string required_response{"none"};
    double start_timeout = 5.0;
    double baseline_duration = 5.0;
    double load_duration = 10.0;
    double recovery_duration = 5.0;
    double minimum_sample_rate = 5.0;
    double maximum_sample_gap = 0.5;
    double minimum_force_delta = 0.5;
    double minimum_torque_delta = 0.05;
    double maximum_recovery_force_offset = 0.0;
    double maximum_recovery_torque_offset = 0.0;
    node->get_parameter_or(
        "wrench_topic", wrench_topic, std::string{"/jaka_driver/wrench"});
    node->get_parameter_or(
        "expected_frame", expected_frame, std::string{"Link_06"});
    node->get_parameter_or("csv_path", csv_path, std::string{});
    node->get_parameter_or(
        "required_response", required_response, std::string{"none"});
    node->get_parameter_or("start_timeout", start_timeout, 5.0);
    node->get_parameter_or("baseline_duration", baseline_duration, 5.0);
    node->get_parameter_or("load_duration", load_duration, 10.0);
    node->get_parameter_or("recovery_duration", recovery_duration, 5.0);
    node->get_parameter_or("minimum_sample_rate", minimum_sample_rate, 5.0);
    node->get_parameter_or("maximum_sample_gap", maximum_sample_gap, 0.5);
    node->get_parameter_or("minimum_force_delta", minimum_force_delta, 0.5);
    node->get_parameter_or(
        "minimum_torque_delta", minimum_torque_delta, 0.05);
    node->get_parameter_or(
        "maximum_recovery_force_offset",
        maximum_recovery_force_offset, 0.0);
    node->get_parameter_or(
        "maximum_recovery_torque_offset",
        maximum_recovery_torque_offset, 0.0);

    const std::array<std::string, 5> response_modes{
        "none", "force", "torque", "either", "both"};
    const bool response_mode_valid = std::find(
        response_modes.begin(), response_modes.end(), required_response) !=
        response_modes.end();
    const bool parameters_valid = !wrench_topic.empty() &&
        response_mode_valid &&
        std::isfinite(start_timeout) && start_timeout > 0.0 &&
        std::isfinite(baseline_duration) && baseline_duration > 0.0 &&
        std::isfinite(load_duration) && load_duration > 0.0 &&
        std::isfinite(recovery_duration) && recovery_duration > 0.0 &&
        std::isfinite(minimum_sample_rate) && minimum_sample_rate >= 0.0 &&
        std::isfinite(maximum_sample_gap) && maximum_sample_gap >= 0.0 &&
        std::isfinite(minimum_force_delta) && minimum_force_delta >= 0.0 &&
        std::isfinite(minimum_torque_delta) && minimum_torque_delta >= 0.0 &&
        std::isfinite(maximum_recovery_force_offset) &&
        maximum_recovery_force_offset >= 0.0 &&
        std::isfinite(maximum_recovery_torque_offset) &&
        maximum_recovery_torque_offset >= 0.0;
    if (!parameters_valid)
    {
        RCLCPP_ERROR(node->get_logger(), "FT 观测参数无效");
        rclcpp::shutdown();
        return 2;
    }
    if ((required_response == "force" || required_response == "either" ||
        required_response == "both") &&
        minimum_force_delta <= 0.0)
    {
        RCLCPP_ERROR(node->get_logger(), "力响应门禁要求正的 minimum_force_delta");
        rclcpp::shutdown();
        return 2;
    }
    if ((required_response == "torque" || required_response == "either" ||
        required_response == "both") &&
        minimum_torque_delta <= 0.0)
    {
        RCLCPP_ERROR(node->get_logger(), "力矩响应门禁要求正的 minimum_torque_delta");
        rclcpp::shutdown();
        return 2;
    }
    if (csv_path.empty())
    {
        csv_path = default_csv_path();
    }

    std::mutex sample_mutex;
    std::condition_variable sample_condition;
    std::vector<WrenchObservationSample> samples;
    std::uint64_t accepted_samples = 0;
    std::uint64_t rejected_samples = 0;
    bool first_sample_received = false;
    bool collection_active = false;
    WrenchObservationPhase current_phase = WrenchObservationPhase::kBaseline;
    const auto collector_started_at = std::chrono::steady_clock::now();

    auto subscription =
        node->create_subscription<geometry_msgs::msg::WrenchStamped>(
            wrench_topic,
            rclcpp::SensorDataQoS(),
            [&](geometry_msgs::msg::WrenchStamped::SharedPtr message)
            {
                const auto received_at = std::chrono::steady_clock::now();
                const auto values = wrench_values(message->wrench);
                const bool valid = !message->header.frame_id.empty() &&
                    (expected_frame.empty() ||
                    message->header.frame_id == expected_frame) &&
                    finite_values(values);

                std::lock_guard<std::mutex> lock(sample_mutex);
                if (!valid)
                {
                    ++rejected_samples;
                    sample_condition.notify_all();
                    return;
                }

                ++accepted_samples;
                first_sample_received = true;
                if (collection_active)
                {
                    WrenchObservationSample sample;
                    sample.phase = current_phase;
                    sample.stamp_nanoseconds =
                        rclcpp::Time(message->header.stamp).nanoseconds();
                    sample.receive_time = std::chrono::duration<double>(
                        received_at - collector_started_at).count();
                    sample.frame_id = message->header.frame_id;
                    sample.values = values;
                    samples.push_back(std::move(sample));
                }
                sample_condition.notify_all();
            });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    int exit_code = 1;
    try
    {
        {
            std::unique_lock<std::mutex> lock(sample_mutex);
            if (!sample_condition.wait_for(
                    lock,
                    std::chrono::duration<double>(start_timeout),
                    [&first_sample_received]() {return first_sample_received;}))
            {
                throw std::runtime_error("等待有效 FT 数据超时");
            }
        }

        const auto collect_phase = [&](WrenchObservationPhase phase,
                double duration, const char * instruction)
            {
                {
                    std::lock_guard<std::mutex> lock(sample_mutex);
                    current_phase = phase;
                    collection_active = true;
                }
                RCLCPP_INFO(
                    node->get_logger(), "%s，持续 %.1f 秒", instruction, duration);
                const auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration<double>(duration);
                while (rclcpp::ok() &&
                    std::chrono::steady_clock::now() < deadline)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                {
                    std::lock_guard<std::mutex> lock(sample_mutex);
                    collection_active = false;
                }
                if (!rclcpp::ok())
                {
                    throw std::runtime_error("FT 观测被中断");
                }
            };

        collect_phase(
            WrenchObservationPhase::kBaseline,
            baseline_duration,
            "baseline 阶段：保持传感器无外部加载");
        collect_phase(
            WrenchObservationPhase::kLoad,
            load_duration,
            "load 阶段：在此时间窗内施加待观察的力或力矩");
        collect_phase(
            WrenchObservationPhase::kRecovery,
            recovery_duration,
            "recovery 阶段：卸载并保持静止");

        std::vector<WrenchObservationSample> collected_samples;
        std::uint64_t final_accepted_samples;
        std::uint64_t final_rejected_samples;
        {
            std::lock_guard<std::mutex> lock(sample_mutex);
            collected_samples = samples;
            final_accepted_samples = accepted_samples;
            final_rejected_samples = rejected_samples;
        }
        write_csv(csv_path, collected_samples);

        const auto report = massage_motion::analyze_wrench_observation(
            collected_samples,
            expected_frame,
            minimum_sample_rate,
            maximum_sample_gap);
        if (!report.valid)
        {
            throw std::runtime_error(report.message);
        }

        log_phase(node->get_logger(), "baseline", report.baseline);
        log_phase(node->get_logger(), "load", report.load);
        log_phase(node->get_logger(), "recovery", report.recovery);
        const double force_delta = maximum_force_delta(report);
        const double torque_delta = maximum_torque_delta(report);
        const double force_recovery = maximum_force_recovery_offset(report);
        const double torque_recovery = maximum_torque_recovery_offset(report);
        RCLCPP_INFO(
            node->get_logger(),
            "加载最大变化: force=%.4f N, torque=%.4f N*m；"
            "卸载均值偏差: force=%.4f N, torque=%.4f N*m",
            force_delta, torque_delta, force_recovery, torque_recovery);

        const bool force_response = force_delta >= minimum_force_delta;
        const bool torque_response = torque_delta >= minimum_torque_delta;
        bool response_passed = true;
        if (required_response == "force")
        {
            response_passed = force_response;
        }
        else if (required_response == "torque")
        {
            response_passed = torque_response;
        }
        else if (required_response == "either")
        {
            response_passed = force_response || torque_response;
        }
        else if (required_response == "both")
        {
            response_passed = force_response && torque_response;
        }
        if (!response_passed)
        {
            throw std::runtime_error(
                "加载阶段未达到 required_response 指定的变化门限");
        }
        if (maximum_recovery_force_offset > 0.0 &&
            force_recovery > maximum_recovery_force_offset)
        {
            throw std::runtime_error("卸载后力偏差超过配置门限");
        }
        if (maximum_recovery_torque_offset > 0.0 &&
            torque_recovery > maximum_recovery_torque_offset)
        {
            throw std::runtime_error("卸载后力矩偏差超过配置门限");
        }

        RCLCPP_INFO(
            node->get_logger(),
            "FT 被动观测完成: accepted=%lu, rejected=%lu, frame=%s, csv=%s",
            static_cast<unsigned long>(final_accepted_samples),
            static_cast<unsigned long>(final_rejected_samples),
            report.frame_id.c_str(), csv_path.c_str());
        exit_code = 0;
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(node->get_logger(), "FT 被动观测失败: %s", exception.what());
        std::vector<WrenchObservationSample> partial_samples;
        {
            std::lock_guard<std::mutex> lock(sample_mutex);
            collection_active = false;
            partial_samples = samples;
        }
        try
        {
            write_csv(csv_path, partial_samples);
            RCLCPP_ERROR(
                node->get_logger(),
                "失败前的 %zu 个样本已保存: %s",
                partial_samples.size(), csv_path.c_str());
        }
        catch (const std::exception & csv_exception)
        {
            RCLCPP_ERROR(
                node->get_logger(), "失败样本无法保存: %s",
                csv_exception.what());
        }
        exit_code = 3;
    }

    (void)subscription;
    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
