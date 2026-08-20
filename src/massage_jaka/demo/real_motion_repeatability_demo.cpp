#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "jaka_msgs/msg/robot_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_motion/execution_timing.hpp"
#include "massage_motion/guarded_trajectory_executor.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/motion_repeatability.hpp"
#include "massage_motion/trajectory_execution_contract.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"

namespace
{

const std::vector<std::string> kJointNames{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};

bool robot_ready(const jaka_msgs::msg::RobotMsg & state)
{
    return state.motion_state == 0 && state.power_state == 1 &&
           state.servo_state == 1 && state.collision_state == 0;
}

bool extract_ordered_positions(
    const sensor_msgs::msg::JointState & state,
    std::vector<double> & positions,
    std::string & error)
{
    if (state.name.size() != state.position.size())
    {
        error = "关节状态名称和位置数组长度不一致";
        return false;
    }
    positions.clear();
    positions.reserve(kJointNames.size());
    for (const auto & name : kJointNames)
    {
        const auto iterator = std::find(state.name.begin(), state.name.end(), name);
        if (iterator == state.name.end())
        {
            error = "关节状态缺少 " + name;
            return false;
        }
        const auto index = static_cast<std::size_t>(
            std::distance(state.name.begin(), iterator));
        if (index >= state.position.size() || !std::isfinite(state.position[index]))
        {
            error = name + " 的反馈无效";
            return false;
        }
        positions.push_back(state.position[index]);
    }
    return true;
}

std::string make_csv_path()
{
    const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return "/tmp/massage_motion_repeatability_" +
           std::to_string(stamp) + ".csv";
}

std::string csv_text(const std::string & value)
{
    std::string escaped;
    escaped.reserve(value.size() + 2U);
    escaped.push_back('"');
    for (const char character : value)
    {
        if (character == '"')
        {
            escaped.push_back('"');
        }
        escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

bool write_trials_csv(
    const std::string & path,
    const std::vector<massage_motion::MotionRepeatabilityTrial> & trials)
{
    std::ofstream stream(path);
    if (!stream)
    {
        return false;
    }
    stream << "cycle,direction,passed,initial_position,target_position,"
              "actual_position,commanded_delta,achieved_delta,completion_ratio,"
              "endpoint_error,maximum_endpoint_error,planned_duration,"
              "execution_duration,message\n";
    stream << std::setprecision(12);
    for (const auto & trial : trials)
    {
        stream << trial.cycle << ','
               << massage_motion::to_string(trial.direction) << ','
               << trial.passed << ','
               << trial.initial_position << ','
               << trial.target_position << ','
               << trial.actual_position << ','
               << trial.commanded_delta << ','
               << trial.achieved_delta << ','
               << trial.completion_ratio << ','
               << trial.endpoint_error << ','
               << trial.maximum_endpoint_error << ','
               << trial.planned_duration << ','
               << trial.execution_duration << ','
               << csv_text(trial.message) << '\n';
    }
    return stream.good();
}

void log_direction_summary(
    const rclcpp::Logger & logger,
    const char * name,
    const massage_motion::DirectionRepeatabilityStatistics & statistics)
{
    RCLCPP_INFO(
        logger,
        "%s 汇总: passed=%zu/%zu, mean_actual=%.9f rad, "
        "range=%.9f rad, mean_error=%.9f rad, max_error=%.9f rad, "
        "mean_completion=%.2f%%, min_completion=%.2f%%",
        name, statistics.passed, statistics.samples,
        statistics.mean_actual_position, statistics.position_range,
        statistics.mean_endpoint_error, statistics.maximum_endpoint_error,
        statistics.mean_completion_ratio * 100.0,
        statistics.minimum_completion_ratio * 100.0);
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "real_motion_repeatability_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    bool execute = false;
    bool parameters_confirmed = false;
    std::string joint_name{"joint_1"};
    int cycles = 3;
    double joint_delta = 0.01;
    double maximum_joint_delta = 0.02;
    double velocity_scale = 0.02;
    double acceleration_scale = 0.02;
    double planning_timeout = 5.0;
    double execution_timeout_margin = 10.0;
    double state_timeout = 3.0;
    double readiness_timeout = 30.0;
    double feedback_timeout = 1.0;
    double endpoint_tolerance = 0.002;
    double minimum_completion_ratio = 0.90;
    double maximum_position_range = 0.001;
    double settling_duration = 0.3;
    double inter_trial_delay = 0.5;
    std::string output_csv;
    node->get_parameter_or("execute", execute, false);
    node->get_parameter_or("parameters_confirmed", parameters_confirmed, false);
    node->get_parameter_or("joint_name", joint_name, std::string{"joint_1"});
    node->get_parameter_or("cycles", cycles, 3);
    node->get_parameter_or("joint_delta", joint_delta, 0.01);
    node->get_parameter_or("maximum_joint_delta", maximum_joint_delta, 0.02);
    node->get_parameter_or("velocity_scale", velocity_scale, 0.02);
    node->get_parameter_or("acceleration_scale", acceleration_scale, 0.02);
    node->get_parameter_or("planning_timeout", planning_timeout, 5.0);
    node->get_parameter_or(
        "execution_timeout_margin", execution_timeout_margin, 10.0);
    node->get_parameter_or("state_timeout", state_timeout, 3.0);
    node->get_parameter_or("readiness_timeout", readiness_timeout, 30.0);
    node->get_parameter_or("feedback_timeout", feedback_timeout, 1.0);
    node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.002);
    node->get_parameter_or(
        "minimum_completion_ratio", minimum_completion_ratio, 0.90);
    node->get_parameter_or(
        "maximum_position_range", maximum_position_range, 0.001);
    node->get_parameter_or("settling_duration", settling_duration, 0.3);
    node->get_parameter_or("inter_trial_delay", inter_trial_delay, 0.5);
    node->get_parameter_or("output_csv", output_csv, std::string{});

    const auto selected_joint_iterator = std::find(
        kJointNames.begin(), kJointNames.end(), joint_name);
    if (execute && !parameters_confirmed)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "真机重复性执行需要 execute:=true 和 parameters_confirmed:=true");
        rclcpp::shutdown();
        return 2;
    }
    if (selected_joint_iterator == kJointNames.end() || cycles < 1 || cycles > 10 ||
        !std::isfinite(joint_delta) || joint_delta <= 0.0 ||
        !std::isfinite(maximum_joint_delta) || maximum_joint_delta <= 0.0 ||
        joint_delta > maximum_joint_delta ||
        !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
        velocity_scale > 1.0 ||
        !std::isfinite(acceleration_scale) || acceleration_scale <= 0.0 ||
        acceleration_scale > 1.0 ||
        !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
        !std::isfinite(execution_timeout_margin) || execution_timeout_margin < 0.0 ||
        !std::isfinite(state_timeout) || state_timeout <= 0.0 ||
        !std::isfinite(readiness_timeout) || readiness_timeout <= 0.0 ||
        !std::isfinite(feedback_timeout) || feedback_timeout <= 0.0 ||
        !std::isfinite(endpoint_tolerance) || endpoint_tolerance <= 0.0 ||
        endpoint_tolerance >= joint_delta ||
        !std::isfinite(minimum_completion_ratio) ||
        minimum_completion_ratio <= 0.0 || minimum_completion_ratio > 1.0 ||
        !std::isfinite(maximum_position_range) || maximum_position_range < 0.0 ||
        !std::isfinite(settling_duration) || settling_duration < 0.0 ||
        !std::isfinite(inter_trial_delay) || inter_trial_delay < 0.0)
    {
        RCLCPP_ERROR(node->get_logger(), "真机重复性测试参数无效");
        rclcpp::shutdown();
        return 2;
    }
    const auto selected_joint_index = static_cast<std::size_t>(
        std::distance(kJointNames.begin(), selected_joint_iterator));
    if (output_csv.empty())
    {
        output_csv = make_csv_path();
    }

    std::mutex state_mutex;
    std::condition_variable state_condition;
    sensor_msgs::msg::JointState latest_joint_state;
    jaka_msgs::msg::RobotMsg latest_robot_state;
    std::uint64_t joint_sequence = 0U;
    std::uint64_t robot_sequence = 0U;
    auto joint_received_at = std::chrono::steady_clock::time_point::min();
    auto robot_received_at = std::chrono::steady_clock::time_point::min();
    auto joint_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [&](sensor_msgs::msg::JointState::SharedPtr message)
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                latest_joint_state = *message;
                ++joint_sequence;
                joint_received_at = std::chrono::steady_clock::now();
            }
            state_condition.notify_all();
        });
    auto robot_subscription = node->create_subscription<jaka_msgs::msg::RobotMsg>(
        "/jaka_driver/robot_states", rclcpp::SensorDataQoS(),
        [&](jaka_msgs::msg::RobotMsg::SharedPtr message)
        {
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                latest_robot_state = *message;
                ++robot_sequence;
                robot_received_at = std::chrono::steady_clock::now();
            }
            state_condition.notify_all();
        });

    rclcpp::executors::SingleThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    std::thread spin_thread([&ros_executor]() {ros_executor.spin();});

    int exit_code = 1;
    std::vector<massage_motion::MotionRepeatabilityTrial> trials;
    try
    {
        {
            std::unique_lock<std::mutex> lock(state_mutex);
            if (!state_condition.wait_for(
                    lock, std::chrono::duration<double>(state_timeout),
                    [&]() {return joint_sequence > 0U && robot_sequence > 0U;}))
            {
                throw std::runtime_error(
                    "等待 /joint_states 或 /jaka_driver/robot_states 超时");
            }
            if (execute && !robot_ready(latest_robot_state))
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "等待机器人进入可执行状态，最长 %.1f 秒",
                    readiness_timeout);
                if (!state_condition.wait_for(
                        lock, std::chrono::duration<double>(readiness_timeout),
                        [&]() {return robot_ready(latest_robot_state);} ))
                {
                    throw std::runtime_error("等待真机可执行状态超时");
                }
            }
        }

        sensor_msgs::msg::JointState baseline_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            baseline_state = latest_joint_state;
        }
        std::vector<double> baseline_positions;
        std::string state_error;
        if (!extract_ordered_positions(
                baseline_state, baseline_positions, state_error))
        {
            throw std::runtime_error("批次基准状态无效: " + state_error);
        }
        const double baseline_position = baseline_positions[selected_joint_index];
        const double positive_target_position = baseline_position + joint_delta;
        RCLCPP_INFO(
            node->get_logger(),
            "重复性固定目标: joint=%s, baseline=%.9f rad, positive=%.9f rad, "
            "cycles=%d, minimum_completion=%.2f%%, maximum_range=%.9f rad",
            joint_name.c_str(), baseline_position, positive_target_position,
            cycles, minimum_completion_ratio * 100.0, maximum_position_range);

        massage_motion::PlannerConfig planner_config;
        planner_config.planning_group = "jaka_s5";
        planner_config.end_effector_link = "massage_tool_tip";
        planner_config.reference_frame = "world";
        planner_config.planning_pipeline = "pilz_industrial_motion_planner";
        auto planner = std::make_shared<massage_motion::MotionPlanningSdk>(
            node, planner_config);
        auto backend =
            std::make_shared<massage_motion::MoveItTrajectoryExecutor>(node);
        auto trajectory_executor =
            std::make_shared<massage_motion::GuardedTrajectoryExecutor>(
                backend,
                [&]() -> massage_motion::ExecutionValidationResult
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    const auto now = std::chrono::steady_clock::now();
                    const double joint_age = std::chrono::duration<double>(
                        now - joint_received_at).count();
                    const double robot_age = std::chrono::duration<double>(
                        now - robot_received_at).count();
                    if (joint_sequence == 0U || robot_sequence == 0U ||
                        joint_age > feedback_timeout || robot_age > feedback_timeout)
                    {
                        return {
                            false, massage_motion::ExecutionError::kRejected,
                            "关节状态或机器人状态缺失、过期"};
                    }
                    if (!robot_ready(latest_robot_state))
                    {
                        return {
                            false, massage_motion::ExecutionError::kRejected,
                            "机器人不再满足静止、上电、使能、无碰撞条件"};
                    }
                    return {
                        true, massage_motion::ExecutionError::kNone, "ready"};
                });

        if (!execute)
        {
            auto target = baseline_positions;
            target[selected_joint_index] = positive_target_position;
            massage_motion::MotionRequest request;
            request.request_id = "repeatability_dry_run";
            request.motion_type = massage_motion::MotionType::kPtp;
            request.target = massage_motion::JointTarget{target};
            request.velocity_scale = velocity_scale;
            request.acceleration_scale = acceleration_scale;
            request.planning_timeout = planning_timeout;
            const auto plan = planner->plan(request);
            if (!plan.success)
            {
                throw std::runtime_error(
                    "重复性首个正向目标只规划失败: " + plan.message);
            }
            RCLCPP_INFO(
                node->get_logger(),
                "重复性只规划通过: points=%zu；execute=false，未发送运动命令",
                plan.trajectory.joint_trajectory.points.size());
            exit_code = 0;
        }
        else
        {
            auto parameter_node = std::make_shared<rclcpp::Node>(
                "repeatability_driver_parameter_client");
            auto parameter_client =
                std::make_shared<rclcpp::SyncParametersClient>(
                parameter_node, "/jaka_driver");
            if (!parameter_client->wait_for_service(
                    std::chrono::duration<double>(state_timeout)))
            {
                throw std::runtime_error(
                    "等待 /jaka_driver 参数服务超时");
            }
            const auto driver_parameters = parameter_client->get_parameters({
                "trajectory_goal_tolerance", "trajectory_goal_timeout"});
            if (driver_parameters.size() != 2U)
            {
                throw std::runtime_error("驱动终点参数读取不完整");
            }
            const double driver_goal_tolerance =
                driver_parameters[0].as_double();
            const double driver_goal_timeout =
                driver_parameters[1].as_double();
            const double minimum_round_trip_command =
                joint_delta - driver_goal_tolerance;
            const double guaranteed_completion =
                minimum_round_trip_command > 0.0 ?
                1.0 - driver_goal_tolerance / minimum_round_trip_command : 0.0;
            const auto execution_contract =
                massage_motion::validate_trajectory_execution_contract(
                driver_goal_tolerance, driver_goal_timeout,
                endpoint_tolerance, execution_timeout_margin);
            if (!execution_contract.valid ||
                guaranteed_completion < minimum_completion_ratio)
            {
                const double required_goal_tolerance =
                    joint_delta * (1.0 - minimum_completion_ratio) /
                    (2.0 - minimum_completion_ratio);
                std::ostringstream message;
                message << "驱动终点参数与重复性门槛不一致: "
                        << execution_contract.message << "; tolerance="
                        << driver_goal_tolerance << " rad, timeout="
                        << driver_goal_timeout << " s, execution_margin="
                        << execution_timeout_margin << " s, worst_completion="
                        << guaranteed_completion * 100.0
                        << "%, required_completion="
                        << minimum_completion_ratio * 100.0
                        << "%, tolerance 必须不大于 "
                        << required_goal_tolerance << " rad";
                throw std::runtime_error(message.str());
            }
            RCLCPP_INFO(
                node->get_logger(),
                "驱动终点门槛通过: tolerance=%.9f rad, "
                "driver_margin=%.3f s, execution_margin=%.3f s, "
                "最差往返保证完成度=%.2f%%",
                driver_goal_tolerance, driver_goal_timeout,
                execution_timeout_margin, guaranteed_completion * 100.0);

            bool stop_batch = false;
            for (int cycle = 1; cycle <= cycles && !stop_batch; ++cycle)
            {
                for (int leg = 0; leg < 2 && !stop_batch; ++leg)
                {
                    const auto direction = leg == 0 ?
                        massage_motion::RepeatabilityDirection::kPositive :
                        massage_motion::RepeatabilityDirection::kNegative;
                    const double target_position = leg == 0 ?
                        positive_target_position : baseline_position;
                    sensor_msgs::msg::JointState initial_state;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        initial_state = latest_joint_state;
                    }
                    std::vector<double> initial_positions;
                    if (!extract_ordered_positions(
                            initial_state, initial_positions, state_error))
                    {
                        throw std::runtime_error(
                            "第 " + std::to_string(cycle) +
                            " 轮初始状态无效: " + state_error);
                    }
                    const double initial_position =
                        initial_positions[selected_joint_index];
                    const double commanded_delta =
                        target_position - initial_position;
                    const bool direction_valid =
                        (direction ==
                            massage_motion::RepeatabilityDirection::kPositive &&
                            commanded_delta > endpoint_tolerance) ||
                        (direction ==
                            massage_motion::RepeatabilityDirection::kNegative &&
                            commanded_delta < -endpoint_tolerance);
                    auto target_positions = baseline_positions;
                    target_positions[selected_joint_index] = target_position;
                    double maximum_commanded_delta = 0.0;
                    for (std::size_t joint = 0; joint < kJointNames.size(); ++joint)
                    {
                        maximum_commanded_delta = std::max(
                            maximum_commanded_delta,
                            std::abs(
                                target_positions[joint] -
                                initial_positions[joint]));
                    }
                    if (!direction_valid ||
                        maximum_commanded_delta > maximum_joint_delta)
                    {
                        std::ostringstream message;
                        message << "固定目标与当前反馈形成的命令方向或幅值无效: "
                                << "selected_delta=" << commanded_delta
                                << " rad, maximum_joint_delta="
                                << maximum_commanded_delta << " rad, limit="
                                << maximum_joint_delta << " rad";
                        throw std::runtime_error(message.str());
                    }

                    massage_motion::MotionRequest request;
                    request.request_id =
                        "repeatability_" + std::to_string(cycle) + "_" +
                        massage_motion::to_string(direction);
                    request.motion_type = massage_motion::MotionType::kPtp;
                    request.target = massage_motion::JointTarget{target_positions};
                    request.velocity_scale = velocity_scale;
                    request.acceleration_scale = acceleration_scale;
                    request.planning_timeout = planning_timeout;

                    massage_motion::MotionRepeatabilityTrial trial;
                    trial.cycle = static_cast<std::size_t>(cycle);
                    trial.direction = direction;
                    trial.initial_position = initial_position;
                    trial.target_position = target_position;
                    trial.actual_position = initial_position;
                    trial.commanded_delta = commanded_delta;

                    const auto plan = planner->plan(request);
                    if (!plan.success)
                    {
                        trial.endpoint_error = std::abs(commanded_delta);
                        trial.maximum_endpoint_error = trial.endpoint_error;
                        trial.message = "规划失败: " + plan.message;
                    }
                    else
                    {
                        massage_motion::ExecutionTimingPolicy timing_policy;
                        timing_policy.margin = execution_timeout_margin;
                        const auto timing =
                            massage_motion::calculate_execution_timing(
                                plan.trajectory, timing_policy);
                        if (!timing.valid)
                        {
                            trial.endpoint_error = std::abs(commanded_delta);
                            trial.maximum_endpoint_error = trial.endpoint_error;
                            trial.message = "执行时限无效: " + timing.message;
                        }
                        else
                        {
                            trial.planned_duration = timing.expected_duration;
                            massage_motion::ExecutionRequest execution_request;
                            execution_request.request_id =
                                request.request_id + "_execution";
                            execution_request.robot_trajectory = plan.trajectory;
                            execution_request.timeout = timing.timeout;
                            const auto execution_started =
                                std::chrono::steady_clock::now();
                            const auto execution =
                                trajectory_executor->execute(execution_request);
                            trial.execution_duration = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() -
                                execution_started).count();

                            std::uint64_t sequence_before_settling;
                            {
                                std::lock_guard<std::mutex> lock(state_mutex);
                                sequence_before_settling = joint_sequence;
                            }
                            std::this_thread::sleep_for(
                                std::chrono::duration<double>(settling_duration));
                            sensor_msgs::msg::JointState final_state;
                            jaka_msgs::msg::RobotMsg final_robot_state;
                            double final_joint_age = 0.0;
                            double final_robot_age = 0.0;
                            {
                                std::unique_lock<std::mutex> lock(state_mutex);
                                if (!state_condition.wait_for(
                                        lock,
                                        std::chrono::duration<double>(state_timeout),
                                        [&]()
                                        {
                                            return joint_sequence >
                                                sequence_before_settling;
                                        }))
                                {
                                    throw std::runtime_error(
                                        "等待本程执行后的关节反馈超时");
                                }
                                final_state = latest_joint_state;
                                final_robot_state = latest_robot_state;
                                const auto feedback_checked_at =
                                    std::chrono::steady_clock::now();
                                final_joint_age = std::chrono::duration<double>(
                                    feedback_checked_at - joint_received_at).count();
                                final_robot_age = std::chrono::duration<double>(
                                    feedback_checked_at - robot_received_at).count();
                            }

                            const auto endpoint =
                                massage_motion::calculate_trajectory_endpoint_error(
                                    plan.trajectory, final_state);
                            const auto selected_error = std::find_if(
                                endpoint.joint_errors.begin(),
                                endpoint.joint_errors.end(),
                                [&](const auto & joint_error)
                                {
                                    return joint_error.joint_name == joint_name;
                                });
                            if (selected_error == endpoint.joint_errors.end())
                            {
                                throw std::runtime_error(
                                    "终点误差结果缺少被测关节");
                            }
                            trial.actual_position =
                                selected_error->actual_position;
                            trial.achieved_delta =
                                trial.actual_position - initial_position;
                            trial.completion_ratio =
                                trial.achieved_delta / commanded_delta;
                            trial.endpoint_error = selected_error->absolute_error;
                            trial.maximum_endpoint_error =
                                endpoint.max_absolute_error;
                            const bool final_feedback_fresh =
                                final_joint_age <= feedback_timeout &&
                                final_robot_age <= feedback_timeout;
                            const bool final_robot_ready =
                                robot_ready(final_robot_state);
                            trial.passed = execution.success && endpoint.valid &&
                                final_feedback_fresh && final_robot_ready &&
                                trial.maximum_endpoint_error <= endpoint_tolerance &&
                                trial.completion_ratio >= minimum_completion_ratio;
                            std::ostringstream message;
                            message << (trial.passed ? "passed" : "failed")
                                    << "; execution=" << execution.message
                                    << "; endpoint=" << endpoint.message
                                    << "; final_feedback_fresh="
                                    << final_feedback_fresh
                                    << "; robot_ready=" << final_robot_ready
                                    << "; joint_age=" << final_joint_age
                                    << " s; robot_age=" << final_robot_age << " s";
                            trial.message = message.str();
                        }
                    }

                    trials.push_back(trial);
                    if (!write_trials_csv(output_csv, trials))
                    {
                        throw std::runtime_error(
                            "无法写入重复性 CSV: " + output_csv);
                    }
                    RCLCPP_INFO(
                        node->get_logger(),
                        "重复性 %d/%d %s: passed=%s, initial=%.9f, "
                        "target=%.9f, actual=%.9f, commanded=%.9f, "
                        "achieved=%.9f, completion=%.2f%%, error=%.9f, "
                        "max_error=%.9f, execution=%.3f s",
                        cycle, cycles, massage_motion::to_string(direction),
                        trial.passed ? "true" : "false",
                        trial.initial_position, trial.target_position,
                        trial.actual_position, trial.commanded_delta,
                        trial.achieved_delta, trial.completion_ratio * 100.0,
                        trial.endpoint_error, trial.maximum_endpoint_error,
                        trial.execution_duration);
                    if (!trial.passed)
                    {
                        RCLCPP_ERROR(
                            node->get_logger(),
                            "重复性批次在首个失败样本后停止: %s",
                            trial.message.c_str());
                        stop_batch = true;
                    }
                    else if (!(cycle == cycles && leg == 1))
                    {
                        std::this_thread::sleep_for(
                            std::chrono::duration<double>(inter_trial_delay));
                    }
                }
            }

            const auto summary =
                massage_motion::analyze_motion_repeatability(trials);
            if (summary.valid)
            {
                log_direction_summary(
                    node->get_logger(), "positive", summary.positive);
                log_direction_summary(
                    node->get_logger(), "negative", summary.negative);
            }
            const auto expected_samples = static_cast<std::size_t>(cycles * 2);
            const bool complete = summary.valid && summary.all_passed &&
                summary.samples == expected_samples;
            const bool range_passed = complete &&
                summary.positive.position_range <= maximum_position_range &&
                summary.negative.position_range <= maximum_position_range;
            if (!complete || !range_passed)
            {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "REPEATABILITY TEST: FAIL: complete=%s, range_passed=%s, "
                    "samples=%zu/%zu, csv=%s",
                    complete ? "true" : "false",
                    range_passed ? "true" : "false",
                    summary.samples, expected_samples, output_csv.c_str());
                exit_code = 5;
            }
            else
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "REPEATABILITY TEST: PASS: samples=%zu, csv=%s",
                    summary.samples, output_csv.c_str());
                exit_code = 0;
            }
        }
    }
    catch (const std::exception & exception)
    {
        if (!trials.empty())
        {
            (void)write_trials_csv(output_csv, trials);
        }
        RCLCPP_ERROR(
            node->get_logger(),
            "真机重复性测试异常: %s; csv=%s",
            exception.what(), output_csv.c_str());
        exit_code = 6;
    }

    (void)joint_subscription;
    (void)robot_subscription;
    ros_executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
