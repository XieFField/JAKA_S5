#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "jaka_msgs/srv/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

const std::vector<std::string> kJointNames{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};

bool finite_values(const std::vector<double> & values)
{
    return std::all_of(
        values.begin(), values.end(),
        [](double value) {return std::isfinite(value);});
}

bool reorder_joint_state(
    const sensor_msgs::msg::JointState & state,
    std::vector<double> & positions)
{
    positions.clear();
    positions.reserve(kJointNames.size());
    for (const auto & joint_name : kJointNames)
    {
        const auto iterator = std::find(
            state.name.begin(), state.name.end(), joint_name);
        if (iterator == state.name.end())
        {
            return false;
        }
        const auto index = static_cast<std::size_t>(
            std::distance(state.name.begin(), iterator));
        if (index >= state.position.size() ||
            !std::isfinite(state.position[index]))
        {
            return false;
        }
        positions.push_back(state.position[index]);
    }
    return true;
}

builtin_interfaces::msg::Duration duration_from_seconds(double seconds)
{
    return static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(seconds));
}

FollowJointTrajectory::Goal make_stationary_goal(
    const std::vector<double> & positions,
    double duration)
{
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = kJointNames;
    constexpr double kSegmentDuration = 0.1;
    for (double point_time = kSegmentDuration;
        point_time < duration; point_time += kSegmentDuration)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.velocities.assign(kJointNames.size(), 0.0);
        point.time_from_start = duration_from_seconds(point_time);
        goal.trajectory.points.push_back(point);
    }
    trajectory_msgs::msg::JointTrajectoryPoint final_point;
    final_point.positions = positions;
    final_point.velocities.assign(kJointNames.size(), 0.0);
    final_point.time_from_start = duration_from_seconds(duration);
    goal.trajectory.points.push_back(final_point);
    goal.goal_time_tolerance = duration_from_seconds(2.0);
    return goal;
}

template<typename FutureT>
bool ready_within(FutureT & future, double timeout)
{
    return future.wait_for(std::chrono::duration<double>(timeout)) ==
           std::future_status::ready;
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "real_action_protocol_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    bool run_protocol_test = false;
    bool test_external_stop = false;
    double joint_state_timeout = 3.0;
    double action_timeout = 5.0;
    double hold_duration = 3.0;
    double maximum_stationary_delta = 0.005;
    node->get_parameter_or("run_protocol_test", run_protocol_test, false);
    node->get_parameter_or("test_external_stop", test_external_stop, false);
    node->get_parameter_or("joint_state_timeout", joint_state_timeout, 3.0);
    node->get_parameter_or("action_timeout", action_timeout, 5.0);
    node->get_parameter_or("hold_duration", hold_duration, 3.0);
    node->get_parameter_or(
        "maximum_stationary_delta", maximum_stationary_delta, 0.005);

    if (!std::isfinite(joint_state_timeout) || joint_state_timeout <= 0.0 ||
        !std::isfinite(action_timeout) || action_timeout <= 0.0 ||
        !std::isfinite(hold_duration) || hold_duration <= 1.0 ||
        !std::isfinite(maximum_stationary_delta) ||
        maximum_stationary_delta <= 0.0)
    {
        RCLCPP_ERROR(node->get_logger(), "Action 协议验证参数无效");
        rclcpp::shutdown();
        return 2;
    }

    std::mutex state_mutex;
    std::condition_variable state_condition;
    sensor_msgs::msg::JointState latest_state;
    bool state_received = false;
    auto state_subscription =
        node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [&](sensor_msgs::msg::JointState::SharedPtr message)
            {
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    latest_state = *message;
                    state_received = true;
                }
                state_condition.notify_all();
            });

    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
        node, "/jaka_s5_controller/follow_joint_trajectory");
    auto legacy_move_client =
        node->create_client<jaka_msgs::srv::Move>("/jaka_driver/joint_move");
    auto stop_client =
        node->create_client<std_srvs::srv::Trigger>("/jaka_driver/stop_move");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    int exit_code = 1;
    GoalHandle::SharedPtr active_goal;
    try
    {
        {
            std::unique_lock<std::mutex> lock(state_mutex);
            if (!state_condition.wait_for(
                    lock,
                    std::chrono::duration<double>(joint_state_timeout),
                    [&state_received]() {return state_received;}))
            {
                throw std::runtime_error("等待 /joint_states 超时");
            }
        }

        std::vector<double> stationary_positions;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            if (!reorder_joint_state(latest_state, stationary_positions))
            {
                throw std::runtime_error("/joint_states 缺少有效的 JAKA S5 六轴位置");
            }
        }

        if (!action_client->wait_for_action_server(
                std::chrono::duration<double>(action_timeout)))
        {
            throw std::runtime_error("等待 FollowJointTrajectory Action 超时");
        }

        if (!run_protocol_test)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "只读门禁通过：关节状态和轨迹 Action 均可用；未发送任何 Goal。"
                "设置 run_protocol_test:=true 后执行零位移协议验证");
            exit_code = 0;
        }
        else
        {
            std::mutex feedback_mutex;
            std::condition_variable feedback_condition;
            std::size_t feedback_count = 0;
            bool feedback_valid = true;
            std::string feedback_error;
            double maximum_actual_delta = 0.0;
            double maximum_reported_error = 0.0;

            rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
            options.feedback_callback =
                [&](GoalHandle::SharedPtr,
                    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
                {
                    bool valid = feedback->joint_names == kJointNames &&
                        feedback->desired.positions.size() == kJointNames.size() &&
                        feedback->actual.positions.size() == kJointNames.size() &&
                        feedback->error.positions.size() == kJointNames.size() &&
                        finite_values(feedback->desired.positions) &&
                        finite_values(feedback->actual.positions) &&
                        finite_values(feedback->error.positions);

                    std::lock_guard<std::mutex> lock(feedback_mutex);
                    if (!valid)
                    {
                        feedback_valid = false;
                        feedback_error = "feedback 的关节名、数组长度或数值无效";
                    }
                    else
                    {
                        for (std::size_t index = 0;
                            index < kJointNames.size(); ++index)
                        {
                            maximum_actual_delta = std::max(
                                maximum_actual_delta,
                                std::abs(
                                    feedback->actual.positions[index] -
                                    stationary_positions[index]));
                            maximum_reported_error = std::max(
                                maximum_reported_error,
                                std::abs(feedback->error.positions[index]));
                        }
                    }
                    ++feedback_count;
                    feedback_condition.notify_all();
                };

            const auto stationary_goal = make_stationary_goal(
                stationary_positions, hold_duration);
            auto first_goal_future = action_client->async_send_goal(
                stationary_goal, options);
            if (!ready_within(first_goal_future, action_timeout))
            {
                throw std::runtime_error("等待首个 Goal 响应超时");
            }
            const auto first_goal_handle = first_goal_future.get();
            if (!first_goal_handle)
            {
                throw std::runtime_error("首个零位移 Goal 被拒绝");
            }
            active_goal = first_goal_handle;

            auto second_goal_future = action_client->async_send_goal(stationary_goal);
            if (!ready_within(second_goal_future, action_timeout))
            {
                throw std::runtime_error("等待并发 Goal 响应超时");
            }
            const auto second_goal_handle = second_goal_future.get();
            if (second_goal_handle)
            {
                auto second_cancel_future =
                    action_client->async_cancel_goal(second_goal_handle);
                (void)ready_within(second_cancel_future, action_timeout);
                throw std::runtime_error("驱动错误地接受了第二个并发 Goal");
            }

            {
                std::unique_lock<std::mutex> lock(feedback_mutex);
                if (!feedback_condition.wait_for(
                        lock,
                        std::chrono::duration<double>(action_timeout),
                        [&feedback_count]() {return feedback_count > 0;}))
                {
                    throw std::runtime_error("等待 Action feedback 超时");
                }
                if (!feedback_valid)
                {
                    throw std::runtime_error(feedback_error);
                }
            }

            auto result_future = action_client->async_get_result(first_goal_handle);
            auto cancel_future = action_client->async_cancel_goal(first_goal_handle);
            if (!ready_within(cancel_future, action_timeout))
            {
                throw std::runtime_error("等待取消响应超时");
            }
            if (!ready_within(result_future, action_timeout))
            {
                throw std::runtime_error("等待取消终态超时");
            }
            if (result_future.get().code != rclcpp_action::ResultCode::CANCELED)
            {
                throw std::runtime_error("首个 Goal 未进入 CANCELED 终态");
            }
            active_goal.reset();

            const auto recovery_goal = make_stationary_goal(
                stationary_positions, 0.5);
            auto recovery_goal_future = action_client->async_send_goal(recovery_goal);
            if (!ready_within(recovery_goal_future, action_timeout))
            {
                throw std::runtime_error("等待恢复 Goal 响应超时");
            }
            const auto recovery_goal_handle = recovery_goal_future.get();
            if (!recovery_goal_handle)
            {
                throw std::runtime_error("取消后恢复 Goal 被拒绝");
            }
            active_goal = recovery_goal_handle;
            auto recovery_result_future =
                action_client->async_get_result(recovery_goal_handle);
            if (!ready_within(recovery_result_future, action_timeout))
            {
                throw std::runtime_error("等待恢复 Goal 终态超时");
            }
            if (recovery_result_future.get().code !=
                rclcpp_action::ResultCode::SUCCEEDED)
            {
                throw std::runtime_error("取消后恢复 Goal 未成功");
            }
            active_goal.reset();

            if (test_external_stop)
            {
                if (!legacy_move_client->wait_for_service(
                        std::chrono::duration<double>(action_timeout)) ||
                    !stop_client->wait_for_service(
                        std::chrono::duration<double>(action_timeout)))
                {
                    throw std::runtime_error(
                        "等待 legacy move 或 stop_move 服务超时");
                }

                auto stop_goal_future = action_client->async_send_goal(
                    stationary_goal, options);
                if (!ready_within(stop_goal_future, action_timeout))
                {
                    throw std::runtime_error("等待外部停止测试 Goal 响应超时");
                }
                const auto stop_goal_handle = stop_goal_future.get();
                if (!stop_goal_handle)
                {
                    throw std::runtime_error("外部停止测试 Goal 被拒绝");
                }
                active_goal = stop_goal_handle;
                auto stop_result_future =
                    action_client->async_get_result(stop_goal_handle);

                auto legacy_request =
                    std::make_shared<jaka_msgs::srv::Move::Request>();
                legacy_request->pose.reserve(stationary_positions.size());
                for (const double position : stationary_positions)
                {
                    legacy_request->pose.push_back(
                        static_cast<float>(position));
                }
                legacy_request->mvvelo = 0.01F;
                legacy_request->mvacc = 0.01F;
                auto legacy_future =
                    legacy_move_client->async_send_request(legacy_request);
                if (!ready_within(legacy_future, action_timeout))
                {
                    throw std::runtime_error("等待 legacy move 拒绝响应超时");
                }
                const auto legacy_response = legacy_future.get();
                if (legacy_response->ret != 0 ||
                    legacy_response->message.find("trajectory") ==
                    std::string::npos)
                {
                    throw std::runtime_error(
                        "轨迹控制期间 legacy move 未按预期被拒绝: " +
                        legacy_response->message);
                }

                auto stop_request =
                    std::make_shared<std_srvs::srv::Trigger::Request>();
                auto stop_future = stop_client->async_send_request(stop_request);
                if (!ready_within(stop_future, action_timeout))
                {
                    throw std::runtime_error("等待 stop_move 响应超时");
                }
                const auto stop_response = stop_future.get();
                if (!stop_response->success)
                {
                    throw std::runtime_error(
                        "stop_move 执行失败: " + stop_response->message);
                }
                if (!ready_within(stop_result_future, action_timeout))
                {
                    throw std::runtime_error("等待外部停止后的 Action 终态超时");
                }
                if (stop_result_future.get().code !=
                    rclcpp_action::ResultCode::ABORTED)
                {
                    throw std::runtime_error(
                        "外部 stop_move 后 Action 未进入 ABORTED 终态");
                }
                active_goal.reset();

                auto stop_recovery_goal_future =
                    action_client->async_send_goal(recovery_goal);
                if (!ready_within(stop_recovery_goal_future, action_timeout))
                {
                    throw std::runtime_error("等待外部停止后恢复 Goal 响应超时");
                }
                const auto stop_recovery_goal_handle =
                    stop_recovery_goal_future.get();
                if (!stop_recovery_goal_handle)
                {
                    throw std::runtime_error("外部停止后恢复 Goal 被拒绝");
                }
                active_goal = stop_recovery_goal_handle;
                auto stop_recovery_result_future =
                    action_client->async_get_result(stop_recovery_goal_handle);
                if (!ready_within(stop_recovery_result_future, action_timeout))
                {
                    throw std::runtime_error("等待外部停止后恢复终态超时");
                }
                if (stop_recovery_result_future.get().code !=
                    rclcpp_action::ResultCode::SUCCEEDED)
                {
                    throw std::runtime_error("外部停止后恢复 Goal 未成功");
                }
                active_goal.reset();
            }

            std::size_t final_feedback_count;
            double final_maximum_actual_delta;
            double final_maximum_reported_error;
            {
                std::lock_guard<std::mutex> lock(feedback_mutex);
                final_feedback_count = feedback_count;
                final_maximum_actual_delta = maximum_actual_delta;
                final_maximum_reported_error = maximum_reported_error;
            }
            if (final_maximum_actual_delta > maximum_stationary_delta)
            {
                throw std::runtime_error(
                    "零位移验证期间实际关节变化超过限制: " +
                    std::to_string(final_maximum_actual_delta) + " rad");
            }

            RCLCPP_INFO(
                node->get_logger(),
                "Action 协议验证通过: feedback=%zu, 并发 Goal 已拒绝, "
                "取消与恢复成功%s, 最大实际位移=%.9f rad, 最大反馈误差=%.9f rad",
                final_feedback_count,
                test_external_stop ? ", legacy 互斥与外部停止恢复成功" : "",
                final_maximum_actual_delta,
                final_maximum_reported_error);
            exit_code = 0;
        }
    }
    catch (const std::exception & exception)
    {
        if (active_goal)
        {
            auto cleanup_future = action_client->async_cancel_goal(active_goal);
            (void)ready_within(cleanup_future, action_timeout);
        }
        RCLCPP_ERROR(
            node->get_logger(), "Action 协议验证失败: %s", exception.what());
        exit_code = 3;
    }

    (void)state_subscription;
    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
