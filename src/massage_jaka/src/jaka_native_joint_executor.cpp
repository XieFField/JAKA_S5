#include "massage_jaka/jaka_native_joint_executor.hpp"

#include <cmath>
#include <future>
#include <stdexcept>
#include <utility>

namespace massage_jaka
{
namespace
{

massage_motion::ExecutionResult failure(
  massage_motion::ExecutionError error,
  const std::string & message,
  std::int32_t backend_code = 0,
  massage_motion::ExecutionStatus status =
  massage_motion::ExecutionStatus::kFailed)
{
  return {false, error, backend_code, message, status};
}

}  // namespace

JakaNativeJointExecutor::JakaNativeJointExecutor(
  rclcpp::Node::SharedPtr node,
  JakaNativeJointExecutorConfig config)
: node_(std::move(node)),
  logger_(node_ ? node_->get_logger() : rclcpp::get_logger("jaka_native_ptp")),
  config_(std::move(config))
{
  if (!node_ || config_.action_name.empty() ||
    config_.joint_state_topic.empty() || config_.rapid_rate_service.empty() ||
    !std::isfinite(config_.joint_state_timeout) ||
    config_.joint_state_timeout <= 0.0 ||
    !std::isfinite(config_.action_server_timeout) ||
    config_.action_server_timeout <= 0.0 ||
    !std::isfinite(config_.result_grace_period) ||
    config_.result_grace_period <= 0.0)
  {
    throw std::invalid_argument("JAKA 原生 PTP 执行器配置无效");
  }
  client_ = rclcpp_action::create_client<Action>(node_, config_.action_name);
  rapid_rate_client_ =
    node_->create_client<jaka_msgs::srv::GetRapidRate>(
    config_.rapid_rate_service);
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_state_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = *message;
      state_received_at_ = std::chrono::steady_clock::now();
    });
}

massage_motion::ExecutionResult JakaNativeJointExecutor::execute(
  const massage_motion::ExecutionRequest & request)
{
  std::unique_lock<std::mutex> execution_lock(execution_mutex_, std::try_to_lock);
  if (!execution_lock.owns_lock())
  {
    return failure(
      massage_motion::ExecutionError::kRejected,
      "JAKA 原生 PTP 执行器已有活动请求");
  }
  if (!request.has_motion_semantics ||
    request.motion_type != massage_motion::MotionType::kPtp)
  {
    return failure(
      massage_motion::ExecutionError::kInvalidRequest,
      "JAKA 原生关节执行器只接受显式 PTP 请求");
  }

  sensor_msgs::msg::JointState current;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_received_at_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - state_received_at_).count() >
      config_.joint_state_timeout)
    {
      return failure(
        massage_motion::ExecutionError::kRejected,
        "原生 PTP 执行前没有新鲜的 /joint_states");
    }
    current = latest_state_;
  }
  if (!rapid_rate_client_->wait_for_service(
      std::chrono::duration<double>(config_.action_server_timeout)))
  {
    return failure(
      massage_motion::ExecutionError::kBackendUnavailable,
      "等待 /jaka_driver/get_rapid_rate 服务超时");
  }
  auto rapid_request =
    std::make_shared<jaka_msgs::srv::GetRapidRate::Request>();
  auto rapid_future = rapid_rate_client_->async_send_request(rapid_request);
  if (rapid_future.wait_for(std::chrono::duration<double>(
      config_.action_server_timeout)) != std::future_status::ready)
  {
    return failure(
      massage_motion::ExecutionError::kTimeout,
      "读取 JAKA 全局速度倍率超时", 0,
      massage_motion::ExecutionStatus::kTimedOut);
  }
  const auto rapid_response = rapid_future.get();
  if (!rapid_response || !rapid_response->success ||
    !std::isfinite(rapid_response->rapid_rate) ||
    rapid_response->rapid_rate <= 0.0 || rapid_response->rapid_rate > 1.0)
  {
    return failure(
      massage_motion::ExecutionError::kRejected,
      rapid_response ? rapid_response->message :
      "JAKA 全局速度倍率响应无效",
      rapid_response ? rapid_response->error_code : 0);
  }
  const auto command = massage_motion::make_native_ptp_command(
    request.robot_trajectory, current, request.timeout, config_.ptp,
    rapid_response->rapid_rate);
  if (!command.valid)
  {
    return failure(
      massage_motion::ExecutionError::kRejected,
      "NATIVE PTP EQUIVALENCE GATE: REJECTED: " + command.message);
  }
  if (command.already_at_target)
  {
    RCLCPP_INFO(
      logger_,
      "NATIVE PTP ALREADY AT TARGET: PASS: request=%s, start_error=%.9f rad; 未发送 joint_move Goal",
      request.request_id.c_str(), command.maximum_start_error);
    status_.store(massage_motion::ExecutionStatus::kSucceeded);
    return {
      true, massage_motion::ExecutionError::kNone, 0,
      command.message, massage_motion::ExecutionStatus::kSucceeded};
  }
  RCLCPP_INFO(
    logger_,
    "NATIVE PTP EQUIVALENCE GATE: PASS: request=%s, planner=%s, start_error=%.9f rad, path_deviation=%.9f rad, programmed_speed=%.6f rad/s, rapid_rate=%.3f, effective_speed=%.6f rad/s, acceleration=%.6f rad/s^2, estimated_duration=%.3f s, effective_timeout=%.3f s",
    request.request_id.c_str(), request.planner_id.c_str(),
    command.maximum_start_error, command.maximum_path_deviation,
    command.speed, command.rapid_rate, command.effective_speed,
    command.acceleration, command.estimated_duration,
    command.effective_timeout);

  if (!client_->wait_for_action_server(
      std::chrono::duration<double>(config_.action_server_timeout)))
  {
    return failure(
      massage_motion::ExecutionError::kBackendUnavailable,
      "等待 /jaka_driver/execute_joint_move Action 超时");
  }

  Action::Goal goal;
  goal.request_id = request.request_id;
  goal.target_positions = command.target_positions;
  goal.speed = command.speed;
  goal.acceleration = command.acceleration;
  goal.endpoint_tolerance = config_.ptp.endpoint_tolerance;
  goal.timeout = command.effective_timeout;
  auto goal_future = client_->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::duration<double>(
      config_.action_server_timeout)) != std::future_status::ready)
  {
    client_->async_cancel_all_goals();
    return failure(
      massage_motion::ExecutionError::kTimeout,
      "原生 joint_move Goal 响应超时，已请求取消该 Action 的全部 Goal", 0,
      massage_motion::ExecutionStatus::kTimedOut);
  }
  auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    return failure(
      massage_motion::ExecutionError::kRejected,
      "原生 joint_move Goal 被驱动拒绝");
  }
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_ = goal_handle;
  }
  status_.store(massage_motion::ExecutionStatus::kExecuting);
  auto result_future = client_->async_get_result(goal_handle);
  const auto client_timeout = command.effective_timeout +
    config_.result_grace_period;
  if (result_future.wait_for(std::chrono::duration<double>(client_timeout)) !=
    std::future_status::ready)
  {
    client_->async_cancel_goal(goal_handle);
    status_.store(massage_motion::ExecutionStatus::kTimedOut);
    return failure(
      massage_motion::ExecutionError::kTimeout,
      "等待原生 joint_move 终态超时，已请求取消", 0,
      massage_motion::ExecutionStatus::kTimedOut);
  }
  const auto wrapped = result_future.get();
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
  }
  if (wrapped.code == rclcpp_action::ResultCode::CANCELED)
  {
    status_.store(massage_motion::ExecutionStatus::kCanceled);
    return failure(
      massage_motion::ExecutionError::kCanceled,
      wrapped.result ? wrapped.result->message : "原生 PTP 已取消",
      wrapped.result ? wrapped.result->sdk_error_code : 0,
      massage_motion::ExecutionStatus::kCanceled);
  }
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped.result || !wrapped.result->success)
  {
    status_.store(massage_motion::ExecutionStatus::kFailed);
    return failure(
      massage_motion::ExecutionError::kExecutionFailed,
      wrapped.result ? wrapped.result->message : "原生 PTP Action 失败",
      wrapped.result ? wrapped.result->sdk_error_code : 0);
  }
  status_.store(massage_motion::ExecutionStatus::kSucceeded);
  return {
    true, massage_motion::ExecutionError::kNone,
    wrapped.result->sdk_error_code,
    wrapped.result->message,
    massage_motion::ExecutionStatus::kSucceeded};
}

bool JakaNativeJointExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!active_goal_) return false;
  client_->async_cancel_goal(active_goal_);
  return true;
}

massage_motion::ExecutionStatus JakaNativeJointExecutor::status() const
{
  return status_.load();
}

}  // namespace massage_jaka
