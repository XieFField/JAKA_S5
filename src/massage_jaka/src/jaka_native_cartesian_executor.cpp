#include "massage_jaka/jaka_native_cartesian_executor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace massage_jaka
{
namespace
{

massage_motion::ExecutionResult failure(
  massage_motion::ExecutionError error, const std::string & message,
  std::int32_t backend_code = 0,
  massage_motion::ExecutionStatus status =
  massage_motion::ExecutionStatus::kFailed)
{
  return {false, error, backend_code, message, status};
}

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

std::array<double, 6> sdk_pose(
  const geometry_msgs::msg::PoseStamped & value,
  const std::string & expected_frame)
{
  if (value.header.frame_id != expected_frame)
  {
    throw std::invalid_argument(
            "原生笛卡尔目标必须位于 " + expected_frame + " 坐标系");
  }
  const auto & q = value.pose.orientation;
  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  if (!std::isfinite(quaternion.length2()) || quaternion.length2() < 1.0e-12)
  {
    throw std::invalid_argument("原生笛卡尔目标四元数无效");
  }
  quaternion.normalize();
  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  std::array<double, 6> result{{
      value.pose.position.x * 1000.0,
      value.pose.position.y * 1000.0,
      value.pose.position.z * 1000.0,
      roll, pitch, yaw}};
  if (!std::all_of(
      result.begin(), result.end(),
      [](double item) {return std::isfinite(item);}))
  {
    throw std::invalid_argument("原生笛卡尔目标包含非有限值");
  }
  return result;
}

double start_error(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const sensor_msgs::msg::JointState & current)
{
  if (trajectory.joint_trajectory.points.empty()) return INFINITY;
  const auto & names = trajectory.joint_trajectory.joint_names;
  const auto & planned = trajectory.joint_trajectory.points.front().positions;
  if (names.size() != planned.size()) return INFINITY;
  std::unordered_map<std::string, double> actual;
  for (std::size_t index = 0; index < current.name.size() &&
    index < current.position.size(); ++index)
  {
    actual[current.name[index]] = current.position[index];
  }
  double result = 0.0;
  for (std::size_t index = 0; index < names.size(); ++index)
  {
    const auto found = actual.find(names[index]);
    if (found == actual.end() || !std::isfinite(found->second) ||
      !std::isfinite(planned[index])) return INFINITY;
    result = std::max(result, std::abs(found->second - planned[index]));
  }
  return result;
}

}  // namespace

JakaNativeCartesianExecutor::JakaNativeCartesianExecutor(
  rclcpp::Node::SharedPtr node, JakaNativeCartesianExecutorConfig config)
: node_(std::move(node)),
  logger_(node_ ? node_->get_logger() : rclcpp::get_logger("jaka_native_cartesian")),
  config_(std::move(config))
{
  if (!node_ || config_.action_name.empty() ||
    config_.rapid_rate_service.empty() || config_.joint_state_topic.empty() ||
    config_.controller_frame.empty() ||
    !finite_positive(config_.joint_state_timeout) ||
    !finite_positive(config_.action_server_timeout) ||
    !finite_positive(config_.result_grace_period) ||
    !finite_positive(config_.maximum_start_error) ||
    !finite_positive(config_.maximum_linear_speed_mm_s) ||
    !finite_positive(config_.maximum_linear_acceleration_mm_s2) ||
    !finite_positive(config_.orientation_speed_rad_s) ||
    !finite_positive(config_.orientation_acceleration_rad_s2) ||
    !finite_positive(config_.translation_tolerance_mm) ||
    !finite_positive(config_.rotation_tolerance_rad))
  {
    throw std::invalid_argument("JAKA 原生笛卡尔执行器配置无效");
  }
  client_ = rclcpp_action::create_client<Action>(node_, config_.action_name);
  rapid_rate_client_ = node_->create_client<jaka_msgs::srv::GetRapidRate>(
    config_.rapid_rate_service);
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_state_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_state_ = *message;
      state_received_at_ = std::chrono::steady_clock::now();
    });
}

massage_motion::ExecutionResult JakaNativeCartesianExecutor::execute(
  const massage_motion::ExecutionRequest & request)
{
  std::unique_lock<std::mutex> execution_lock(execution_mutex_, std::try_to_lock);
  if (!execution_lock.owns_lock())
  {
    return failure(
      massage_motion::ExecutionError::kRejected,
      "JAKA 原生笛卡尔执行器已有活动请求");
  }
  if (!request.has_motion_semantics ||
    (request.motion_type != massage_motion::MotionType::kLin &&
    request.motion_type != massage_motion::MotionType::kCirc) ||
    !finite_positive(request.velocity_scale) || request.velocity_scale > 1.0 ||
    !finite_positive(request.acceleration_scale) ||
    request.acceleration_scale > 1.0 || !finite_positive(request.timeout))
  {
    return failure(
      massage_motion::ExecutionError::kInvalidRequest,
      "原生笛卡尔执行请求缺少 LIN/CIRC 语义或速度参数无效");
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
        "原生笛卡尔执行前没有新鲜的 /joint_states");
    }
    current = latest_state_;
  }
  const double maximum_start_error = start_error(
    request.robot_trajectory, current);
  if (!std::isfinite(maximum_start_error) ||
    maximum_start_error > config_.maximum_start_error)
  {
    return failure(
      massage_motion::ExecutionError::kRejected,
      "NATIVE CARTESIAN START GATE: REJECTED: 规划起点与当前关节状态不一致");
  }

  Action::Goal goal;
  goal.request_id = request.request_id;
  goal.motion_type = request.motion_type == massage_motion::MotionType::kLin ?
    goal.LINEAR : goal.CIRCULAR;
  try
  {
    if (request.motion_type == massage_motion::MotionType::kLin)
    {
      const auto * target = std::get_if<massage_motion::PoseTarget>(
        &request.motion_target);
      if (!target) throw std::invalid_argument("LIN 缺少 PoseTarget");
      goal.target_pose = sdk_pose(target->pose, config_.controller_frame);
      goal.has_midpoint = false;
    }
    else
    {
      const auto * target = std::get_if<massage_motion::CircularTarget>(
        &request.motion_target);
      if (!target) throw std::invalid_argument("CIRC 缺少 CircularTarget");
      goal.target_pose = sdk_pose(target->goal_pose, config_.controller_frame);
      goal.midpoint_pose = sdk_pose(
        target->interim_pose, config_.controller_frame);
      goal.has_midpoint = true;
    }
  }
  catch (const std::exception & exception)
  {
    return failure(
      massage_motion::ExecutionError::kInvalidRequest, exception.what());
  }

  if (!rapid_rate_client_->wait_for_service(
      std::chrono::duration<double>(config_.action_server_timeout)))
  {
    return failure(
      massage_motion::ExecutionError::kBackendUnavailable,
      "等待 /jaka_driver/get_rapid_rate 服务超时");
  }
  auto rapid_future = rapid_rate_client_->async_send_request(
    std::make_shared<jaka_msgs::srv::GetRapidRate::Request>());
  if (rapid_future.wait_for(std::chrono::duration<double>(
      config_.action_server_timeout)) != std::future_status::ready)
  {
    return failure(
      massage_motion::ExecutionError::kTimeout,
      "读取 JAKA 全局速度倍率超时", 0,
      massage_motion::ExecutionStatus::kTimedOut);
  }
  const auto rapid = rapid_future.get();
  if (!rapid || !rapid->success || !finite_positive(rapid->rapid_rate) ||
    rapid->rapid_rate > 1.0)
  {
    return failure(
      massage_motion::ExecutionError::kRejected,
      rapid ? rapid->message : "JAKA 全局速度倍率响应无效",
      rapid ? rapid->error_code : 0);
  }

  const double requested_effective_speed =
    finite_positive(request.desired_cartesian_speed_m_s) ?
    request.desired_cartesian_speed_m_s * 1000.0 :
    config_.maximum_linear_speed_mm_s * request.velocity_scale;
  const double desired_effective_speed = std::min(
    requested_effective_speed, config_.maximum_linear_speed_mm_s);
  const double desired_effective_acceleration =
    config_.maximum_linear_acceleration_mm_s2 * request.acceleration_scale;
  goal.speed = std::min(desired_effective_speed / rapid->rapid_rate, 500.0);
  goal.acceleration = std::min(
    desired_effective_acceleration / rapid->rapid_rate, 2000.0);
  goal.orientation_speed = config_.orientation_speed_rad_s;
  goal.orientation_acceleration = config_.orientation_acceleration_rad_s2;
  goal.translation_tolerance = config_.translation_tolerance_mm;
  goal.rotation_tolerance = config_.rotation_tolerance_rad;
  // The SDK speed and acceleration above are already compensated for the
  // controller-wide rapid rate, so the planned execution window remains in
  // wall-clock seconds. Dividing it by rapid_rate again would make failures
  // take up to 5x longer to surface at the commonly used 20% override.
  goal.timeout = request.timeout;
  RCLCPP_INFO(
    logger_,
    "NATIVE CARTESIAN GATE: PASS: request=%s, type=%s, start_error=%.9f rad, programmed_speed=%.3f mm/s, rapid_rate=%.3f, effective_speed=%.3f mm/s, timeout=%.3f s",
    request.request_id.c_str(),
    request.motion_type == massage_motion::MotionType::kLin ? "LIN" : "CIRC",
    maximum_start_error, goal.speed, rapid->rapid_rate,
    goal.speed * rapid->rapid_rate, goal.timeout);

  if (!client_->wait_for_action_server(
      std::chrono::duration<double>(config_.action_server_timeout)))
  {
    return failure(
      massage_motion::ExecutionError::kBackendUnavailable,
      "等待 /jaka_driver/execute_cartesian_move Action 超时");
  }
  auto goal_future = client_->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::duration<double>(
      config_.action_server_timeout)) != std::future_status::ready)
  {
    client_->async_cancel_all_goals();
    return failure(
      massage_motion::ExecutionError::kTimeout,
      "原生笛卡尔 Goal 响应超时", 0,
      massage_motion::ExecutionStatus::kTimedOut);
  }
  const auto goal_handle = goal_future.get();
  if (!goal_handle)
  {
    return failure(
      massage_motion::ExecutionError::kRejected,
      "原生笛卡尔 Goal 被驱动拒绝");
  }
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_ = goal_handle;
  }
  status_.store(massage_motion::ExecutionStatus::kExecuting);
  auto result_future = client_->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::duration<double>(
      goal.timeout + config_.result_grace_period)) != std::future_status::ready)
  {
    client_->async_cancel_goal(goal_handle);
    status_.store(massage_motion::ExecutionStatus::kTimedOut);
    return failure(
      massage_motion::ExecutionError::kTimeout,
      "等待原生笛卡尔运动终态超时，已请求取消", 0,
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
      wrapped.result ? wrapped.result->message : "原生笛卡尔运动已取消",
      wrapped.result ? wrapped.result->sdk_error_code : 0,
      massage_motion::ExecutionStatus::kCanceled);
  }
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped.result || !wrapped.result->success)
  {
    status_.store(massage_motion::ExecutionStatus::kFailed);
    return failure(
      massage_motion::ExecutionError::kExecutionFailed,
      wrapped.result ? wrapped.result->message : "原生笛卡尔 Action 失败",
      wrapped.result ? wrapped.result->sdk_error_code : 0);
  }
  status_.store(massage_motion::ExecutionStatus::kSucceeded);
  return {
    true, massage_motion::ExecutionError::kNone,
    wrapped.result->sdk_error_code, wrapped.result->message,
    massage_motion::ExecutionStatus::kSucceeded};
}

bool JakaNativeCartesianExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!active_goal_) return false;
  client_->async_cancel_goal(active_goal_);
  return true;
}

massage_motion::ExecutionStatus JakaNativeCartesianExecutor::status() const
{
  return status_.load();
}

}  // namespace massage_jaka
