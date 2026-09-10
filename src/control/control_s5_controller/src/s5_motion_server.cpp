#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_jaka_msgs/msg/joint_move_status.hpp"
#include "control_jaka_msgs/msg/robot_diagnostic.hpp"
#include "control_jaka_msgs/srv/jog_joint.hpp"
#include "control_jaka_msgs/srv/move_joint.hpp"
#include "control_jaka_msgs/srv/set_rapid_rate.hpp"
#include "control_jaka_sdk_vendor/JAKAZuRobot.h"
#include "control_s5_controller/robot_shutdown_utils.hpp"
#include "control_s5_controller/s5_joint_utils.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using JointMoveStatus = control_jaka_msgs::msg::JointMoveStatus;
using JointMoveStatusPublisher = rclcpp::Publisher<JointMoveStatus>;

constexpr std::size_t kJointCount = 6;
constexpr double kPositionTolerance = 0.2 * 3.14159265358979323846 / 180.0;
constexpr double kProgressEpsilon = 0.05 * 3.14159265358979323846 / 180.0;
constexpr double kMaximumJointMoveSpeedRadS = 3.14;
constexpr double kMaximumJointMoveAccelerationRadS2 = 90.0;
constexpr auto kJointMoveStatusPeriod = std::chrono::milliseconds(200);
constexpr auto kJointMoveStallTimeout = std::chrono::seconds(10);
const std::array<std::string, kJointCount> kJointNames = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

JAKAZuRobot robot;
std::mutex sdk_mutex;
std::mutex worker_mutex;
std::mutex motion_state_mutex;
std::mutex shutdown_mutex;
std::thread goal_worker;
std::atomic<bool> goal_active{false};
std::atomic<bool> cancel_requested{false};
std::atomic<bool> jog_active{false};
std::atomic<bool> jog_cancel_requested{false};
std::atomic<bool> joint_move_active{false};
std::atomic<bool> joint_move_cancel_requested{false};
std::atomic<bool> shutting_down{false};
std::atomic<double> joint_move_max_speed_rad_s{1.57};
std::atomic<double> joint_move_max_acceleration_rad_s2{1.57};
bool sdk_logged_in = false;
control_s5_controller::RobotShutdownState robot_shutdown_state;
bool have_previous_enabled = false;
bool previous_enabled = false;

const std::map<int, std::string> kErrors = {
  {2, "ERR_FUCTION_CALL_ERROR"}, {-1, "ERR_INVALID_HANDLER"},
  {-2, "ERR_INVALID_PARAMETER"}, {-3, "ERR_COMMUNICATION_ERR"},
  {-4, "ERR_KINE_INVERSE_ERR"}, {-5, "ERR_EMERGENCY_PRESSED"},
  {-6, "ERR_NOT_POWERED"}, {-7, "ERR_NOT_ENABLED"},
  {-8, "ERR_DISABLE_SERVOMODE"}, {-9, "ERR_NOT_OFF_ENABLE"},
  {-10, "ERR_PROGRAM_IS_RUNNING"}, {-11, "ERR_CANNOT_OPEN_FILE"},
  {-12, "ERR_MOTION_ABNORMAL"}};

std::string errorText(int code)
{
  const auto found = kErrors.find(code);
  return found == kErrors.end() ? "SDK error " + std::to_string(code) : found->second;
}

std::shared_ptr<FollowJointTrajectory::Result> makeResult(
  int32_t error_code, const std::string & message)
{
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = error_code;
  result->error_string = message;
  return result;
}

int64_t durationNanoseconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<int64_t>(duration.sec) * 1000000000LL + duration.nanosec;
}

bool validateGoal(const FollowJointTrajectory::Goal & goal, std::string & reason)
{
  const auto & trajectory = goal.trajectory;
  if (trajectory.joint_names.size() != kJointCount) {
    reason = "trajectory must contain exactly six joint names";
    return false;
  }
  std::unordered_set<std::string> names(
    trajectory.joint_names.begin(), trajectory.joint_names.end());
  if (names.size() != kJointCount ||
    !std::all_of(kJointNames.begin(), kJointNames.end(),
      [&names](const std::string & name) {return names.count(name) == 1;}))
  {
    reason = "trajectory joint names do not match joint_1 through joint_6";
    return false;
  }
  if (trajectory.points.empty()) {
    reason = "trajectory has no points";
    return false;
  }

  int64_t previous_time = -1;
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != kJointCount ||
      !std::all_of(point.positions.begin(), point.positions.end(),
        [](double value) {return std::isfinite(value);}))
    {
      reason = "every trajectory point must contain six finite positions";
      return false;
    }
    const int64_t point_time = durationNanoseconds(point.time_from_start);
    if (point_time < 0 || point_time <= previous_time) {
      reason = "trajectory point times must be non-negative and strictly increasing";
      return false;
    }
    previous_time = point_time;
  }
  return true;
}

std::array<std::size_t, kJointCount> trajectoryOrder(
  const std::vector<std::string> & trajectory_names)
{
  std::unordered_map<std::string, std::size_t> by_name;
  for (std::size_t index = 0; index < trajectory_names.size(); ++index) {
    by_name.emplace(trajectory_names[index], index);
  }
  std::array<std::size_t, kJointCount> order{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    order[index] = by_name.at(kJointNames[index]);
  }
  return order;
}

int readJointPosition(JointValue & position)
{
  std::lock_guard<std::mutex> lock(sdk_mutex);
  return robot.get_joint_position(&position);
}

void disableServoMode()
{
  std::lock_guard<std::mutex> lock(sdk_mutex);
  const int result = robot.servo_move_enable(false);
  if (result != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"),
      "Failed to disable servo mode: %s", errorText(result).c_str());
  }
}

bool targetReached(const JointValue & target, bool & sdk_ok)
{
  JointValue current{};
  const int result = readJointPosition(current);
  if (result != 0) {
    sdk_ok = false;
    RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"),
      "Failed to read joint position: %s", errorText(result).c_str());
    return false;
  }
  sdk_ok = true;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (std::abs(current.jVal[index] - target.jVal[index]) > kPositionTolerance) {
      return false;
    }
  }
  return true;
}

control_s5_controller::s5::JointPositions jointPositions(const JointValue & value)
{
  control_s5_controller::s5::JointPositions positions{};
  std::copy(std::begin(value.jVal), std::end(value.jVal), positions.begin());
  return positions;
}

void abortSdkMotion()
{
  std::lock_guard<std::mutex> lock(sdk_mutex);
  const int result = robot.motion_abort();
  if (result != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"),
      "Failed to abort motion: %s", errorText(result).c_str());
  }
}

void finishJointMove()
{
  joint_move_cancel_requested.store(false);
  std::lock_guard<std::mutex> lock(motion_state_mutex);
  joint_move_active.store(false);
}

void handleSetRapidRate(
  const std::shared_ptr<control_jaka_msgs::srv::SetRapidRate::Request> request,
  std::shared_ptr<control_jaka_msgs::srv::SetRapidRate::Response> response)
{
  response->applied_rapid_rate = 0.0;
  if (!control_s5_controller::s5::validMotionScaling(request->rapid_rate)) {
    response->success = false;
    response->error_code = -2;
    response->message = "rapid_rate must be finite and in [0.01, 1.0]";
    return;
  }

  std::lock_guard<std::mutex> state_lock(motion_state_mutex);
  if (shutting_down.load() || goal_active.load() || joint_move_active.load() || jog_active.load()) {
    response->success = false;
    response->error_code = -10;
    response->message = shutting_down.load() ?
      "robot shutdown is in progress" : "rapid_rate can only be changed while the robot is idle";
    return;
  }

  double applied_rapid_rate = 0.0;
  int sdk_result;
  {
    std::lock_guard<std::mutex> sdk_lock(sdk_mutex);
    sdk_result = robot.set_rapidrate(request->rapid_rate);
    if (sdk_result == 0) {
      sdk_result = robot.get_rapidrate(&applied_rapid_rate);
    }
  }
  response->applied_rapid_rate = applied_rapid_rate;
  response->success = sdk_result == 0 &&
    control_s5_controller::s5::validRapidRate(applied_rapid_rate);
  response->error_code = response->success ? 0 : (sdk_result == 0 ? -2 : sdk_result);
  response->message = response->success ?
    "rapid_rate applied" :
    (sdk_result != 0 ? "failed to set or read rapid_rate: " + errorText(sdk_result) :
    "controller returned an invalid rapid_rate");

  if (response->success) {
    RCLCPP_INFO(
      rclcpp::get_logger("control_s5_motion_server"),
      "Controller rapidrate set to %.1f%% (requested %.1f%%)",
      applied_rapid_rate * 100.0, request->rapid_rate * 100.0);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"), "%s", response->message.c_str());
  }
}

void publishJointMoveStatus(
  const JointMoveStatusPublisher::SharedPtr & publisher,
  const rclcpp::Clock::SharedPtr & clock,
  uint8_t state, double velocity_scaling, double acceleration_scaling,
  double command_speed, double command_acceleration, double rapidrate,
  double remaining_error,
  const std::chrono::steady_clock::time_point & started_at,
  const std::chrono::steady_clock::time_point & last_progress_at,
  const std::chrono::steady_clock::time_point & hard_deadline,
  int error_code, const std::string & message)
{
  const auto now = std::chrono::steady_clock::now();
  JointMoveStatus status;
  status.header.stamp = clock->now();
  status.state = state;
  status.velocity_scaling = velocity_scaling;
  status.acceleration_scaling = acceleration_scaling;
  status.command_speed_rad_s = command_speed;
  status.command_acceleration_rad_s2 = command_acceleration;
  status.controller_rapidrate = rapidrate;
  status.estimated_effective_speed_rad_s =
    control_s5_controller::s5::effectiveJointSpeed(command_speed, rapidrate);
  status.remaining_error_rad = remaining_error;
  status.elapsed_s = std::max(
    0.0, std::chrono::duration<double>(now - started_at).count());
  status.stall_elapsed_s = std::max(
    0.0, std::chrono::duration<double>(now - last_progress_at).count());
  status.hard_timeout_remaining_s = std::max(
    0.0, std::chrono::duration<double>(hard_deadline - now).count());
  status.error_code = error_code;
  status.message = message;
  publisher->publish(status);
}

void handleMoveJoint(
  const std::shared_ptr<control_jaka_msgs::srv::MoveJoint::Request> request,
  std::shared_ptr<control_jaka_msgs::srv::MoveJoint::Response> response,
  const JointMoveStatusPublisher::SharedPtr & status_publisher,
  const rclcpp::Clock::SharedPtr & clock)
{
  control_s5_controller::s5::JointPositions requested_positions{};
  std::copy(request->positions.begin(), request->positions.end(), requested_positions.begin());
  if (!control_s5_controller::s5::targetWithinLimits(requested_positions)) {
    response->success = false;
    response->error_code = -2;
    response->message = "joint target is non-finite or outside the S5 limits";
    return;
  }
  if (!control_s5_controller::s5::validMotionScaling(request->velocity_scaling) ||
    !control_s5_controller::s5::validMotionScaling(request->acceleration_scaling))
  {
    response->success = false;
    response->error_code = -2;
    response->message = "velocity and acceleration scaling must be in [0.01, 1.0]";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(motion_state_mutex);
    if (shutting_down.load() || goal_active.load() || jog_active.load() ||
      joint_move_active.load())
    {
      response->success = false;
      response->error_code = -10;
      response->message = shutting_down.load() ?
        "robot shutdown is in progress" : "another robot motion is active";
      return;
    }
    joint_move_active.store(true);
    joint_move_cancel_requested.store(false);
  }

  const auto started_at = std::chrono::steady_clock::now();
  auto last_progress_at = started_at;
  JointValue current{};
  double rapidrate = 0.0;
  int sdk_result;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    sdk_result = robot.get_joint_position(&current);
    if (sdk_result == 0) {
      sdk_result = robot.get_rapidrate(&rapidrate);
    }
  }
  const double speed = control_s5_controller::s5::scaledMotionLimit(
    joint_move_max_speed_rad_s.load(), request->velocity_scaling);
  const double acceleration = control_s5_controller::s5::scaledMotionLimit(
    joint_move_max_acceleration_rad_s2.load(), request->acceleration_scaling);
  if (sdk_result != 0 || speed <= 0.0 || acceleration <= 0.0 ||
    !control_s5_controller::s5::validRapidRate(rapidrate))
  {
    response->success = false;
    response->error_code = sdk_result == 0 ? -2 : sdk_result;
    response->message = sdk_result != 0 ?
      "failed to read initial joint position or rapidrate: " + errorText(sdk_result) :
      "joint_move speed, acceleration, or controller rapidrate is invalid";
    publishJointMoveStatus(
      status_publisher, clock, JointMoveStatus::STATE_FAILED,
      request->velocity_scaling, request->acceleration_scaling,
      speed, acceleration, rapidrate, 0.0, started_at,
      last_progress_at, started_at, response->error_code, response->message);
    RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"), "%s", response->message.c_str());
    finishJointMove();
    return;
  }
  double remaining_error = control_s5_controller::s5::maximumPositionError(
    jointPositions(current), requested_positions);
  auto hard_deadline = started_at + control_s5_controller::s5::jointMoveHardTimeout(
    remaining_error, speed, acceleration, rapidrate);
  if (control_s5_controller::s5::positionsNear(
      jointPositions(current), requested_positions, kPositionTolerance))
  {
    response->success = true;
    response->error_code = 0;
    response->message = "joint_move target already reached";
    publishJointMoveStatus(
      status_publisher, clock, JointMoveStatus::STATE_SUCCEEDED,
      request->velocity_scaling, request->acceleration_scaling,
      speed, acceleration, rapidrate, remaining_error, started_at,
      last_progress_at, hard_deadline, 0, response->message);
    RCLCPP_INFO(rclcpp::get_logger("control_s5_motion_server"), "%s", response->message.c_str());
    finishJointMove();
    return;
  }

  publishJointMoveStatus(
    status_publisher, clock, JointMoveStatus::STATE_STARTING,
    request->velocity_scaling, request->acceleration_scaling,
    speed, acceleration, rapidrate, remaining_error, started_at,
    last_progress_at, hard_deadline, 0, "joint_move starting");

  JointValue target{};
  std::copy(requested_positions.begin(), requested_positions.end(), std::begin(target.jVal));
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    sdk_result = robot.servo_move_enable(false);
    if (sdk_result == 0) {
      sdk_result = robot.joint_move(
        &target, MoveMode::ABS, FALSE, speed, acceleration, 0.0, nullptr);
    }
  }
  if (sdk_result != 0) {
    response->success = false;
    response->error_code = sdk_result;
    response->message = "joint_move failed: " + errorText(sdk_result);
    publishJointMoveStatus(
      status_publisher, clock, JointMoveStatus::STATE_FAILED,
      request->velocity_scaling, request->acceleration_scaling,
      speed, acceleration, rapidrate, remaining_error, started_at,
      last_progress_at, hard_deadline, sdk_result, response->message);
    RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"), "%s", response->message.c_str());
    finishJointMove();
    return;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("control_s5_motion_server"),
    "joint_move start: velocity=%.1f%% acceleration=%.1f%% speed=%.6f rad/s "
    "acceleration=%.6f rad/s^2 rapidrate=%.1f%% effective_speed=%.6f rad/s "
    "hard_timeout=%.1f s",
    request->velocity_scaling * 100.0, request->acceleration_scaling * 100.0,
    speed, acceleration, rapidrate * 100.0,
    control_s5_controller::s5::effectiveJointSpeed(speed, rapidrate),
    std::chrono::duration<double>(hard_deadline - started_at).count());

  double best_error = remaining_error;
  double minimum_rapidrate = rapidrate;
  auto next_status_at = started_at;
  while (rclcpp::ok()) {
    const auto now = std::chrono::steady_clock::now();
    if (joint_move_cancel_requested.load() || shutting_down.load()) {
      response->success = false;
      response->error_code = 0;
      response->message = "joint_move canceled";
      publishJointMoveStatus(
        status_publisher, clock, JointMoveStatus::STATE_CANCELED,
        request->velocity_scaling, request->acceleration_scaling,
        speed, acceleration, rapidrate, remaining_error, started_at,
        last_progress_at, hard_deadline, 0, response->message);
      RCLCPP_WARN(rclcpp::get_logger("control_s5_motion_server"),
        "joint_move canceled after %.1f s",
        std::chrono::duration<double>(now - started_at).count());
      finishJointMove();
      return;
    }

    const bool sample_status = now >= next_status_at;
    MotionStatus status{};
    BOOL on_soft_limit = FALSE;
    JointValue sampled_position{};
    double sampled_rapidrate = rapidrate;
    std::string failed_operation;
    {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      sdk_result = robot.get_motion_status(&status);
      failed_operation = "get_motion_status";
      if (sdk_result == 0) {
        sdk_result = robot.is_on_limit(&on_soft_limit);
        failed_operation = "is_on_limit";
      }
      if (sdk_result == 0 && sample_status) {
        sdk_result = robot.get_joint_position(&sampled_position);
        failed_operation = "get_joint_position";
      }
      if (sdk_result == 0 && sample_status) {
        sdk_result = robot.get_rapidrate(&sampled_rapidrate);
        failed_operation = "get_rapidrate";
      }
    }
    if (sdk_result != 0 || status.err_add_line != FALSE || on_soft_limit != FALSE) {
      abortSdkMotion();
      response->success = false;
      response->error_code = sdk_result == 0 ? -12 : sdk_result;
      response->message = sdk_result != 0 ?
        "failed to monitor joint_move (" + failed_operation + "): " + errorText(sdk_result) :
        (on_soft_limit != FALSE ? "joint_move stopped at a soft limit" :
        "controller rejected the joint_move command");
      publishJointMoveStatus(
        status_publisher, clock, JointMoveStatus::STATE_FAILED,
        request->velocity_scaling, request->acceleration_scaling,
        speed, acceleration, rapidrate, remaining_error, started_at,
        last_progress_at, hard_deadline, response->error_code, response->message);
      RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"), "%s", response->message.c_str());
      finishJointMove();
      return;
    }

    if (sample_status) {
      remaining_error = control_s5_controller::s5::maximumPositionError(
        jointPositions(sampled_position), requested_positions);
      if (!std::isfinite(remaining_error)) {
        abortSdkMotion();
        response->success = false;
        response->error_code = -3;
        response->message = "joint_move returned a non-finite joint position";
        publishJointMoveStatus(
          status_publisher, clock, JointMoveStatus::STATE_FAILED,
          request->velocity_scaling, request->acceleration_scaling,
          speed, acceleration, rapidrate, remaining_error, started_at,
          last_progress_at, hard_deadline, response->error_code, response->message);
        RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"), "%s", response->message.c_str());
        finishJointMove();
        return;
      }
      if (control_s5_controller::s5::madeJointMoveProgress(
          best_error, remaining_error, kProgressEpsilon))
      {
        best_error = remaining_error;
        last_progress_at = now;
      }
      rapidrate = sampled_rapidrate;
      if (control_s5_controller::s5::validRapidRate(rapidrate) && rapidrate < minimum_rapidrate) {
        minimum_rapidrate = rapidrate;
        hard_deadline = std::max(
          hard_deadline,
          now + control_s5_controller::s5::jointMoveHardTimeout(
            remaining_error, speed, acceleration, rapidrate));
      }
      next_status_at = now + kJointMoveStatusPeriod;

      if (control_s5_controller::s5::jointMoveComplete(
          status.inpos != FALSE, status.queue, status.active_queue,
          remaining_error <= kPositionTolerance))
      {
        response->success = true;
        response->error_code = 0;
        response->message = "joint_move target reached";
        publishJointMoveStatus(
          status_publisher, clock, JointMoveStatus::STATE_SUCCEEDED,
          request->velocity_scaling, request->acceleration_scaling,
          speed, acceleration, rapidrate, remaining_error, started_at,
          last_progress_at, hard_deadline, 0, response->message);
        RCLCPP_INFO(rclcpp::get_logger("control_s5_motion_server"),
          "joint_move completed in %.1f s",
          std::chrono::duration<double>(now - started_at).count());
        finishJointMove();
        return;
      }

      publishJointMoveStatus(
        status_publisher, clock, JointMoveStatus::STATE_RUNNING,
        request->velocity_scaling, request->acceleration_scaling,
        speed, acceleration, rapidrate, remaining_error, started_at,
        last_progress_at, hard_deadline, 0,
        status.paused != FALSE ? "joint_move paused" : "joint_move running");
    }

    if (control_s5_controller::s5::jointMoveStalled(
        last_progress_at, now, kJointMoveStallTimeout))
    {
      abortSdkMotion();
      response->success = false;
      response->error_code = -12;
      response->message = "joint_move stalled for 10 seconds";
      publishJointMoveStatus(
        status_publisher, clock, JointMoveStatus::STATE_STALLED,
        request->velocity_scaling, request->acceleration_scaling,
        speed, acceleration, rapidrate, remaining_error, started_at,
        last_progress_at, hard_deadline, response->error_code, response->message);
      RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"),
        "%s (remaining_error=%.6f rad rapidrate=%.1f%%)",
        response->message.c_str(), remaining_error, rapidrate * 100.0);
      finishJointMove();
      return;
    }

    if (now >= hard_deadline) {
      abortSdkMotion();
      response->success = false;
      response->error_code = -12;
      response->message = "joint_move exceeded the dynamic hard timeout";
      publishJointMoveStatus(
        status_publisher, clock, JointMoveStatus::STATE_TIMED_OUT,
        request->velocity_scaling, request->acceleration_scaling,
        speed, acceleration, rapidrate, remaining_error, started_at,
        last_progress_at, hard_deadline, response->error_code, response->message);
      RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"),
        "%s after %.1f s (remaining_error=%.6f rad rapidrate=%.1f%%)",
        response->message.c_str(), std::chrono::duration<double>(now - started_at).count(),
        remaining_error, rapidrate * 100.0);
      finishJointMove();
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  abortSdkMotion();
  response->success = false;
  response->error_code = 0;
  response->message = "joint_move interrupted by ROS shutdown";
  publishJointMoveStatus(
    status_publisher, clock, JointMoveStatus::STATE_CANCELED,
    request->velocity_scaling, request->acceleration_scaling,
    speed, acceleration, rapidrate, remaining_error, started_at,
    last_progress_at, hard_deadline, 0, response->message);
  RCLCPP_WARN(rclcpp::get_logger("control_s5_motion_server"), "%s", response->message.c_str());
  finishJointMove();
}

void finishGoal()
{
  disableServoMode();
  cancel_requested.store(false);
  std::lock_guard<std::mutex> lock(motion_state_mutex);
  goal_active.store(false);
}

void executeGoal(const std::shared_ptr<GoalHandle> & goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto order = trajectoryOrder(goal->trajectory.joint_names);
  JointValue target{};

  int enable_result;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    enable_result = robot.servo_move_enable(true);
  }
  if (enable_result != 0) {
    goal_handle->abort(makeResult(
      FollowJointTrajectory::Result::INVALID_GOAL,
      "failed to enable servo mode: " + errorText(enable_result)));
    finishGoal();
    return;
  }

  int64_t previous_time_ns = 0;
  const auto & points = goal->trajectory.points;
  const std::size_t first_point =
    points.size() > 1 && durationNanoseconds(points.front().time_from_start) == 0 ? 1 : 0;

  for (std::size_t point_index = first_point; point_index < points.size(); ++point_index) {
    if (cancel_requested.load() || goal_handle->is_canceling()) {
      goal_handle->canceled(makeResult(FollowJointTrajectory::Result::SUCCESSFUL, "motion canceled"));
      finishGoal();
      return;
    }

    const auto & point = points[point_index];
    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index) {
      target.jVal[joint_index] = point.positions[order[joint_index]];
    }
    const int64_t point_time_ns = durationNanoseconds(point.time_from_start);
    const double delta_seconds = (point_time_ns - previous_time_ns) / 1.0e9;
    previous_time_ns = point_time_ns;
    const unsigned int step_count = static_cast<unsigned int>(
      std::max(1.0, std::floor(delta_seconds / 0.008)));

    int sdk_result;
    {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      sdk_result = robot.servo_j(&target, MoveMode::ABS, step_count);
    }
    if (sdk_result != 0) {
      goal_handle->abort(makeResult(
        FollowJointTrajectory::Result::INVALID_GOAL,
        "servo_j failed: " + errorText(sdk_result)));
      finishGoal();
      return;
    }
  }

  while (rclcpp::ok()) {
    if (cancel_requested.load() || goal_handle->is_canceling()) {
      goal_handle->canceled(makeResult(FollowJointTrajectory::Result::SUCCESSFUL, "motion canceled"));
      finishGoal();
      return;
    }
    bool sdk_ok = false;
    if (targetReached(target, sdk_ok)) {
      goal_handle->succeed(makeResult(FollowJointTrajectory::Result::SUCCESSFUL, "target reached"));
      finishGoal();
      return;
    }
    if (!sdk_ok) {
      goal_handle->abort(makeResult(
        FollowJointTrajectory::Result::INVALID_GOAL, "failed to monitor final position"));
      finishGoal();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  finishGoal();
}

void publishJointState(
  const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr & publisher,
  const rclcpp::Clock::SharedPtr & clock)
{
  if (shutting_down.load()) {
    return;
  }
  JointValue position{};
  const int result = readJointPosition(position);
  if (result != 0) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("control_s5_motion_server"), *clock, 2000,
      "Failed to publish joint state: %s", errorText(result).c_str());
    return;
  }
  sensor_msgs::msg::JointState message;
  message.header.stamp = clock->now();
  message.name.assign(kJointNames.begin(), kJointNames.end());
  for (std::size_t index = 0; index < kJointCount; ++index) {
    message.position.push_back(position.jVal[index]);
  }
  publisher->publish(message);
}

std::string disableReason(
  const RobotStatus_simple & status, bool in_collision, bool emergency_stop, bool on_soft_limit)
{
  if (emergency_stop) {
    return "emergency_stop";
  }
  if (in_collision) {
    return "collision_protection";
  }
  if (on_soft_limit) {
    return "soft_limit";
  }
  if (status.errcode != 0) {
    const std::string message(status.errmsg, strnlen(status.errmsg, sizeof(status.errmsg)));
    return message.empty() ? "controller_error_" + std::to_string(status.errcode) :
           "controller_error_" + std::to_string(status.errcode) + ": " + message;
  }
  return "unknown_or_external_disable";
}

void publishRobotDiagnostic(
  const rclcpp::Publisher<control_jaka_msgs::msg::RobotDiagnostic>::SharedPtr & publisher,
  const rclcpp::Clock::SharedPtr & clock)
{
  if (shutting_down.load()) {
    return;
  }
  RobotStatus_simple status{};
  BOOL in_servo = FALSE;
  BOOL in_collision = FALSE;
  BOOL emergency_stop = FALSE;
  BOOL on_soft_limit = FALSE;
  int status_result;
  int servo_result;
  int collision_result;
  int emergency_stop_result;
  int soft_limit_result;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    status_result = robot.get_robot_status_simple(&status);
    servo_result = robot.is_in_servomove(&in_servo);
    collision_result = robot.is_in_collision(&in_collision);
    emergency_stop_result = robot.is_in_estop(&emergency_stop);
    soft_limit_result = robot.is_on_limit(&on_soft_limit);
  }

  control_jaka_msgs::msg::RobotDiagnostic message;
  message.header.stamp = clock->now();
  message.robot_status_result = status_result;
  message.servo_status_result = servo_result;
  message.collision_status_result = collision_result;
  message.emergency_stop_status_result = emergency_stop_result;
  message.soft_limit_status_result = soft_limit_result;
  message.error_code = status.errcode;
  message.error_message.assign(status.errmsg, strnlen(status.errmsg, sizeof(status.errmsg)));
  message.powered_on = status_result == 0 && status.powered_on != 0;
  message.enabled = status_result == 0 && status.enabled != 0;
  message.in_servo_mode = servo_result == 0 && in_servo != FALSE;
  message.in_collision = collision_result == 0 && in_collision != FALSE;
  message.emergency_stop = emergency_stop_result == 0 && emergency_stop != FALSE;
  message.on_soft_limit = soft_limit_result == 0 && on_soft_limit != FALSE;
  message.motion_goal_active = goal_active.load() || joint_move_active.load();

  if (status_result != 0 || servo_result != 0 || collision_result != 0 ||
    emergency_stop_result != 0 || soft_limit_result != 0)
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("control_s5_motion_server"), *clock, 2000,
      "Failed to query robot diagnostics: status=%d servo=%d collision=%d estop=%d limit=%d",
      status_result, servo_result, collision_result, emergency_stop_result, soft_limit_result);
  }

  if (status_result == 0) {
    message.disable_event = have_previous_enabled && previous_enabled && !message.enabled;
    if (message.disable_event) {
      message.disable_reason = disableReason(
        status, message.in_collision, message.emergency_stop, message.on_soft_limit);
      RCLCPP_ERROR(
        rclcpp::get_logger("control_s5_motion_server"),
        "Robot became disabled: reason=%s error_code=%d error_message=%s",
        message.disable_reason.c_str(), message.error_code, message.error_message.c_str());
    }
    previous_enabled = message.enabled;
    have_previous_enabled = true;
  }
  publisher->publish(message);
}

void stopActiveMotion()
{
  cancel_requested.store(true);
  joint_move_cancel_requested.store(true);
  jog_cancel_requested.store(true);
  std::lock_guard<std::mutex> lock(sdk_mutex);
  if (jog_active.load()) {
    robot.jog_stop(-1);
  }
  robot.motion_abort();
}

void handleStopMotion(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const bool had_motion = goal_active.load() || joint_move_active.load() || jog_active.load();
  if (!had_motion) {
    response->success = true;
    response->message = "no robot motion is active";
    return;
  }
  cancel_requested.store(true);
  joint_move_cancel_requested.store(true);
  jog_cancel_requested.store(true);
  int first_error = 0;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    if (jog_active.load()) {
      first_error = robot.jog_stop(-1);
    }
    const int abort_result = robot.motion_abort();
    if (first_error == 0) {
      first_error = abort_result;
    }
  }
  response->success = first_error == 0;
  response->message = first_error == 0 ? "robot motion stop requested" : errorText(first_error);
}

int stopMotionForShutdown()
{
  cancel_requested.store(true);
  joint_move_cancel_requested.store(true);
  jog_cancel_requested.store(true);
  const bool had_goal = goal_active.load();
  const bool had_joint_move = joint_move_active.load();
  const bool had_jog = jog_active.load();
  int first_error = 0;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    if (had_jog) {
      const int result = robot.jog_stop(-1);
      if (result != 0) {
        first_error = result;
      }
    }
    if (had_goal || had_joint_move || had_jog) {
      const int result = robot.motion_abort();
      if (first_error == 0 && result != 0) {
        first_error = result;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(worker_mutex);
    if (goal_worker.joinable()) {
      goal_worker.join();
    }
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((jog_active.load() || joint_move_active.load()) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if ((jog_active.load() || joint_move_active.load()) && first_error == 0) {
    first_error = -12;
  }
  return first_error;
}

control_s5_controller::RobotShutdownResult shutdownRobot(bool auto_power_off, bool logout)
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex);
  shutting_down.store(true);
  if (!sdk_logged_in) {
    return {};
  }

  control_s5_controller::RobotShutdownOperations operations{
    stopMotionForShutdown,
    []() {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      return robot.servo_move_enable(false);
    },
    []() {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      return robot.disable_robot();
    },
    []() {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      return robot.power_off();
    },
    []() {
      std::lock_guard<std::mutex> lock(sdk_mutex);
      const int result = robot.login_out();
      if (result == 0) {
        sdk_logged_in = false;
      }
      return result;
    }};

  const auto result = control_s5_controller::runRobotShutdownSequence(
    auto_power_off, logout, operations, robot_shutdown_state);
  for (const auto & step : result.steps) {
    if (step.second == 0) {
      RCLCPP_INFO(rclcpp::get_logger("control_s5_motion_server"),
        "Robot shutdown step succeeded: %s", step.first.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"),
        "Robot shutdown step failed: %s: %s", step.first.c_str(),
        errorText(step.second).c_str());
    }
  }
  return result;
}

void handleJogJoint(
  const std::shared_ptr<control_jaka_msgs::srv::JogJoint::Request> request,
  std::shared_ptr<control_jaka_msgs::srv::JogJoint::Response> response)
{
  constexpr double kMaximumJogSpeed = 3.14;
  constexpr double kMaximumJogStep = 10.0 * 3.14159265358979323846 / 180.0;
  if (request->joint_index >= kJointCount ||
    (request->direction != -1 && request->direction != 1) ||
    !std::isfinite(request->speed_rad_s) || request->speed_rad_s <= 0.0 ||
    request->speed_rad_s > kMaximumJogSpeed ||
    !std::isfinite(request->step_rad) || request->step_rad <= 0.0 ||
    request->step_rad > kMaximumJogStep)
  {
    response->success = false;
    response->error_code = -2;
    response->message = "invalid JOG joint, direction, speed, or step";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(motion_state_mutex);
    if (shutting_down.load() || goal_active.load() || jog_active.load() ||
      joint_move_active.load())
    {
      response->success = false;
      response->error_code = -10;
      response->message = shutting_down.load() ?
        "robot shutdown is in progress" : "another robot motion is active";
      return;
    }
    jog_active.store(true);
    jog_cancel_requested.store(false);
  }

  int sdk_result;
  JointValue initial_position{};
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    sdk_result = robot.get_joint_position(&initial_position);
    if (sdk_result == 0) {
      robot.servo_move_enable(false);
      sdk_result = robot.jog(
        request->joint_index, MoveMode::INCR, COORD_JOINT,
        request->direction * request->speed_rad_s, request->step_rad);
    }
  }
  const double expected_position = initial_position.jVal[request->joint_index] +
    request->direction * request->step_rad;
  bool reached = false;
  if (sdk_result == 0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (rclcpp::ok() && !jog_cancel_requested.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
      JointValue current_position{};
      sdk_result = readJointPosition(current_position);
      if (sdk_result != 0 ||
        std::abs(current_position.jVal[request->joint_index] - expected_position) <=
        kPositionTolerance)
      {
        reached = sdk_result == 0;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  if (jog_cancel_requested.load()) {
    response->success = false;
    response->error_code = 0;
    response->message = "JOG motion stopped";
  } else if (sdk_result != 0) {
    response->success = false;
    response->error_code = sdk_result;
    response->message = "JOG failed: " + errorText(sdk_result);
  } else if (reached) {
    response->success = true;
    response->error_code = 0;
    response->message = "JOG increment completed";
  } else {
    {
      std::lock_guard<std::mutex> sdk_lock(sdk_mutex);
      robot.jog_stop(-1);
    }
    response->success = false;
    response->error_code = -12;
    response->message = "JOG timed out before reaching the increment";
  }

  std::lock_guard<std::mutex> lock(motion_state_mutex);
  jog_active.store(false);
  jog_cancel_requested.store(false);
}

void handleStopJog(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (shutting_down.load()) {
    response->success = true;
    response->message = "robot shutdown is already in progress";
    return;
  }
  jog_cancel_requested.store(true);
  std::lock_guard<std::mutex> lock(sdk_mutex);
  const int result = robot.jog_stop(-1);
  response->success = result == 0;
  response->message = result == 0 ? "JOG stop requested" : errorText(result);
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("control_s5_motion_server");
  const std::string robot_ip = node->declare_parameter<std::string>("ip", "10.5.5.100");
  const std::string robot_model = node->declare_parameter<std::string>("model", "s5");
  node->declare_parameter<bool>("diagnostics_enabled", false);
  joint_move_max_speed_rad_s.store(
    node->declare_parameter<double>("joint_move_max_speed_rad_s", 1.57));
  joint_move_max_acceleration_rad_s2.store(
    node->declare_parameter<double>("joint_move_max_acceleration_rad_s2", 1.57));
  const bool auto_power_off_on_exit =
    node->declare_parameter<bool>("auto_power_off_on_exit", true);
  if (!std::isfinite(joint_move_max_speed_rad_s.load()) ||
    joint_move_max_speed_rad_s.load() <= 0.0 ||
    joint_move_max_speed_rad_s.load() > kMaximumJointMoveSpeedRadS ||
    !std::isfinite(joint_move_max_acceleration_rad_s2.load()) ||
    joint_move_max_acceleration_rad_s2.load() <= 0.0 ||
    joint_move_max_acceleration_rad_s2.load() > kMaximumJointMoveAccelerationRadS2)
  {
    RCLCPP_FATAL(node->get_logger(), "joint_move maximum speed or acceleration is invalid");
    rclcpp::shutdown();
    return 1;
  }

  auto parameter_callback = node->add_on_set_parameters_callback(
    [](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      double new_speed = joint_move_max_speed_rad_s.load();
      double new_acceleration = joint_move_max_acceleration_rad_s2.load();
      bool changes_joint_move_limit = false;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "joint_move_max_speed_rad_s") {
          changes_joint_move_limit = true;
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = "joint_move maximum speed must be a double";
            return result;
          }
          new_speed = parameter.as_double();
        } else if (parameter.get_name() == "joint_move_max_acceleration_rad_s2") {
          changes_joint_move_limit = true;
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = "joint_move maximum acceleration must be a double";
            return result;
          }
          new_acceleration = parameter.as_double();
        }
      }
      if (!changes_joint_move_limit) {
        return result;
      }
      if (!std::isfinite(new_speed) || new_speed <= 0.0 ||
        new_speed > kMaximumJointMoveSpeedRadS ||
        !std::isfinite(new_acceleration) || new_acceleration <= 0.0 ||
        new_acceleration > kMaximumJointMoveAccelerationRadS2)
      {
        result.successful = false;
        result.reason = "joint_move maximum speed or acceleration is outside the safe range";
        return result;
      }
      std::lock_guard<std::mutex> lock(motion_state_mutex);
      if (goal_active.load() || joint_move_active.load() || jog_active.load() ||
        shutting_down.load())
      {
        result.successful = false;
        result.reason = "joint_move limits can only be changed while the robot is idle";
        return result;
      }
      joint_move_max_speed_rad_s.store(new_speed);
      joint_move_max_acceleration_rad_s2.store(new_acceleration);
      return result;
    });

  int result = robot.login_in(robot_ip.c_str(), false);
  if (result != 0) {
    RCLCPP_FATAL(node->get_logger(), "Robot login failed: %s", errorText(result).c_str());
    rclcpp::shutdown();
    return 1;
  }
  sdk_logged_in = true;
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    robot.servo_move_enable(false);
    robot.servo_move_use_joint_LPF(0.5);
    result = robot.power_on();
  }
  if (result != 0) {
    RCLCPP_FATAL(node->get_logger(), "Robot power-on failed: %s", errorText(result).c_str());
    shutdownRobot(auto_power_off_on_exit, true);
    rclcpp::shutdown();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(8));
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    result = robot.enable_robot();
  }
  if (result != 0) {
    RCLCPP_FATAL(node->get_logger(), "Robot enable failed: %s", errorText(result).c_str());
    shutdownRobot(auto_power_off_on_exit, true);
    rclcpp::shutdown();
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(4));
  {
    std::lock_guard<std::mutex> lock(sdk_mutex);
    result = robot.set_motion_planner(GlobalPlanner_S);
  }
  if (result != 0) {
    RCLCPP_FATAL(node->get_logger(), "Failed to select S motion planner: %s", errorText(result).c_str());
    shutdownRobot(auto_power_off_on_exit, true);
    rclcpp::shutdown();
    return 1;
  }

  auto joint_state_publisher =
    node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto publish_timer = node->create_wall_timer(
    std::chrono::milliseconds(8),
    [joint_state_publisher, clock = node->get_clock()]() {
      publishJointState(joint_state_publisher, clock);
    });
  auto diagnostic_publisher = node->create_publisher<control_jaka_msgs::msg::RobotDiagnostic>(
    "/jaka_planner/robot_diagnostics", 10);
  auto joint_move_status_publisher = node->create_publisher<JointMoveStatus>(
    "/jaka_planner/joint_move_status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  auto diagnostic_timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [node, diagnostic_publisher, clock = node->get_clock()]() {
      if (node->get_parameter("diagnostics_enabled").as_bool()) {
        publishRobotDiagnostic(diagnostic_publisher, clock);
      } else {
        have_previous_enabled = false;
      }
    });

  auto jog_callback_group =
    node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto joint_move_service = node->create_service<control_jaka_msgs::srv::MoveJoint>(
    "/jaka_planner/joint_move",
    [joint_move_status_publisher, clock = node->get_clock()](
      const std::shared_ptr<control_jaka_msgs::srv::MoveJoint::Request> request,
      std::shared_ptr<control_jaka_msgs::srv::MoveJoint::Response> response)
    {
      handleMoveJoint(request, response, joint_move_status_publisher, clock);
    },
    rmw_qos_profile_services_default, jog_callback_group);
  auto set_rapid_rate_service = node->create_service<control_jaka_msgs::srv::SetRapidRate>(
    "/jaka_driver/set_rapid_rate", handleSetRapidRate,
    rmw_qos_profile_services_default, jog_callback_group);
  auto stop_motion_service = node->create_service<std_srvs::srv::Trigger>(
    "/jaka_planner/stop_motion", handleStopMotion,
    rmw_qos_profile_services_default, jog_callback_group);
  auto jog_service = node->create_service<control_jaka_msgs::srv::JogJoint>(
    "/jaka_planner/jog_joint", handleJogJoint,
    rmw_qos_profile_services_default, jog_callback_group);
  auto stop_jog_service = node->create_service<std_srvs::srv::Trigger>(
    "/jaka_planner/stop_jog", handleStopJog,
    rmw_qos_profile_services_default, jog_callback_group);
  auto shutdown_service = node->create_service<std_srvs::srv::Trigger>(
    "/jaka_planner/shutdown_robot",
    [auto_power_off_on_exit](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      const auto result = shutdownRobot(auto_power_off_on_exit, false);
      response->success = result.success();
      response->message = result.summary();
      if (response->success) {
        std::thread([]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
          if (rclcpp::ok()) {
            rclcpp::shutdown();
          }
        }).detach();
      }
    },
    rmw_qos_profile_services_default, jog_callback_group);

  auto action_server = rclcpp_action::create_server<FollowJointTrajectory>(
    node,
    "/jaka_" + robot_model + "_controller/follow_joint_trajectory",
    [](const rclcpp_action::GoalUUID &,
      const std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
      std::string reason;
      if (!validateGoal(*goal, reason)) {
        RCLCPP_ERROR(rclcpp::get_logger("control_s5_motion_server"),
          "Rejecting trajectory: %s", reason.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      std::lock_guard<std::mutex> lock(motion_state_mutex);
      if (shutting_down.load() || goal_active.load() || jog_active.load() ||
        joint_move_active.load())
      {
        RCLCPP_WARN(rclcpp::get_logger("control_s5_motion_server"), "Rejecting concurrent trajectory goal");
        return rclcpp_action::GoalResponse::REJECT;
      }
      goal_active.store(true);
      cancel_requested.store(false);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandle>) {
      if (!goal_active.load()) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      stopActiveMotion();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<GoalHandle> goal_handle) {
      std::lock_guard<std::mutex> lock(worker_mutex);
      if (shutting_down.load()) {
        goal_handle->abort(makeResult(
          FollowJointTrajectory::Result::INVALID_GOAL, "robot shutdown is in progress"));
        std::lock_guard<std::mutex> state_lock(motion_state_mutex);
        goal_active.store(false);
        return;
      }
      if (goal_worker.joinable()) {
        goal_worker.join();
      }
      goal_worker = std::thread(executeGoal, goal_handle);
    });

  (void)publish_timer;
  (void)diagnostic_timer;
  (void)joint_move_status_publisher;
  (void)parameter_callback;
  (void)joint_move_service;
  (void)set_rapid_rate_service;
  (void)stop_motion_service;
  (void)jog_service;
  (void)stop_jog_service;
  (void)shutdown_service;
  (void)action_server;
  RCLCPP_INFO(node->get_logger(),
    "JAKA motion server started for model %s (joint_move service and legacy trajectory action)",
    robot_model.c_str());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();

  shutdownRobot(auto_power_off_on_exit, true);
  rclcpp::shutdown();
  return 0;
}
