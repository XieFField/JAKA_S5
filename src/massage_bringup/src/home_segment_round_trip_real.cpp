#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "jaka_msgs/msg/robot_msg.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_bringup/default_targets.hpp"
#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/execution_timing.hpp"
#include "massage_motion/guarded_trajectory_executor.hpp"
#include "massage_motion/joint_target_interpolation.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"
#include "massage_motion/trajectory_execution_contract.hpp"

namespace
{

struct JointLegResult
{
  std::string name;
  double initial{0.0};
  double target{0.0};
  double actual{0.0};
  double commanded_delta{0.0};
  double achieved_delta{0.0};
  double completion_ratio{0.0};
  double absolute_error{0.0};
};

struct LegResult
{
  std::string name;
  bool passed{false};
  std::string message;
  std::vector<JointLegResult> joints;
  double maximum_endpoint_error{0.0};
  double planned_duration{0.0};
  double execution_timeout{0.0};
  double execution_duration{0.0};
  bool execution_success{false};
  std::int32_t execution_status{0};
  std::int32_t backend_error_code{0};
  std::int32_t motion_state{-1};
  std::int32_t power_state{-1};
  std::int32_t servo_state{-1};
  std::int32_t collision_state{-1};
  std::size_t trajectory_points{0U};
  std::size_t selected_candidate{std::numeric_limits<std::size_t>::max()};
  double selected_score{std::numeric_limits<double>::infinity()};
  double path_length{0.0};
  double maximum_joint_travel{0.0};
};

bool robot_ready(const jaka_msgs::msg::RobotMsg & state)
{
  return state.motion_state == 0 && state.power_state == 1 &&
         state.servo_state == 1 && state.collision_state == 0;
}

bool extract_ordered_positions(
  const sensor_msgs::msg::JointState & state,
  const std::vector<std::string> & expected_joint_names,
  std::vector<double> & positions,
  std::string & error)
{
  if (state.name.size() != state.position.size())
  {
    error = "JointState 名称和位置数组长度不一致";
    return false;
  }
  const std::set<std::string> unique_names(state.name.begin(), state.name.end());
  if (unique_names.size() != state.name.size())
  {
    error = "JointState 包含重复关节名称";
    return false;
  }

  positions.clear();
  positions.reserve(expected_joint_names.size());
  for (const auto & name : expected_joint_names)
  {
    const auto iterator = std::find(state.name.begin(), state.name.end(), name);
    if (iterator == state.name.end())
    {
      error = "JointState 缺少 " + name;
      return false;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(state.name.begin(), iterator));
    if (!std::isfinite(state.position[index]))
    {
      error = name + " 的位置不是有限数值";
      return false;
    }
    positions.push_back(state.position[index]);
  }
  return true;
}

double maximum_position_difference(
  const std::vector<double> & first,
  const std::vector<double> & second)
{
  if (first.size() != second.size())
  {
    return std::numeric_limits<double>::infinity();
  }
  double maximum = 0.0;
  for (std::size_t index = 0; index < first.size(); ++index)
  {
    if (!std::isfinite(first[index]) || !std::isfinite(second[index]))
    {
      return std::numeric_limits<double>::infinity();
    }
    maximum = std::max(maximum, std::abs(first[index] - second[index]));
  }
  return maximum;
}

bool extract_trajectory_start(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::vector<std::string> & expected_joint_names,
  std::vector<double> & positions,
  std::string & error)
{
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty())
  {
    error = "规划轨迹没有关节名称或轨迹点";
    return false;
  }
  const auto & first_point = joint_trajectory.points.front();
  if (first_point.positions.size() != joint_trajectory.joint_names.size())
  {
    error = "规划轨迹首点维度无效";
    return false;
  }

  positions.clear();
  positions.reserve(expected_joint_names.size());
  for (const auto & name : expected_joint_names)
  {
    const auto iterator = std::find(
      joint_trajectory.joint_names.begin(), joint_trajectory.joint_names.end(), name);
    if (iterator == joint_trajectory.joint_names.end())
    {
      error = "规划轨迹缺少 " + name;
      return false;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(joint_trajectory.joint_names.begin(), iterator));
    if (!std::isfinite(first_point.positions[index]))
    {
      error = "规划轨迹首点包含非有限数值";
      return false;
    }
    positions.push_back(first_point.positions[index]);
  }
  return true;
}

void log_competition(
  const rclcpp::Logger & logger,
  const std::string & leg_name,
  const massage_motion::PlanCompetitionReport & report)
{
  for (std::size_t index = 0; index < report.candidates.size(); ++index)
  {
    const auto & candidate = report.candidates[index];
    RCLCPP_INFO(
      logger,
      "%s 候选[%zu]: source=%s attempt=%zu accepted=%s score=%.6f "
      "path=%.6f rad duration=%.3f s max_joint=%.6f rad message=%s",
      leg_name.c_str(), index, candidate.source_name.c_str(), candidate.attempt,
      candidate.accepted ? "true" : "false", candidate.score,
      candidate.metrics.joint_path_length, candidate.metrics.duration,
      candidate.metrics.maximum_joint_travel, candidate.message.c_str());
  }
}

std::string make_csv_path()
{
  const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  return "/tmp/massage_home_segment_round_trip_" +
         std::to_string(stamp) + ".csv";
}

std::string csv_text(const std::string & value)
{
  std::string escaped{"\""};
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

void write_csv_header(std::ofstream & stream)
{
  stream << "leg,passed,joint,initial,target,actual,commanded_delta,"
            "achieved_delta,completion_ratio,absolute_error,"
            "maximum_endpoint_error,planned_duration,execution_timeout,"
            "execution_duration,execution_success,execution_status,"
            "backend_error_code,motion_state,power_state,servo_state,"
            "collision_state,trajectory_points,selected_candidate,selected_score,"
            "path_length,maximum_joint_travel,message\n";
}

void write_leg_csv(std::ofstream & stream, const LegResult & result)
{
  stream << std::setprecision(12);
  for (const auto & joint : result.joints)
  {
    stream << result.name << ',' << result.passed << ',' << joint.name << ','
           << joint.initial << ',' << joint.target << ',' << joint.actual << ','
           << joint.commanded_delta << ',' << joint.achieved_delta << ','
           << joint.completion_ratio << ',' << joint.absolute_error << ','
           << result.maximum_endpoint_error << ',' << result.planned_duration << ','
           << result.execution_timeout << ',' << result.execution_duration << ','
           << result.execution_success << ',' << result.execution_status << ','
           << result.backend_error_code << ',' << result.motion_state << ','
           << result.power_state << ',' << result.servo_state << ','
           << result.collision_state << ','
           << result.trajectory_points << ',';
    if (result.selected_candidate == std::numeric_limits<std::size_t>::max())
    {
      stream << -1;
    }
    else
    {
      stream << result.selected_candidate;
    }
    stream << ',' << result.selected_score << ',' << result.path_length << ','
           << result.maximum_joint_travel << ',' << csv_text(result.message) << '\n';
  }
  stream.flush();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "home_segment_round_trip_real",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  bool execute = false;
  bool parameters_confirmed = false;
  double segment_ratio = 0.25;
  int planning_attempts = 3;
  double velocity_scale = 0.02;
  double acceleration_scale = 0.02;
  double planning_timeout = 5.0;
  double execution_timeout_margin = 10.0;
  double state_timeout = 3.0;
  double readiness_timeout = 30.0;
  double feedback_timeout = 1.0;
  double maximum_joint_travel = 0.15;
  double start_state_tolerance = 0.01;
  double endpoint_tolerance = 0.002;
  double settling_duration = 0.3;
  double inter_leg_delay = 0.5;
  std::string output_csv;
  node->get_parameter_or("execute", execute, false);
  node->get_parameter_or("parameters_confirmed", parameters_confirmed, false);
  node->get_parameter_or("segment_ratio", segment_ratio, 0.25);
  node->get_parameter_or("planning_attempts", planning_attempts, 3);
  node->get_parameter_or("velocity_scale", velocity_scale, 0.02);
  node->get_parameter_or("acceleration_scale", acceleration_scale, 0.02);
  node->get_parameter_or("planning_timeout", planning_timeout, 5.0);
  node->get_parameter_or(
    "execution_timeout_margin", execution_timeout_margin, 10.0);
  node->get_parameter_or("state_timeout", state_timeout, 3.0);
  node->get_parameter_or("readiness_timeout", readiness_timeout, 30.0);
  node->get_parameter_or("feedback_timeout", feedback_timeout, 1.0);
  node->get_parameter_or("maximum_joint_travel", maximum_joint_travel, 0.15);
  node->get_parameter_or("start_state_tolerance", start_state_tolerance, 0.01);
  node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.002);
  node->get_parameter_or("settling_duration", settling_duration, 0.3);
  node->get_parameter_or("inter_leg_delay", inter_leg_delay, 0.5);
  node->get_parameter_or("output_csv", output_csv, std::string{});

  if (!execute || !parameters_confirmed)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "真机分段往返只允许显式执行：需要 execute:=true 和 "
      "parameters_confirmed:=true");
    rclcpp::shutdown();
    return 2;
  }
  if (!std::isfinite(segment_ratio) || segment_ratio <= 0.0 ||
    segment_ratio > 1.0 || planning_attempts < 1 || planning_attempts > 10 ||
    !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
    velocity_scale > 1.0 || !std::isfinite(acceleration_scale) ||
    acceleration_scale <= 0.0 || acceleration_scale > 1.0 ||
    !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
    !std::isfinite(execution_timeout_margin) || execution_timeout_margin < 0.0 ||
    !std::isfinite(state_timeout) || state_timeout <= 0.0 ||
    !std::isfinite(readiness_timeout) || readiness_timeout <= 0.0 ||
    !std::isfinite(feedback_timeout) || feedback_timeout <= 0.0 ||
    !std::isfinite(maximum_joint_travel) || maximum_joint_travel <= 0.0 ||
    !std::isfinite(start_state_tolerance) || start_state_tolerance <= 0.0 ||
    !std::isfinite(endpoint_tolerance) || endpoint_tolerance <= 0.0 ||
    endpoint_tolerance >= maximum_joint_travel ||
    !std::isfinite(settling_duration) || settling_duration < 0.0 ||
    !std::isfinite(inter_leg_delay) || inter_leg_delay < 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "真机分段往返参数无效");
    rclcpp::shutdown();
    return 2;
  }
  if (output_csv.empty())
  {
    output_csv = make_csv_path();
  }

  const std::vector<std::string> joint_names(
    massage_bringup::kJakaS5JointNames.begin(),
    massage_bringup::kJakaS5JointNames.end());
  const std::vector<double> home_positions(
    massage_bringup::kMassageHomeJointPositions.begin(),
    massage_bringup::kMassageHomeJointPositions.end());

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
  std::shared_ptr<massage_motion::ITrajectoryExecutor> trajectory_executor;
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
      if (!robot_ready(latest_robot_state))
      {
        RCLCPP_INFO(
          node->get_logger(), "等待机器人进入可执行状态，最长 %.1f 秒",
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
        baseline_state, joint_names, baseline_positions, state_error))
    {
      throw std::runtime_error("往返基准状态无效: " + state_error);
    }
    const auto segment = massage_motion::interpolate_joint_target(
      baseline_positions, home_positions, segment_ratio, maximum_joint_travel);
    if (!segment.valid)
    {
      throw std::runtime_error("分段目标无效: " + segment.message);
    }
    if (segment.maximum_joint_travel <= endpoint_tolerance)
    {
      throw std::runtime_error("分段目标位移不大于终点容差，拒绝执行");
    }

    RCLCPP_INFO(
      node->get_logger(),
      "分段往返固定目标已锁定: ratio=%.3f, max_joint=%.9f rad；"
      "返程目标为本次启动时保存的原始六轴状态",
      segment_ratio, segment.maximum_joint_travel);
    for (std::size_t joint = 0; joint < joint_names.size(); ++joint)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "%s: baseline=%.9f rad, home=%.9f rad, outbound=%.9f rad, "
        "travel=%.9f rad",
        joint_names[joint].c_str(), baseline_positions[joint],
        home_positions[joint], segment.target_positions[joint],
        segment.joint_travels[joint]);
    }

    auto parameter_node = std::make_shared<rclcpp::Node>(
      "home_segment_driver_parameter_client");
    auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(
      parameter_node, "/jaka_driver");
    if (!parameter_client->wait_for_service(
        std::chrono::duration<double>(state_timeout)))
    {
      throw std::runtime_error("等待 /jaka_driver 参数服务超时");
    }
    const auto driver_parameters = parameter_client->get_parameters({
      "trajectory_goal_tolerance", "trajectory_goal_timeout"});
    if (driver_parameters.size() != 2U)
    {
      throw std::runtime_error("驱动终点参数读取不完整");
    }
    const double driver_goal_tolerance = driver_parameters[0].as_double();
    const double driver_goal_timeout = driver_parameters[1].as_double();
    const auto execution_contract =
      massage_motion::validate_trajectory_execution_contract(
      driver_goal_tolerance, driver_goal_timeout, endpoint_tolerance,
      execution_timeout_margin);
    if (!execution_contract.valid)
    {
      std::ostringstream message;
      message << "驱动终点参数与往返门槛不一致: "
              << execution_contract.message << "; tolerance="
              << driver_goal_tolerance << " rad, endpoint_tolerance="
              << endpoint_tolerance << " rad, driver_margin="
              << driver_goal_timeout << " s, execution_margin="
              << execution_timeout_margin << " s";
      throw std::runtime_error(message.str());
    }
    RCLCPP_INFO(
      node->get_logger(),
      "驱动终点参数读回通过: tolerance=%.9f rad, driver_margin=%.3f s, "
      "execution_margin=%.3f s",
      driver_goal_tolerance, driver_goal_timeout, execution_timeout_margin);

    std::ofstream csv_stream(output_csv);
    if (!csv_stream)
    {
      throw std::runtime_error("无法创建往返遥测 CSV: " + output_csv);
    }
    write_csv_header(csv_stream);

    massage_motion::PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "massage_tool_tip";
    planner_config.reference_frame = "world";
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";
    auto sdk = std::make_shared<massage_motion::MotionPlanningSdk>(
      node, planner_config);
    massage_motion::PlanCompetitionConfig competition_config;
    competition_config.maximum_joint_travel = maximum_joint_travel;
    auto planner = std::make_shared<massage_motion::CompetitiveMotionPlanner>(
      std::vector<massage_motion::PlanningSource>{
        {"motion_sdk", sdk, static_cast<std::size_t>(planning_attempts)}},
      competition_config);
    auto backend =
      std::make_shared<massage_motion::MoveItTrajectoryExecutor>(node);
    trajectory_executor =
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
        return {true, massage_motion::ExecutionError::kNone, "ready"};
      });

    const auto run_leg = [&](
      const std::string & leg_name,
      const std::vector<double> & target_positions) -> LegResult
      {
        LegResult result;
        result.name = leg_name;
        sensor_msgs::msg::JointState initial_state;
        std::vector<double> initial_positions;
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          initial_state = latest_joint_state;
          const auto now = std::chrono::steady_clock::now();
          const double joint_age = std::chrono::duration<double>(
            now - joint_received_at).count();
          const double robot_age = std::chrono::duration<double>(
            now - robot_received_at).count();
          if (joint_age > feedback_timeout || robot_age > feedback_timeout ||
            !robot_ready(latest_robot_state))
          {
            result.message = "本程开始状态过期或机器人未就绪";
            return result;
          }
        }
        if (!extract_ordered_positions(
            initial_state, joint_names, initial_positions, state_error))
        {
          result.message = "本程初始关节状态无效: " + state_error;
          return result;
        }
        result.joints.reserve(joint_names.size());
        for (std::size_t joint = 0; joint < joint_names.size(); ++joint)
        {
          JointLegResult joint_result;
          joint_result.name = joint_names[joint];
          joint_result.initial = initial_positions[joint];
          joint_result.target = target_positions[joint];
          joint_result.actual = initial_positions[joint];
          joint_result.commanded_delta =
            target_positions[joint] - initial_positions[joint];
          result.joints.push_back(joint_result);
        }
        const double commanded_travel = maximum_position_difference(
          initial_positions, target_positions);
        if (!std::isfinite(commanded_travel) ||
          commanded_travel <= endpoint_tolerance ||
          commanded_travel > maximum_joint_travel)
        {
          std::ostringstream message;
          message << "本程命令行程无效: max_joint=" << commanded_travel
                  << " rad, allowed=(" << endpoint_tolerance << ", "
                  << maximum_joint_travel << "] rad";
          result.message = message.str();
          return result;
        }

        RCLCPP_INFO(
          node->get_logger(),
          "%s 阶段开始: current_to_target_max=%.9f rad, attempts=%d",
          leg_name.c_str(), commanded_travel, planning_attempts);
        massage_motion::MotionRequest request;
        request.request_id = "home_segment_" + leg_name;
        request.motion_type = massage_motion::MotionType::kPtp;
        request.target = massage_motion::JointTarget{target_positions};
        request.velocity_scale = velocity_scale;
        request.acceleration_scale = acceleration_scale;
        request.planning_timeout = planning_timeout;

        const auto plan = planner->plan(request);
        const auto report = planner->last_report();
        log_competition(node->get_logger(), leg_name, report);
        if (!plan.success || !report.success ||
          report.selected_candidate >= report.candidates.size())
        {
          result.message = "竞争规划失败: " + plan.message;
          return result;
        }
        result.selected_candidate = report.selected_candidate;
        const auto & selected = report.candidates[result.selected_candidate];
        result.selected_score = selected.score;
        result.path_length = selected.metrics.joint_path_length;
        result.maximum_joint_travel = selected.metrics.maximum_joint_travel;
        result.trajectory_points =
          plan.trajectory.joint_trajectory.points.size();

        std::vector<double> planned_start;
        std::string trajectory_error;
        if (!extract_trajectory_start(
            plan.trajectory, joint_names, planned_start, trajectory_error))
        {
          result.message = "规划起点无效: " + trajectory_error;
          return result;
        }
        const double start_error = maximum_position_difference(
          initial_positions, planned_start);
        if (!std::isfinite(start_error) || start_error > start_state_tolerance)
        {
          std::ostringstream message;
          message << "规划起点与本程新鲜反馈不一致: max_error=" << start_error
                  << " rad, tolerance=" << start_state_tolerance << " rad";
          result.message = message.str();
          return result;
        }

        massage_motion::ExecutionTimingPolicy timing_policy;
        timing_policy.margin = execution_timeout_margin;
        const auto timing = massage_motion::calculate_execution_timing(
          plan.trajectory, timing_policy);
        if (!timing.valid)
        {
          result.message = "无法计算执行时限: " + timing.message;
          return result;
        }
        result.planned_duration = timing.expected_duration;
        result.execution_timeout = timing.timeout;
        RCLCPP_INFO(
          node->get_logger(),
          "%s: selected=%zu, points=%zu, start_error=%.9f rad, %s",
          leg_name.c_str(), result.selected_candidate, result.trajectory_points,
          start_error,
          timing.message.c_str());

        massage_motion::ExecutionRequest execution_request;
        execution_request.request_id = request.request_id + "_execution";
        execution_request.robot_trajectory = plan.trajectory;
        execution_request.timeout = timing.timeout;
        const auto execution_started = std::chrono::steady_clock::now();
        const auto execution = trajectory_executor->execute(execution_request);
        result.execution_duration = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - execution_started).count();
        result.execution_success = execution.success;
        result.execution_status = static_cast<std::int32_t>(execution.status);
        result.backend_error_code = execution.backend_error_code;

        std::uint64_t sequence_before_settling = 0U;
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
              lock, std::chrono::duration<double>(state_timeout),
              [&]() {return joint_sequence > sequence_before_settling;}))
          {
            result.message = "轨迹执行结果: " + execution.message +
              "; 等待本程执行后的新鲜关节反馈超时";
            return result;
          }
          final_state = latest_joint_state;
          final_robot_state = latest_robot_state;
          const auto checked_at = std::chrono::steady_clock::now();
          final_joint_age = std::chrono::duration<double>(
            checked_at - joint_received_at).count();
          final_robot_age = std::chrono::duration<double>(
            checked_at - robot_received_at).count();
        }
        result.motion_state = final_robot_state.motion_state;
        result.power_state = final_robot_state.power_state;
        result.servo_state = final_robot_state.servo_state;
        result.collision_state = final_robot_state.collision_state;
        const auto endpoint = massage_motion::calculate_trajectory_endpoint_error(
          plan.trajectory, final_state);
        if (!endpoint.valid)
        {
          result.message = "终点误差计算失败: " + endpoint.message;
          return result;
        }
        std::unordered_map<std::string, massage_motion::JointPositionError>
          endpoint_by_name;
        for (const auto & joint_error : endpoint.joint_errors)
        {
          endpoint_by_name.emplace(joint_error.joint_name, joint_error);
        }
        for (auto & joint : result.joints)
        {
          const auto iterator = endpoint_by_name.find(joint.name);
          if (iterator == endpoint_by_name.end())
          {
            result.message = "终点误差缺少 " + joint.name;
            return result;
          }
          joint.target = iterator->second.target_position;
          joint.actual = iterator->second.actual_position;
          joint.commanded_delta = joint.target - joint.initial;
          joint.achieved_delta = joint.actual - joint.initial;
          joint.completion_ratio = std::abs(joint.commanded_delta) > 1e-12 ?
            joint.achieved_delta / joint.commanded_delta : 1.0;
          joint.absolute_error = iterator->second.absolute_error;
          RCLCPP_INFO(
            node->get_logger(),
            "%s %s: initial=%.9f target=%.9f actual=%.9f rad, "
            "commanded=%.9f achieved=%.9f rad, completion=%.2f%%, error=%.9f rad",
            leg_name.c_str(), joint.name.c_str(), joint.initial, joint.target,
            joint.actual, joint.commanded_delta, joint.achieved_delta,
            joint.completion_ratio * 100.0, joint.absolute_error);
        }
        result.maximum_endpoint_error = endpoint.max_absolute_error;
        const bool feedback_fresh = final_joint_age <= feedback_timeout &&
          final_robot_age <= feedback_timeout;
        const bool final_ready = robot_ready(final_robot_state);
        result.passed = execution.success && feedback_fresh && final_ready &&
          result.maximum_endpoint_error <= endpoint_tolerance;
        std::ostringstream message;
        message << "execution=" << execution.message
                << ", execution_status=" << result.execution_status
                << ", backend_error=" << result.backend_error_code
                << ", feedback_fresh=" << feedback_fresh
                << ", robot_ready=" << final_ready
                << ", max_error=" << result.maximum_endpoint_error
                << " rad, tolerance=" << endpoint_tolerance << " rad";
        result.message = message.str();
        return result;
      };

    const auto outbound = run_leg("outbound", segment.target_positions);
    write_leg_csv(csv_stream, outbound);
    if (!outbound.passed)
    {
      throw std::runtime_error(
              "分段去程失败，已禁止返程: " + outbound.message);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "分段去程通过: ratio=%.3f, max_error=%.9f rad, execution=%.3f s；"
      "%.1f 秒后规划返程",
      segment_ratio, outbound.maximum_endpoint_error, outbound.execution_duration,
      inter_leg_delay);
    std::this_thread::sleep_for(std::chrono::duration<double>(inter_leg_delay));

    const auto inbound = run_leg("return", baseline_positions);
    write_leg_csv(csv_stream, inbound);
    if (!inbound.passed)
    {
      throw std::runtime_error(
              "分段返程失败，已停止后续运动: " + inbound.message);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "HOME SEGMENT ROUND-TRIP: PASS: ratio=%.3f, outbound_error=%.9f rad, "
      "return_error=%.9f rad, csv=%s",
      segment_ratio, outbound.maximum_endpoint_error,
      inbound.maximum_endpoint_error, output_csv.c_str());
    exit_code = 0;
  }
  catch (const std::exception & exception)
  {
    if (trajectory_executor)
    {
      (void)trajectory_executor->cancel();
    }
    RCLCPP_ERROR(
      node->get_logger(), "真机分段往返失败: %s", exception.what());
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      if (robot_sequence > 0U)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "失败停止状态快照: motion=%d, power=%d, servo=%d, collision=%d",
          latest_robot_state.motion_state, latest_robot_state.power_state,
          latest_robot_state.servo_state, latest_robot_state.collision_state);
      }
    }
    RCLCPP_INFO(node->get_logger(), "往返遥测 CSV: %s", output_csv.c_str());
    exit_code = 3;
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
