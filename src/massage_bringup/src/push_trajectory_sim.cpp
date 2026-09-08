#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"

#include "massage_bringup/default_targets.hpp"
#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/elbow_posture_geometry.hpp"
#include "massage_motion/link_trajectory_geometry.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/pose_ik_candidate_generator.hpp"
#include "massage_motion/progressive_joint_checkpoints.hpp"
#include "massage_motion/technique_path_generator.hpp"
#include "massage_motion/tool_orientation.hpp"
#ifndef MASSAGE_PLAN_ONLY_BUILD
#include "massage_motion/cartesian_path_verification.hpp"
#include "massage_motion/execution_timing.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"
#endif

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kFullRevolution = 2.0 * kPi;
#ifdef MASSAGE_PLAN_ONLY_BUILD
constexpr bool kExecutionBackendCompiled = false;
#else
constexpr bool kExecutionBackendCompiled = true;
#endif

struct PushTestProfile
{
  geometry_msgs::msg::PoseStamped start_pose;
  double direction_x{0.0};
  double direction_y{0.0};
  double length{0.0};
  double speed{0.0};
  double sample_period{0.05};
  double maximum_speed{0.02};
  geometry_msgs::msg::Point expected_end;
  std::array<double, 3> surface_normal{0.0, 0.0, 1.0};
};

double required_double(const YAML::Node & node, const std::string & key)
{
  if (!node[key])
  {
    throw std::runtime_error("配置缺少字段: " + key);
  }
  const double value = node[key].as<double>();
  if (!std::isfinite(value))
  {
    throw std::runtime_error("配置字段不是有限数值: " + key);
  }
  return value;
}

PushTestProfile load_profile(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (!root["profile"] ||
    root["profile"].as<std::string>() != "push_contact_sequence_stage_b")
  {
    throw std::runtime_error("配置 profile 不是 push_contact_sequence_stage_b");
  }
  const auto validation = root["validation"];
  if (!validation ||
    !validation["free_space_start_ptp_plan_valid"].as<bool>() ||
    validation["free_space_push_path_execution_validated"].as<bool>())
  {
    throw std::runtime_error("配置验证标志不符合 Stage B 首次执行前状态");
  }

  PushTestProfile profile;
  const auto start = root["pre_contact_pose"];
  const auto position = start["position"];
  profile.start_pose.header.frame_id = start["reference_frame"].as<std::string>();
  profile.start_pose.pose.position.x = required_double(position, "x");
  profile.start_pose.pose.position.y = required_double(position, "y");
  profile.start_pose.pose.position.z = required_double(position, "z");

  const auto test = root["free_space_push_test"];
  const auto direction = test["direction_xy"];
  const auto expected_end = test["expected_end_pose"];
  profile.direction_x = required_double(direction, "x");
  profile.direction_y = required_double(direction, "y");
  profile.length = required_double(test, "length");
  profile.speed = required_double(test, "speed");
  if (test["sample_period"])
  {
    profile.sample_period = required_double(test, "sample_period");
  }
  if (test["maximum_speed"])
  {
    profile.maximum_speed = required_double(test, "maximum_speed");
  }
  profile.expected_end.x = required_double(expected_end, "x");
  profile.expected_end.y = required_double(expected_end, "y");
  profile.expected_end.z = required_double(expected_end, "z");
  const auto contact_entry = root["business_contact_entry"];
  const auto surface_normal = contact_entry["surface_normal"];
  profile.surface_normal = {
    required_double(surface_normal, "x"),
    required_double(surface_normal, "y"),
    required_double(surface_normal, "z")};
  return profile;
}

moveit_msgs::msg::RobotState terminal_state(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty() ||
    joint_trajectory.points.back().positions.size() !=
    joint_trajectory.joint_names.size())
  {
    throw std::runtime_error("无法从上一段轨迹构造下一段规划起点");
  }
  moveit_msgs::msg::RobotState state;
  state.joint_state.name = joint_trajectory.joint_names;
  state.joint_state.position = joint_trajectory.points.back().positions;
  state.is_diff = true;
  return state;
}

moveit_msgs::msg::RobotState joint_target_state(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions)
{
  if (joint_names.empty() || joint_names.size() != positions.size() ||
    !std::all_of(
      positions.begin(), positions.end(),
      [](double value) {return std::isfinite(value);}))
  {
    throw std::runtime_error("无法从关节目标构造规划起点");
  }
  moveit_msgs::msg::RobotState state;
  state.joint_state.name = joint_names;
  state.joint_state.position = positions;
  state.is_diff = true;
  return state;
}

double maximum_position_difference(
  const std::vector<double> & first, const std::vector<double> & second)
{
  if (first.empty() || first.size() != second.size())
  {
    return std::numeric_limits<double>::infinity();
  }
  double maximum = 0.0;
  for (std::size_t index = 0; index < first.size(); ++index)
  {
    maximum = std::max(maximum, std::abs(second[index] - first[index]));
  }
  return maximum;
}

std::vector<double> parse_fraction_sequence(const std::string & text)
{
  const YAML::Node fractions = YAML::Load(text);
  if (!fractions.IsSequence())
  {
    throw std::runtime_error(
      "progressive_checkpoint_fractions 必须是 YAML 数组");
  }
  std::vector<double> result;
  result.reserve(fractions.size());
  for (const auto & fraction : fractions)
  {
    const double value = fraction.as<double>();
    if (!std::isfinite(value))
    {
      throw std::runtime_error("渐进检查点比例包含非有限数值");
    }
    result.push_back(value);
  }
  return result;
}

int signed_side(double value, double tolerance)
{
  if (value > tolerance)
  {
    return 1;
  }
  if (value < -tolerance)
  {
    return -1;
  }
  return 0;
}

void log_ik_report(
  const rclcpp::Logger & logger,
  const massage_motion::PoseIkCandidateReport & report)
{
  RCLCPP_INFO(
    logger,
    "动态 IK 汇总: success=%s, frame=%s, business_tip=%s, solver_tip=%s, "
    "fixed_tip_offset=%s, translation=%.6f m, rotation=%.6f rad, "
    "attempts=%zu, unique=%zu, message=%s",
    report.success ? "true" : "false", report.model_frame.c_str(),
    report.requested_tip_link.c_str(), report.solver_tip_link.c_str(),
    report.fixed_tip_offset_applied ? "true" : "false",
    report.target_distance.translation, report.target_distance.rotation,
    report.attempts.size(), report.candidates.size(), report.message.c_str());
  for (const auto & attempt : report.attempts)
  {
    RCLCPP_INFO(
      logger,
      "IK 尝试[%zu]: timeout=%.6f s, seed=%s, success=%s, "
      "duplicate=%s, message=%s",
      attempt.attempt, attempt.timeout,
      attempt.used_current_state_seed ? "current" : "deterministic_random",
      attempt.ik_success ? "true" : "false",
      attempt.duplicate ? "true" : "false", attempt.message.c_str());
  }
  for (std::size_t index = 0; index < report.candidates.size(); ++index)
  {
    const auto & candidate = report.candidates[index];
    std::string positions;
    for (std::size_t joint_index = 0;
      joint_index < candidate.positions.size(); ++joint_index)
    {
      if (!positions.empty())
      {
        positions += ", ";
      }
      positions += report.variable_names[joint_index] + "=" +
        std::to_string(candidate.positions[joint_index]);
    }
    RCLCPP_INFO(
      logger,
      "唯一 IK 候选[%zu]: source_attempt=%zu, timeout=%.6f s, joints=[%s]",
      index, candidate.source_attempt, candidate.timeout, positions.c_str());
  }
}

void log_competition(
  const rclcpp::Logger & logger, const std::string & phase,
  const massage_motion::PlanCompetitionReport & report,
  double maximum_joint_travel,
  const moveit::core::RobotModelConstPtr & robot_model)
{
  const std::unordered_set<std::string> model_variables(
    robot_model->getVariableNames().begin(),
    robot_model->getVariableNames().end());
  for (std::size_t index = 0; index < report.candidates.size(); ++index)
  {
    const auto & candidate = report.candidates[index];
    RCLCPP_INFO(
      logger,
      "%s 候选[%zu]: attempt=%zu accepted=%s score=%.6f path=%.6f rad "
      "duration=%.3f s max_joint=%s max_travel=%.6f rad limit=%.6f rad "
      "message=%s",
      phase.c_str(), index, candidate.attempt,
      candidate.accepted ? "true" : "false", candidate.score,
      candidate.metrics.joint_path_length, candidate.metrics.duration,
      candidate.metrics.maximum_joint_travel_name.empty() ?
      "unavailable" : candidate.metrics.maximum_joint_travel_name.c_str(),
      candidate.metrics.maximum_joint_travel, maximum_joint_travel,
      candidate.message.c_str());
    if (!candidate.metrics.valid)
    {
      continue;
    }
    for (const auto & joint : candidate.metrics.joint_travels)
    {
      if (model_variables.count(joint.joint_name) == 0U)
      {
        RCLCPP_WARN(
          logger,
          "%s 候选[%zu] 关节 %s 不存在于 RobotModel，无法读取物理限制",
          phase.c_str(), index, joint.joint_name.c_str());
        continue;
      }
      const auto & bounds = robot_model->getVariableBounds(joint.joint_name);
      if (bounds.position_bounded_)
      {
        RCLCPP_INFO(
          logger,
          "%s 候选[%zu] 关节 %s: start=%+.9f rad, goal=%+.9f rad, "
          "delta=%+.9f rad, abs=%.9f rad, revolution_ratio=%.3f, "
          "limits=[%+.6f, %+.6f] rad, goal_margin=[lower=%.6f, upper=%.6f] rad",
          phase.c_str(), index, joint.joint_name.c_str(), joint.start_position,
          joint.goal_position, joint.signed_travel, joint.absolute_travel,
          joint.absolute_travel / kFullRevolution, bounds.min_position_,
          bounds.max_position_, joint.goal_position - bounds.min_position_,
          bounds.max_position_ - joint.goal_position);
      }
      else
      {
        RCLCPP_INFO(
          logger,
          "%s 候选[%zu] 关节 %s: start=%+.9f rad, goal=%+.9f rad, "
          "delta=%+.9f rad, abs=%.9f rad, revolution_ratio=%.3f, "
          "position_limits=unbounded",
          phase.c_str(), index, joint.joint_name.c_str(), joint.start_position,
          joint.goal_position, joint.signed_travel, joint.absolute_travel,
          joint.absolute_travel / kFullRevolution);
      }
    }
  }
}

void log_elbow_posture_metrics(
  const rclcpp::Logger & logger, const std::string & phase,
  const massage_motion::ElbowPostureMetrics & metrics,
  const massage_motion::ElbowPostureGeometryConfig & config)
{
  if (!metrics.valid)
  {
    RCLCPP_WARN(
      logger, "%s 肘部构型诊断失败: %s", phase.c_str(),
      metrics.message.c_str());
    return;
  }
  RCLCPP_INFO(
    logger,
    "%s 肘部构型诊断: mode=diagnostic_only, links=[%s %s %s], "
    "surface_normal=[%.6f %.6f %.6f], samples=%zu, "
    "signed_offset=[start=%+.6f end=%+.6f min=%+.6f at point=%zu "
    "time=%.3f s max=%+.6f] m, min_bend=%.6f m at point=%zu "
    "time=%.3f s, min_observability=%.6f at point=%zu time=%.3f s, "
    "min_shoulder_wrist=%.6f m, side_changes=%zu, ambiguous=%zu, "
    "direction_degenerate=%zu, enforcement=false",
    phase.c_str(), config.shoulder_link.c_str(), config.elbow_link.c_str(),
    config.wrist_link.c_str(), config.surface_normal[0],
    config.surface_normal[1], config.surface_normal[2], metrics.sample_count,
    metrics.start_signed_offset, metrics.end_signed_offset,
    metrics.minimum_signed_offset,
    metrics.minimum_signed_offset_point_index,
    metrics.minimum_signed_offset_point_time, metrics.maximum_signed_offset,
    metrics.minimum_bend_distance, metrics.minimum_bend_point_index,
    metrics.minimum_bend_point_time,
    metrics.minimum_direction_observability,
    metrics.minimum_observability_point_index,
    metrics.minimum_observability_point_time,
    metrics.minimum_shoulder_wrist_distance, metrics.side_change_count,
    metrics.ambiguous_side_sample_count,
    metrics.direction_degenerate_sample_count);
}

#ifndef MASSAGE_PLAN_ONLY_BUILD
struct ExecutionVerificationResult
{
  bool success{false};
  sensor_msgs::msg::JointState final_state;
  std::vector<geometry_msgs::msg::Pose> tcp_trace;
};

ExecutionVerificationResult execute_and_verify(
  const rclcpp::Node::SharedPtr & node,
  massage_motion::MoveItTrajectoryExecutor & executor,
  const std::string & phase,
  const massage_motion::PlanResult & plan,
  const moveit::core::RobotModelConstPtr & robot_model,
  const std::string & tool_link,
  const geometry_msgs::msg::Pose & expected_tool_pose,
  double timeout_margin,
  double endpoint_tolerance,
  double cartesian_position_tolerance,
  double cartesian_orientation_tolerance,
  bool capture_tcp_trace,
  double joint_state_timeout,
  std::mutex & state_mutex,
  std::condition_variable & state_condition,
  const sensor_msgs::msg::JointState & latest_state,
  std::uint64_t & state_sequence,
  bool & record_tcp_trace,
  std::vector<sensor_msgs::msg::JointState> & recorded_states)
{
  ExecutionVerificationResult verification;
  massage_motion::ExecutionTimingPolicy timing_policy;
  timing_policy.margin = timeout_margin;
  const auto timing = massage_motion::calculate_execution_timing(
    plan.trajectory, timing_policy);
  if (!timing.valid)
  {
    throw std::runtime_error(phase + " 无法计算执行时限: " + timing.message);
  }
  RCLCPP_INFO(node->get_logger(), "%s %s", phase.c_str(), timing.message.c_str());

  std::uint64_t previous_sequence = 0U;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    previous_sequence = state_sequence;
    recorded_states.clear();
    if (capture_tcp_trace)
    {
      recorded_states.push_back(latest_state);
    }
    record_tcp_trace = capture_tcp_trace;
  }
  massage_motion::ExecutionRequest request;
  request.request_id = "push_trajectory_sim_" + phase;
  request.robot_trajectory = plan.trajectory;
  request.timeout = timing.timeout;
  const auto result = executor.execute(request);
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    record_tcp_trace = false;
  }
  if (!result.success)
  {
    RCLCPP_ERROR(
      node->get_logger(), "%s 执行失败: status=%d error=%d backend=%d message=%s",
      phase.c_str(), static_cast<int>(result.status),
      static_cast<int>(result.error), result.backend_error_code,
      result.message.c_str());
    return verification;
  }

  sensor_msgs::msg::JointState final_state;
  {
    std::unique_lock<std::mutex> lock(state_mutex);
    if (!state_condition.wait_for(
        lock, std::chrono::duration<double>(joint_state_timeout),
        [&]() {return state_sequence > previous_sequence;}))
    {
      throw std::runtime_error(phase + " 执行后等待 /joint_states 超时");
    }
    final_state = latest_state;
  }
  verification.final_state = final_state;
  const auto endpoint = massage_motion::calculate_trajectory_endpoint_error(
    plan.trajectory, final_state);
  if (!massage_motion::joint_target_reached(endpoint, endpoint_tolerance))
  {
    RCLCPP_ERROR(
      node->get_logger(), "%s 终点校验失败: valid=%s max_error=%.9f rad "
      "tolerance=%.9f rad message=%s",
      phase.c_str(), endpoint.valid ? "true" : "false",
      endpoint.max_absolute_error, endpoint_tolerance,
      endpoint.message.c_str());
    return verification;
  }

  const auto actual_tool_pose = massage_motion::calculate_link_pose(
    robot_model, final_state, tool_link);
  if (!actual_tool_pose.valid)
  {
    RCLCPP_ERROR(
      node->get_logger(), "%s TCP 终点 FK 失败: %s", phase.c_str(),
      actual_tool_pose.message.c_str());
    return verification;
  }
  const auto tool_error = massage_motion::calculate_pose_error(
    expected_tool_pose, actual_tool_pose.pose);
  if (!tool_error.valid ||
    tool_error.translation > cartesian_position_tolerance ||
    tool_error.rotation > cartesian_orientation_tolerance)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "%s TCP 终点校验失败: valid=%s, translation=%.9f m "
      "(tolerance=%.9f m), rotation=%.9f rad (tolerance=%.9f rad), message=%s",
      phase.c_str(), tool_error.valid ? "true" : "false",
      tool_error.translation, cartesian_position_tolerance,
      tool_error.rotation, cartesian_orientation_tolerance,
      tool_error.message.c_str());
    return verification;
  }

  if (capture_tcp_trace)
  {
    std::vector<sensor_msgs::msg::JointState> trace_states;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      trace_states = recorded_states;
    }
    trace_states.push_back(final_state);
    verification.tcp_trace.reserve(trace_states.size());
    for (const auto & state : trace_states)
    {
      const auto pose = massage_motion::calculate_link_pose(
        robot_model, state, tool_link);
      if (!pose.valid)
      {
        RCLCPP_ERROR(
          node->get_logger(), "%s TCP 轨迹 FK 失败: %s", phase.c_str(),
          pose.message.c_str());
        verification.tcp_trace.clear();
        return verification;
      }
      verification.tcp_trace.push_back(pose.pose);
    }
  }
  RCLCPP_INFO(
    node->get_logger(),
    "%s 执行及终点校验通过: joint_max_error=%.9f rad, "
    "tcp_translation_error=%.9f m, tcp_rotation_error=%.9f rad, samples=%zu",
    phase.c_str(), endpoint.max_absolute_error, tool_error.translation,
    tool_error.rotation, verification.tcp_trace.size());
  verification.success = true;
  return verification;
}
#endif

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "push_trajectory_sim",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  std::string config_file;
  std::string execution_environment = "simulation";
  bool parameters_confirmed = false;
  bool execute = false;
  int maximum_ik_attempts = 24;
  int maximum_unique_ik_candidates = 8;
  int ptp_planning_attempts_per_candidate = 2;
  int lin_planning_attempts = 1;
  int ik_random_seed = 684;
  double ik_base_timeout = 0.02;
  double ik_timeout_per_meter = 0.25;
  double ik_timeout_per_radian = 0.05;
  double ik_minimum_timeout = 0.02;
  double ik_maximum_timeout = 0.50;
  double ik_failure_backoff_factor = 1.35;
  double ik_duplicate_tolerance = 1.0e-4;
  double velocity_scale = 0.05;
  double acceleration_scale = 0.05;
  double planning_timeout = 5.0;
  double execution_timeout_margin = 5.0;
  double joint_state_timeout = 3.0;
  double maximum_joint_travel = 3.5;
  double endpoint_tolerance = 0.005;
  bool link_height_gate_enabled = true;
  std::string diagnostic_link_name = "Link_03";
  double minimum_link_height = 0.0;
  double maximum_link_drop_from_standby = 0.20;
  bool elbow_posture_diagnostics_enabled = true;
  std::string shoulder_link_name = "Link_02";
  std::string elbow_link_name = "Link_03";
  std::string wrist_link_name = "Link_04";
  std::string workflow_mode = "full_push";
  std::string test_mode = "normal";
  std::string progressive_checkpoint_fractions = "[0.10, 0.25, 0.50, 1.00]";
  int progressive_maximum_checkpoint_count = 64;
  double progressive_maximum_segment_joint_travel = 0.30;
  double progressive_joint_continuity_tolerance = 1.0e-6;
  double progressive_minimum_direction_observability = 0.10;
  double cartesian_position_tolerance = 0.005;
  double cartesian_orientation_tolerance = 0.03;
  double maximum_lin_transverse_error = 0.005;
  double maximum_lin_height_error = 0.003;
  double maximum_lin_orientation_error = 0.03;
  double maximum_lin_longitudinal_overshoot = 0.005;
  node->get_parameter_or("config_file", config_file, std::string{});
  node->get_parameter_or(
    "execution_environment", execution_environment,
    std::string{"simulation"});
  node->get_parameter_or(
    "parameters_confirmed", parameters_confirmed, false);
  node->get_parameter_or("execute", execute, false);
  node->get_parameter_or("maximum_ik_attempts", maximum_ik_attempts, 24);
  node->get_parameter_or(
    "maximum_unique_ik_candidates", maximum_unique_ik_candidates, 8);
  node->get_parameter_or(
    "ptp_planning_attempts_per_candidate",
    ptp_planning_attempts_per_candidate, 2);
  node->get_parameter_or("lin_planning_attempts", lin_planning_attempts, 1);
  node->get_parameter_or("ik_random_seed", ik_random_seed, 684);
  node->get_parameter_or("ik_base_timeout", ik_base_timeout, 0.02);
  node->get_parameter_or(
    "ik_timeout_per_meter", ik_timeout_per_meter, 0.25);
  node->get_parameter_or(
    "ik_timeout_per_radian", ik_timeout_per_radian, 0.05);
  node->get_parameter_or("ik_minimum_timeout", ik_minimum_timeout, 0.02);
  node->get_parameter_or("ik_maximum_timeout", ik_maximum_timeout, 0.50);
  node->get_parameter_or(
    "ik_failure_backoff_factor", ik_failure_backoff_factor, 1.35);
  node->get_parameter_or(
    "ik_duplicate_tolerance", ik_duplicate_tolerance, 1.0e-4);
  node->get_parameter_or("velocity_scale", velocity_scale, 0.05);
  node->get_parameter_or("acceleration_scale", acceleration_scale, 0.05);
  node->get_parameter_or("planning_timeout", planning_timeout, 5.0);
  node->get_parameter_or(
    "execution_timeout_margin", execution_timeout_margin, 5.0);
  node->get_parameter_or("joint_state_timeout", joint_state_timeout, 3.0);
  node->get_parameter_or("maximum_joint_travel", maximum_joint_travel, 3.5);
  node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.005);
  node->get_parameter_or(
    "link_height_gate_enabled", link_height_gate_enabled, true);
  node->get_parameter_or(
    "diagnostic_link_name", diagnostic_link_name, std::string{"Link_03"});
  node->get_parameter_or(
    "minimum_link_height", minimum_link_height, 0.0);
  node->get_parameter_or(
    "maximum_link_drop_from_standby",
    maximum_link_drop_from_standby, 0.20);
  node->get_parameter_or(
    "elbow_posture_diagnostics_enabled",
    elbow_posture_diagnostics_enabled, true);
  node->get_parameter_or(
    "shoulder_link_name", shoulder_link_name, std::string{"Link_02"});
  node->get_parameter_or(
    "elbow_link_name", elbow_link_name, std::string{"Link_03"});
  node->get_parameter_or(
    "wrist_link_name", wrist_link_name, std::string{"Link_04"});
  node->get_parameter_or(
    "workflow_mode", workflow_mode, std::string{"full_push"});
  node->get_parameter_or("test_mode", test_mode, std::string{"normal"});
  node->get_parameter_or(
    "progressive_checkpoint_fractions", progressive_checkpoint_fractions,
    std::string{"[0.10, 0.25, 0.50, 1.00]"});
  node->get_parameter_or(
    "progressive_maximum_checkpoint_count",
    progressive_maximum_checkpoint_count, 64);
  node->get_parameter_or(
    "progressive_maximum_segment_joint_travel",
    progressive_maximum_segment_joint_travel, 0.30);
  node->get_parameter_or(
    "progressive_joint_continuity_tolerance",
    progressive_joint_continuity_tolerance, 1.0e-6);
  node->get_parameter_or(
    "progressive_minimum_direction_observability",
    progressive_minimum_direction_observability, 0.10);
  node->get_parameter_or(
    "cartesian_position_tolerance", cartesian_position_tolerance, 0.005);
  node->get_parameter_or(
    "cartesian_orientation_tolerance", cartesian_orientation_tolerance, 0.03);
  node->get_parameter_or(
    "maximum_lin_transverse_error", maximum_lin_transverse_error, 0.005);
  node->get_parameter_or(
    "maximum_lin_height_error", maximum_lin_height_error, 0.003);
  node->get_parameter_or(
    "maximum_lin_orientation_error", maximum_lin_orientation_error, 0.03);
  node->get_parameter_or(
    "maximum_lin_longitudinal_overshoot",
    maximum_lin_longitudinal_overshoot, 0.005);

  massage_motion::LinkHeightGateConfig link_height_gate_config;
  link_height_gate_config.enabled = link_height_gate_enabled;
  link_height_gate_config.minimum_z = minimum_link_height;
  link_height_gate_config.maximum_drop_below_reference =
    maximum_link_drop_from_standby;

  massage_motion::IkTimeoutPolicy ik_timeout_policy;
  ik_timeout_policy.base_timeout = ik_base_timeout;
  ik_timeout_policy.timeout_per_meter = ik_timeout_per_meter;
  ik_timeout_policy.timeout_per_radian = ik_timeout_per_radian;
  ik_timeout_policy.minimum_timeout = ik_minimum_timeout;
  ik_timeout_policy.maximum_timeout = ik_maximum_timeout;
  ik_timeout_policy.failure_backoff_factor = ik_failure_backoff_factor;
  std::vector<double> parsed_progressive_fractions;
  try
  {
    parsed_progressive_fractions = parse_fraction_sequence(
      progressive_checkpoint_fractions);
  }
  catch (const std::exception &)
  {
    parsed_progressive_fractions.clear();
  }
  const bool real_plan_only = execution_environment == "real_plan_only";
  const bool progressive_workflow = workflow_mode == "progressive_ptp";
  const bool supported_test_mode =
    (workflow_mode == "full_push" &&
    (test_mode == "normal" || test_mode == "reject_after_ptp")) ||
    (progressive_workflow &&
    (test_mode == "normal" || test_mode == "reject_before_first_segment"));
  if (config_file.empty() ||
    (execution_environment != "simulation" && !real_plan_only) ||
    (real_plan_only && !parameters_confirmed) ||
    (execute && (!kExecutionBackendCompiled || real_plan_only)) ||
    maximum_ik_attempts <= 0 ||
    maximum_unique_ik_candidates <= 0 ||
    ptp_planning_attempts_per_candidate <= 0 || lin_planning_attempts <= 0 ||
    ik_random_seed < 0 ||
    !massage_motion::valid_ik_timeout_policy(ik_timeout_policy) ||
    !std::isfinite(ik_duplicate_tolerance) || ik_duplicate_tolerance <= 0.0 ||
    !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
    velocity_scale > 1.0 || !std::isfinite(acceleration_scale) ||
    acceleration_scale <= 0.0 || acceleration_scale > 1.0 ||
    !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
    !std::isfinite(execution_timeout_margin) || execution_timeout_margin < 0.0 ||
    !std::isfinite(joint_state_timeout) || joint_state_timeout <= 0.0 ||
    !std::isfinite(maximum_joint_travel) || maximum_joint_travel <= 0.0 ||
    !std::isfinite(endpoint_tolerance) || endpoint_tolerance <= 0.0 ||
    diagnostic_link_name.empty() || !std::isfinite(minimum_link_height) ||
    !std::isfinite(maximum_link_drop_from_standby) ||
    maximum_link_drop_from_standby < 0.0 ||
    (elbow_posture_diagnostics_enabled &&
    (shoulder_link_name.empty() || elbow_link_name.empty() ||
    wrist_link_name.empty())) ||
    (execute && !link_height_gate_enabled) ||
    (workflow_mode != "full_push" && !progressive_workflow) ||
    !supported_test_mode ||
    (!execute && test_mode != "normal") ||
    (progressive_workflow &&
    (!link_height_gate_enabled || !elbow_posture_diagnostics_enabled)) ||
    parsed_progressive_fractions.empty() ||
    progressive_maximum_checkpoint_count <= 0 ||
    !std::isfinite(progressive_maximum_segment_joint_travel) ||
    progressive_maximum_segment_joint_travel <= 0.0 ||
    !std::isfinite(progressive_joint_continuity_tolerance) ||
    progressive_joint_continuity_tolerance < 0.0 ||
    !std::isfinite(progressive_minimum_direction_observability) ||
    progressive_minimum_direction_observability < 0.0 ||
    !std::isfinite(cartesian_position_tolerance) ||
    cartesian_position_tolerance <= 0.0 ||
    !std::isfinite(cartesian_orientation_tolerance) ||
    cartesian_orientation_tolerance <= 0.0 ||
    !std::isfinite(maximum_lin_transverse_error) ||
    maximum_lin_transverse_error < 0.0 ||
    !std::isfinite(maximum_lin_height_error) ||
    maximum_lin_height_error < 0.0 ||
    !std::isfinite(maximum_lin_orientation_error) ||
    maximum_lin_orientation_error < 0.0 ||
    !std::isfinite(maximum_lin_longitudinal_overshoot) ||
    maximum_lin_longitudinal_overshoot < 0.0)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "推轨迹参数或执行权限无效: environment=%s, confirmed=%s, "
      "workflow=%s, test_mode=%s, execute_requested=%s, "
      "execution_backend_compiled=%s",
      execution_environment.c_str(), parameters_confirmed ? "true" : "false",
      workflow_mode.c_str(), test_mode.c_str(), execute ? "true" : "false",
      kExecutionBackendCompiled ? "true" : "false");
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "推轨迹执行权限门禁: environment=%s, parameters_confirmed=%s, "
    "workflow=%s, test_mode=%s, execute_requested=%s, "
    "execution_backend_compiled=%s",
    execution_environment.c_str(), parameters_confirmed ? "true" : "false",
    workflow_mode.c_str(), test_mode.c_str(), execute ? "true" : "false",
    kExecutionBackendCompiled ? "true" : "false");

  std::mutex state_mutex;
  std::condition_variable state_condition;
  sensor_msgs::msg::JointState latest_state;
  std::uint64_t state_sequence = 0U;
  bool record_tcp_trace = false;
  std::vector<sensor_msgs::msg::JointState> recorded_states;
  constexpr std::size_t maximum_recorded_states = 20000U;
  auto state_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::JointState::SharedPtr message)
    {
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        latest_state = *message;
        ++state_sequence;
        if (record_tcp_trace && recorded_states.size() < maximum_recorded_states)
        {
          recorded_states.push_back(*message);
        }
      }
      state_condition.notify_all();
    });

  rclcpp::executors::SingleThreadedExecutor ros_executor;
  ros_executor.add_node(node);
  std::thread spin_thread([&ros_executor]() {ros_executor.spin();});

  int exit_code = 1;
  try
  {
    {
      std::unique_lock<std::mutex> lock(state_mutex);
      if (!state_condition.wait_for(
          lock, std::chrono::duration<double>(joint_state_timeout),
          [&]() {return state_sequence > 0U;}))
      {
        throw std::runtime_error(
          "等待 " + execution_environment + " /joint_states 超时");
      }
    }

    auto profile = load_profile(config_file);
    massage_motion::ElbowPostureGeometryConfig elbow_posture_config;
    elbow_posture_config.shoulder_link = shoulder_link_name;
    elbow_posture_config.elbow_link = elbow_link_name;
    elbow_posture_config.wrist_link = wrist_link_name;
    elbow_posture_config.surface_normal = profile.surface_normal;
    massage_motion::ToolOrientationRequest orientation_request;
    orientation_request.surface_normal_world = profile.surface_normal;
    orientation_request.tangent_direction_world = {
      profile.direction_x, profile.direction_y, 0.0};
    const auto tool_orientation =
      massage_motion::make_surface_aligned_tool_orientation(orientation_request);
    if (!tool_orientation.valid)
    {
      throw std::runtime_error(
              "无法从表面法向和推行切向构造工具姿态: " +
              tool_orientation.message);
    }
    profile.start_pose.pose.orientation = tool_orientation.orientation;
    profile.start_pose.header.stamp = node->now();

    massage_motion::PushPathRequest path_request;
    path_request.start_pose = profile.start_pose;
    path_request.direction_x = profile.direction_x;
    path_request.direction_y = profile.direction_y;
    path_request.length = profile.length;
    path_request.speed = profile.speed;
    path_request.sample_period = profile.sample_period;
    path_request.maximum_speed = profile.maximum_speed;
    const auto path = massage_motion::TechniquePathGenerator::generate_push(
      path_request);
    if (!path.success)
    {
      throw std::runtime_error("自由空间推路径生成失败: " + path.message);
    }
    const auto & endpoint_pose = path.path.points.back().pose;
    constexpr double endpoint_config_tolerance = 1.0e-9;
    if (std::abs(endpoint_pose.position.x - profile.expected_end.x) >
      endpoint_config_tolerance ||
      std::abs(endpoint_pose.position.y - profile.expected_end.y) >
      endpoint_config_tolerance ||
      std::abs(endpoint_pose.position.z - profile.expected_end.z) >
      endpoint_config_tolerance)
    {
      throw std::runtime_error("路径生成终点与配置 expected_end_pose 不一致");
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Stage B 配置已验证: execute=%s, start=[%.9f %.9f %.9f] m, "
      "direction_xy=[%.6f %.6f], length=%.3f m, end=[%.9f %.9f %.9f] m, "
      "tool_z_world=[%.6f %.6f %.6f], tool_x_world=[%.6f %.6f %.6f]；"
      "FT/柔顺/接触逻辑均未启用",
      execute ? "true" : "false", profile.start_pose.pose.position.x,
      profile.start_pose.pose.position.y, profile.start_pose.pose.position.z,
      profile.direction_x, profile.direction_y, profile.length,
      endpoint_pose.position.x, endpoint_pose.position.y,
      endpoint_pose.position.z, tool_orientation.tool_z_world[0],
      tool_orientation.tool_z_world[1], tool_orientation.tool_z_world[2],
      tool_orientation.tool_x_world[0], tool_orientation.tool_x_world[1],
      tool_orientation.tool_x_world[2]);
    RCLCPP_INFO(
      node->get_logger(),
      "Stage C-B 执行验收配置: test_mode=%s, joint_tolerance=%.6f rad, "
      "tcp_position_tolerance=%.6f m, tcp_orientation_tolerance=%.6f rad, "
      "lin_transverse=%.6f m, lin_height=%.6f m, lin_orientation=%.6f rad, "
      "lin_overshoot=%.6f m",
      test_mode.c_str(), endpoint_tolerance, cartesian_position_tolerance,
      cartesian_orientation_tolerance, maximum_lin_transverse_error,
      maximum_lin_height_error, maximum_lin_orientation_error,
      maximum_lin_longitudinal_overshoot);
    RCLCPP_INFO(
      node->get_logger(),
      "肘部构型诊断配置: enabled=%s, mode=diagnostic_only, "
      "links=[%s %s %s], surface_normal=[%.6f %.6f %.6f], "
      "enforcement=false；法向来自业务接触配置",
      elbow_posture_diagnostics_enabled ? "true" : "false",
      shoulder_link_name.c_str(), elbow_link_name.c_str(),
      wrist_link_name.c_str(), profile.surface_normal[0],
      profile.surface_normal[1], profile.surface_normal[2]);

    massage_motion::PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "massage_tool_tip";
    planner_config.reference_frame = profile.start_pose.header.frame_id;
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";
    auto sdk = std::make_shared<massage_motion::MotionPlanningSdk>(
      node, planner_config);
    // 显式 IK 候选生成依赖本地 RobotModel 上的求解器实例。第三个参数必须
    // 为 true，否则只解析 URDF/SRDF，不会加载 robot_description_kinematics。
    robot_model_loader::RobotModelLoader robot_model_loader(
      node, "robot_description", true);
    const auto robot_model = robot_model_loader.getModel();
    if (!robot_model)
    {
      throw std::runtime_error("无法加载 RobotModel 进行候选诊断");
    }
    std::vector<std::string> standby_joint_names;
    standby_joint_names.reserve(massage_bringup::kJakaS5JointNames.size());
    for (const auto * joint_name : massage_bringup::kJakaS5JointNames)
    {
      standby_joint_names.emplace_back(joint_name);
    }
    const std::vector<double> standby_joint_positions(
      massage_bringup::kMassageHomeJointPositions.begin(),
      massage_bringup::kMassageHomeJointPositions.end());
    const auto standby_link_height =
      massage_motion::calculate_link_height_at_joint_target(
      robot_model, standby_joint_names, standby_joint_positions,
      diagnostic_link_name);
    if (!standby_link_height.valid)
    {
      throw std::runtime_error(
        "无法计算业务待机位连杆参考高度: " +
        standby_link_height.message);
    }
    link_height_gate_config.reference_z = standby_link_height.z;
    RCLCPP_INFO(
      node->get_logger(),
      "Stage C-A 几何门禁: enabled=%s, link=%s, reference=business_standby_fk, "
      "standby_z=%.6f m, maximum_drop_from_standby=%.6f m, "
      "drop_limit_z=%.6f m, absolute_minimum_z=%.6f m；"
      "参考高度由业务待机关节常量和 RobotModel FK 计算，与启动位置无关",
      link_height_gate_enabled ? "true" : "false",
      diagnostic_link_name.c_str(), standby_link_height.z,
      maximum_link_drop_from_standby,
      standby_link_height.z - maximum_link_drop_from_standby,
      minimum_link_height);
    sensor_msgs::msg::JointState planning_start_state;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      planning_start_state = latest_state;
    }

    massage_motion::PoseIkCandidateGeneratorConfig ik_config;
    ik_config.maximum_attempts = static_cast<std::size_t>(maximum_ik_attempts);
    ik_config.maximum_unique_candidates =
      static_cast<std::size_t>(maximum_unique_ik_candidates);
    ik_config.duplicate_tolerance = ik_duplicate_tolerance;
    ik_config.random_seed = static_cast<std::uint32_t>(ik_random_seed);
    ik_config.timeout_policy = ik_timeout_policy;
    massage_motion::PoseIkCandidateGenerator ik_generator(
      robot_model, planner_config.planning_group,
      planner_config.end_effector_link, ik_config);
    const auto ik_report = ik_generator.generate(
      planning_start_state, profile.start_pose);
    log_ik_report(node->get_logger(), ik_report);
    if (!ik_report.success)
    {
      throw std::runtime_error(
        "到自由空间起点的显式 IK 搜索失败: " + ik_report.message);
    }

    massage_motion::PlanCompetitionConfig competition_config;
    competition_config.maximum_joint_travel = maximum_joint_travel;

    massage_motion::MotionRequest ptp_request;
    ptp_request.request_id = "push_stage_b_approach";
    ptp_request.motion_type = massage_motion::MotionType::kPtp;
    ptp_request.velocity_scale = velocity_scale;
    ptp_request.acceleration_scale = acceleration_scale;
    ptp_request.planning_timeout = planning_timeout;
    const auto ordered_planning_start =
      massage_motion::order_joint_state_positions(
        ik_report.variable_names, planning_start_state);
    if (!ordered_planning_start.valid)
    {
      throw std::runtime_error(
        "无法固定 PTP 候选的共同起点: " + ordered_planning_start.message);
    }
    moveit_msgs::msg::RobotState explicit_ptp_start;
    explicit_ptp_start.joint_state.name = ik_report.variable_names;
    explicit_ptp_start.joint_state.position = ordered_planning_start.positions;
    explicit_ptp_start.is_diff = true;
    ptp_request.start_state = explicit_ptp_start;
    massage_motion::PlanResult ptp_plan;
    double best_ptp_score = std::numeric_limits<double>::infinity();
    std::size_t selected_ik_candidate = std::numeric_limits<std::size_t>::max();
    std::size_t planned_ik_candidates = 0U;
    std::size_t geometry_accepted_candidates = 0U;
    std::size_t geometry_rejected_candidates = 0U;
    std::size_t elbow_diagnostic_candidates = 0U;
    massage_motion::ElbowPostureMetrics selected_ptp_elbow_metrics;
    std::vector<std::string> ptp_rejection_reasons;
    for (std::size_t ik_index = 0;
      ik_index < ik_report.candidates.size(); ++ik_index)
    {
      massage_motion::CompetitiveMotionPlanner candidate_planner(
        std::vector<massage_motion::PlanningSource>{
          {"motion_sdk", sdk, static_cast<std::size_t>(
              ptp_planning_attempts_per_candidate)}},
        competition_config);
      ptp_request.request_id = "push_stage_b_approach_ik_" +
        std::to_string(ik_index + 1U);
      ptp_request.target = massage_motion::JointTarget{
        ik_report.candidates[ik_index].positions};
      const auto candidate_plan = candidate_planner.plan(ptp_request);
      const auto candidate_report = candidate_planner.last_report();
      const std::string phase = "PTP IK[" + std::to_string(ik_index) + "]";
      log_competition(
        node->get_logger(), phase, candidate_report, maximum_joint_travel,
        robot_model);
      if (!candidate_plan.success || !candidate_report.success)
      {
        std::string reason = candidate_plan.message;
        if (reason.empty() && !candidate_report.candidates.empty())
        {
          reason = candidate_report.candidates.back().message;
        }
        ptp_rejection_reasons.push_back(
          phase + " planning=" +
          (reason.empty() ? "unknown failure" : reason));
        continue;
      }
      ++planned_ik_candidates;

      massage_motion::ElbowPostureMetrics elbow_metrics;
      if (elbow_posture_diagnostics_enabled)
      {
        elbow_metrics = massage_motion::calculate_elbow_posture_metrics(
          robot_model, candidate_plan.trajectory, elbow_posture_config);
        log_elbow_posture_metrics(
          node->get_logger(), phase, elbow_metrics, elbow_posture_config);
        if (!elbow_metrics.valid)
        {
          throw std::runtime_error(
            phase + " 肘部构型诊断无法生成有效证据: " +
            elbow_metrics.message);
        }
        ++elbow_diagnostic_candidates;
      }

      const auto link_metrics = massage_motion::calculate_link_height_metrics(
        robot_model, candidate_plan.trajectory, diagnostic_link_name);
      if (!link_metrics.valid)
      {
        RCLCPP_WARN(
          node->get_logger(), "%s %s 全轨迹诊断失败: %s",
          phase.c_str(), diagnostic_link_name.c_str(),
          link_metrics.message.c_str());
      }
      else
      {
        RCLCPP_INFO(
          node->get_logger(),
          "%s %s 全轨迹诊断: start_z=%.6f m, end_z=%.6f m, "
          "min_z=%.6f m at point=%zu time=%.3f s, max_z=%.6f m, "
          "drop_below_start=%.6f m, drop_below_standby=%.6f m",
          phase.c_str(), diagnostic_link_name.c_str(),
          link_metrics.start_z, link_metrics.end_z,
          link_metrics.minimum_z, link_metrics.minimum_point_index,
          link_metrics.minimum_point_time, link_metrics.maximum_z,
          link_metrics.maximum_drop_below_start,
          standby_link_height.z - link_metrics.minimum_z);
      }

      const auto height_gate = massage_motion::evaluate_link_height_gate(
        link_metrics, link_height_gate_config);
      RCLCPP_INFO(
        node->get_logger(),
        "%s 几何门禁: %s: valid=%s, link=%s, min_z=%.6f m "
        "(absolute_limit=%.6f m, effective_limit=%.6f m), "
        "drop_below_standby=%.6f m (limit=%.6f m), message=%s",
        phase.c_str(), height_gate.accepted ? "ACCEPTED" : "REJECTED",
        height_gate.valid ? "true" : "false", diagnostic_link_name.c_str(),
        link_metrics.minimum_z, minimum_link_height,
        height_gate.minimum_allowed_z, height_gate.drop_below_reference,
        maximum_link_drop_from_standby,
        height_gate.message.c_str());
      if (!height_gate.accepted)
      {
        ++geometry_rejected_candidates;
        ptp_rejection_reasons.push_back(
          phase + " geometry=" + height_gate.message);
        continue;
      }
      ++geometry_accepted_candidates;

      const auto & selected = candidate_report.candidates[
        candidate_report.selected_candidate];
      if (selected.score < best_ptp_score)
      {
        best_ptp_score = selected.score;
        ptp_plan = candidate_plan;
        selected_ik_candidate = ik_index;
        selected_ptp_elbow_metrics = elbow_metrics;
      }
    }
    if (!ptp_plan.success)
    {
      std::ostringstream failure;
      failure << "到自由空间起点的 PTP 规划失败";
      if (!ptp_rejection_reasons.empty())
      {
        failure << ": ";
        for (std::size_t index = 0; index < ptp_rejection_reasons.size(); ++index)
        {
          if (index > 0U)
          {
            failure << "; ";
          }
          failure << ptp_rejection_reasons[index];
        }
      }
      throw std::runtime_error(failure.str());
    }
    RCLCPP_INFO(
      node->get_logger(),
      "PTP 唯一 IK 分支竞争完成: selected_ik=%zu, score=%.6f, "
      "unique_ik=%zu, planning_attempts_per_candidate=%d",
      selected_ik_candidate, best_ptp_score, ik_report.candidates.size(),
      ptp_planning_attempts_per_candidate);
    RCLCPP_INFO(
      node->get_logger(),
      "Stage C-A 几何门禁汇总: planned=%zu, accepted=%zu, rejected=%zu, "
      "selected_ik=%zu",
      planned_ik_candidates, geometry_accepted_candidates,
      geometry_rejected_candidates, selected_ik_candidate);
    const auto & selected_goal =
      ik_report.candidates.at(selected_ik_candidate).positions;
    for (std::size_t joint_index = 0;
      joint_index < selected_goal.size(); ++joint_index)
    {
      RCLCPP_INFO(
        node->get_logger(), "Stage C-A 最终 PTP 目标 %s=%+.9f rad",
        ik_report.variable_names[joint_index].c_str(),
        selected_goal[joint_index]);
    }

    if (progressive_workflow)
    {
      massage_motion::ProgressiveJointCheckpointConfig checkpoint_config;
      checkpoint_config.required_fractions = parsed_progressive_fractions;
      checkpoint_config.maximum_joint_step =
        progressive_maximum_segment_joint_travel;
      checkpoint_config.maximum_checkpoint_count =
        static_cast<std::size_t>(progressive_maximum_checkpoint_count);
      const auto checkpoints =
        massage_motion::generate_progressive_joint_checkpoints(
        ptp_plan.trajectory, checkpoint_config);
      if (!checkpoints.valid || checkpoints.checkpoints.empty())
      {
        throw std::runtime_error(
          "渐进 PTP 检查点生成失败: " + checkpoints.message);
      }
      RCLCPP_INFO(
        node->get_logger(),
        "渐进 PTP 检查点汇总: count=%zu, required=%zu, "
        "joint_path=%.9f rad, maximum_generated_step=%.9f rad, limit=%.9f rad",
        checkpoints.checkpoints.size(), parsed_progressive_fractions.size(),
        checkpoints.joint_path_length,
        checkpoints.maximum_generated_joint_step,
        progressive_maximum_segment_joint_travel);
      for (std::size_t index = 0; index < checkpoints.checkpoints.size(); ++index)
      {
        const auto & checkpoint = checkpoints.checkpoints[index];
        RCLCPP_INFO(
          node->get_logger(),
          "渐进检查点[%zu]: fraction=%.6f, required=%s, source_segment=%zu, "
          "source_time=%.6f s, max_joint_step=%.9f rad",
          index, checkpoint.fraction, checkpoint.required ? "true" : "false",
          checkpoint.source_segment_index, checkpoint.source_time,
          checkpoint.maximum_joint_step_from_previous);
      }

      massage_motion::PlanCompetitionConfig segment_competition_config;
      segment_competition_config.maximum_joint_travel =
        progressive_maximum_segment_joint_travel;
      const int reference_elbow_side = signed_side(
        selected_ptp_elbow_metrics.start_signed_offset,
        elbow_posture_config.side_tolerance);
      std::vector<massage_motion::PlanResult> segment_plans;
      segment_plans.reserve(checkpoints.checkpoints.size());
      std::vector<double> previous_target = checkpoints.start_positions;

      const auto validate_progressive_plan =
        [&](const std::string & phase,
          const massage_motion::PlanResult & plan,
          const std::vector<double> & expected_start,
          const std::vector<double> & expected_end)
        {
          const auto & trajectory = plan.trajectory.joint_trajectory;
          if (trajectory.joint_names != checkpoints.joint_names ||
            trajectory.points.empty())
          {
            throw std::runtime_error(
              phase + " 轨迹关节顺序变化或轨迹为空");
          }
          const double start_error = maximum_position_difference(
            trajectory.points.front().positions, expected_start);
          const double end_error = maximum_position_difference(
            trajectory.points.back().positions, expected_end);
          if (start_error > progressive_joint_continuity_tolerance ||
            end_error > progressive_joint_continuity_tolerance)
          {
            std::ostringstream message;
            message << phase << " 关节连续性失败: start_error=" << start_error
                    << " rad, end_error=" << end_error << " rad, tolerance="
                    << progressive_joint_continuity_tolerance << " rad";
            throw std::runtime_error(message.str());
          }
          const auto trajectory_metrics =
            massage_motion::calculate_trajectory_metrics(plan.trajectory);
          if (!trajectory_metrics.valid ||
            trajectory_metrics.maximum_joint_travel >
            progressive_maximum_segment_joint_travel + 1.0e-12)
          {
            throw std::runtime_error(
              phase + " 超过渐进分段最大单关节行程");
          }
          const auto height_metrics =
            massage_motion::calculate_link_height_metrics(
            robot_model, plan.trajectory, diagnostic_link_name);
          const auto height_gate = massage_motion::evaluate_link_height_gate(
            height_metrics, link_height_gate_config);
          if (!height_gate.accepted)
          {
            throw std::runtime_error(
              phase + " Link_03 高度门禁失败: " + height_gate.message);
          }
          const auto elbow_metrics =
            massage_motion::calculate_elbow_posture_metrics(
            robot_model, plan.trajectory, elbow_posture_config);
          log_elbow_posture_metrics(
            node->get_logger(), phase, elbow_metrics, elbow_posture_config);
          if (!elbow_metrics.valid)
          {
            throw std::runtime_error(
              phase + " 肘部构型诊断失败: " + elbow_metrics.message);
          }
          const bool direction_observable =
            elbow_metrics.minimum_direction_observability >=
            progressive_minimum_direction_observability;
          if (direction_observable)
          {
            const int start_side = signed_side(
              elbow_metrics.start_signed_offset,
              elbow_posture_config.side_tolerance);
            const int end_side = signed_side(
              elbow_metrics.end_signed_offset,
              elbow_posture_config.side_tolerance);
            if (reference_elbow_side == 0 || start_side != reference_elbow_side ||
              end_side != reference_elbow_side ||
              elbow_metrics.side_change_count != 0U)
            {
              throw std::runtime_error(
                phase + " 可观测区间发生肘侧改变或参考肘侧不明确");
            }
          }
          RCLCPP_INFO(
            node->get_logger(),
            "%s 渐进门禁: ACCEPTED: joint_start_error=%.9f rad, "
            "joint_end_error=%.9f rad, maximum_joint_travel=%.9f rad, "
            "link_min_z=%.6f m, elbow_reference_side=%d, "
            "minimum_observability=%.6f, posture_policy=%s",
            phase.c_str(), start_error, end_error,
            trajectory_metrics.maximum_joint_travel,
            height_metrics.minimum_z, reference_elbow_side,
            elbow_metrics.minimum_direction_observability,
            direction_observable ? "elbow_side" : "joint_continuity_fallback");
        };

      for (std::size_t index = 0; index < checkpoints.checkpoints.size(); ++index)
      {
        const auto & checkpoint = checkpoints.checkpoints[index];
        massage_motion::MotionRequest segment_request;
        segment_request.request_id = "push_progressive_segment_" +
          std::to_string(index + 1U);
        segment_request.motion_type = massage_motion::MotionType::kPtp;
        segment_request.target = massage_motion::JointTarget{checkpoint.positions};
        segment_request.velocity_scale = velocity_scale;
        segment_request.acceleration_scale = acceleration_scale;
        segment_request.planning_timeout = planning_timeout;
        segment_request.start_state = joint_target_state(
          checkpoints.joint_names, previous_target);
        massage_motion::CompetitiveMotionPlanner segment_planner(
          std::vector<massage_motion::PlanningSource>{
            {"motion_sdk", sdk, static_cast<std::size_t>(
                ptp_planning_attempts_per_candidate)}},
          segment_competition_config);
        const auto segment_plan = segment_planner.plan(segment_request);
        const std::string phase = "PROGRESSIVE PTP segment[" +
          std::to_string(index) + "]";
        log_competition(
          node->get_logger(), phase, segment_planner.last_report(),
          progressive_maximum_segment_joint_travel, robot_model);
        if (!segment_plan.success)
        {
          throw std::runtime_error(
            phase + " 重新规划失败: " + segment_plan.message);
        }
        validate_progressive_plan(
          phase, segment_plan, previous_target, checkpoint.positions);
        segment_plans.push_back(segment_plan);
        previous_target = checkpoint.positions;
      }

      massage_motion::MotionRequest return_request;
      return_request.request_id = "push_progressive_first_segment_return";
      return_request.motion_type = massage_motion::MotionType::kPtp;
      return_request.target = massage_motion::JointTarget{
        checkpoints.start_positions};
      return_request.velocity_scale = velocity_scale;
      return_request.acceleration_scale = acceleration_scale;
      return_request.planning_timeout = planning_timeout;
      return_request.start_state = terminal_state(
        segment_plans.front().trajectory);
      massage_motion::CompetitiveMotionPlanner return_planner(
        std::vector<massage_motion::PlanningSource>{
          {"motion_sdk", sdk, static_cast<std::size_t>(
              ptp_planning_attempts_per_candidate)}},
        segment_competition_config);
      const auto return_plan = return_planner.plan(return_request);
      log_competition(
        node->get_logger(), "PROGRESSIVE PTP return",
        return_planner.last_report(), progressive_maximum_segment_joint_travel,
        robot_model);
      if (!return_plan.success)
      {
        throw std::runtime_error(
          "渐进 PTP 第一段返程预规划失败: " + return_plan.message);
      }
      validate_progressive_plan(
        "PROGRESSIVE PTP return", return_plan,
        segment_plans.front().trajectory.joint_trajectory.points.back().positions,
        checkpoints.start_positions);

      if (!execute)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "PROGRESSIVE PTP PLAN-ONLY: PASS: checkpoints=%zu, segments=%zu, "
          "first_fraction=%.6f, first_max_joint_step=%.9f rad, "
          "return_preplanned=true；未发送运动命令",
          checkpoints.checkpoints.size(), segment_plans.size(),
          checkpoints.checkpoints.front().fraction,
          checkpoints.checkpoints.front().maximum_joint_step_from_previous);
        exit_code = 0;
      }
#ifndef MASSAGE_PLAN_ONLY_BUILD
      else if (test_mode == "reject_before_first_segment")
      {
        RCLCPP_INFO(
          node->get_logger(),
          "PROGRESSIVE PTP FAILURE INJECTION: PASS: "
          "outbound_dispatch_count=0, return_dispatch_count=0；"
          "第一段发送前已按测试要求拒绝");
        exit_code = 0;
      }
      else
      {
        const auto make_expected_pose =
          [&](const std::vector<double> & positions)
          {
            sensor_msgs::msg::JointState state;
            state.name = checkpoints.joint_names;
            state.position = positions;
            const auto pose = massage_motion::calculate_link_pose(
              robot_model, state, planner_config.end_effector_link);
            if (!pose.valid)
            {
              throw std::runtime_error(
                "无法计算渐进 PTP 自动验收目标 TCP: " + pose.message);
            }
            return pose.pose;
          };
        const auto first_target_pose = make_expected_pose(
          segment_plans.front().trajectory.joint_trajectory.points.back().positions);
        const auto original_pose = make_expected_pose(checkpoints.start_positions);
        massage_motion::MoveItTrajectoryExecutor trajectory_executor(node);
        std::size_t outbound_dispatch_count = 1U;
        std::size_t return_dispatch_count = 0U;
        const auto outbound = execute_and_verify(
          node, trajectory_executor, "PROGRESSIVE_PTP_FIRST_OUTBOUND",
          segment_plans.front(), robot_model, planner_config.end_effector_link,
          first_target_pose, execution_timeout_margin, endpoint_tolerance,
          cartesian_position_tolerance, cartesian_orientation_tolerance, false,
          joint_state_timeout, state_mutex, state_condition, latest_state,
          state_sequence, record_tcp_trace, recorded_states);
        if (!outbound.success)
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "渐进 PTP 第一段执行失败: outbound_dispatch_count=%zu, "
            "return_dispatch_count=%zu",
            outbound_dispatch_count, return_dispatch_count);
          exit_code = 3;
        }
        else
        {
          const auto & return_start =
            return_plan.trajectory.joint_trajectory.points.front();
          const auto return_start_error =
            massage_motion::calculate_joint_target_error(
            return_plan.trajectory.joint_trajectory.joint_names,
            return_start.positions, outbound.final_state);
          if (!massage_motion::joint_target_reached(
              return_start_error, endpoint_tolerance))
          {
            RCLCPP_ERROR(
              node->get_logger(),
              "渐进 PTP 返程发送前连续性失败: max_error=%.9f rad, "
              "return_dispatch_count=0",
              return_start_error.max_absolute_error);
            exit_code = 3;
          }
          else
          {
            ++return_dispatch_count;
            const auto returned = execute_and_verify(
              node, trajectory_executor, "PROGRESSIVE_PTP_FIRST_RETURN",
              return_plan, robot_model, planner_config.end_effector_link,
              original_pose, execution_timeout_margin, endpoint_tolerance,
              cartesian_position_tolerance, cartesian_orientation_tolerance,
              false, joint_state_timeout, state_mutex, state_condition,
              latest_state, state_sequence, record_tcp_trace, recorded_states);
            if (!returned.success)
            {
              exit_code = 4;
            }
            else
            {
              RCLCPP_INFO(
                node->get_logger(),
                "PROGRESSIVE PTP FIRST-SEGMENT ROUND-TRIP: PASS: "
                "first_fraction=%.6f, outbound_dispatch_count=%zu, "
                "return_dispatch_count=%zu",
                checkpoints.checkpoints.front().fraction,
                outbound_dispatch_count, return_dispatch_count);
              exit_code = 0;
            }
          }
        }
      }
#endif
    }
    else
    {
    massage_motion::MotionRequest lin_request;
    lin_request.request_id = "push_stage_b_linear";
    lin_request.motion_type = massage_motion::MotionType::kLin;
    geometry_msgs::msg::PoseStamped endpoint_stamped;
    endpoint_stamped.header = profile.start_pose.header;
    endpoint_stamped.pose = endpoint_pose;
    lin_request.target = massage_motion::PoseTarget{endpoint_stamped};
    lin_request.velocity_scale = velocity_scale;
    lin_request.acceleration_scale = acceleration_scale;
    lin_request.planning_timeout = planning_timeout;
    lin_request.start_state = terminal_state(ptp_plan.trajectory);
    massage_motion::CompetitiveMotionPlanner lin_planner(
      std::vector<massage_motion::PlanningSource>{
        {"motion_sdk", sdk, static_cast<std::size_t>(lin_planning_attempts)}},
      competition_config);
    const auto lin_plan = lin_planner.plan(lin_request);
    log_competition(
      node->get_logger(), "LIN", lin_planner.last_report(), maximum_joint_travel,
      robot_model);
    if (!lin_plan.success)
    {
      throw std::runtime_error("自由空间 +Y LIN 规划失败: " + lin_plan.message);
    }
    massage_motion::ElbowPostureMetrics lin_elbow_metrics;
    if (elbow_posture_diagnostics_enabled)
    {
      lin_elbow_metrics = massage_motion::calculate_elbow_posture_metrics(
        robot_model, lin_plan.trajectory, elbow_posture_config);
      log_elbow_posture_metrics(
        node->get_logger(), "LIN", lin_elbow_metrics, elbow_posture_config);
      if (!lin_elbow_metrics.valid || !selected_ptp_elbow_metrics.valid)
      {
        throw std::runtime_error(
          "选中 PTP 或 LIN 缺少有效肘部构型诊断证据");
      }
      RCLCPP_INFO(
        node->get_logger(),
        "ELBOW POSTURE DIAGNOSTICS: COMPLETE: enforcement=false, "
        "planned_ptp=%zu, diagnosed_ptp=%zu, selected_ik=%zu, "
        "selected_ptp_min_signed=%+.6f m, selected_ptp_min_bend=%.6f m, "
        "selected_ptp_side_changes=%zu, lin_min_signed=%+.6f m, "
        "lin_min_bend=%.6f m, lin_side_changes=%zu",
        planned_ik_candidates, elbow_diagnostic_candidates,
        selected_ik_candidate,
        selected_ptp_elbow_metrics.minimum_signed_offset,
        selected_ptp_elbow_metrics.minimum_bend_distance,
        selected_ptp_elbow_metrics.side_change_count,
        lin_elbow_metrics.minimum_signed_offset,
        lin_elbow_metrics.minimum_bend_distance,
        lin_elbow_metrics.side_change_count);
    }

    if (!execute)
    {
      const char * result_label = real_plan_only ?
        "REAL PUSH TRAJECTORY PLAN-ONLY" : "PUSH TRAJECTORY PLAN-ONLY";
      RCLCPP_INFO(
        node->get_logger(),
        "%s: PASS: selected_ik=%zu, PTP points=%zu, LIN points=%zu, "
        "geometry_accepted=%zu, geometry_rejected=%zu, "
        "execution_backend_compiled=%s；连续规划使用 PTP 终点作为 LIN "
        "起点；未发送运动命令",
        result_label, selected_ik_candidate,
        ptp_plan.trajectory.joint_trajectory.points.size(),
        lin_plan.trajectory.joint_trajectory.points.size(),
        geometry_accepted_candidates, geometry_rejected_candidates,
        kExecutionBackendCompiled ? "true" : "false");
      exit_code = 0;
    }
#ifndef MASSAGE_PLAN_ONLY_BUILD
    else
    {
      massage_motion::MoveItTrajectoryExecutor trajectory_executor(node);
      std::size_t lin_dispatch_count = 0U;
      const auto ptp_verification = execute_and_verify(
        node, trajectory_executor, "PTP", ptp_plan, robot_model,
        planner_config.end_effector_link, profile.start_pose.pose,
        execution_timeout_margin, endpoint_tolerance,
        cartesian_position_tolerance, cartesian_orientation_tolerance, false,
        joint_state_timeout, state_mutex, state_condition, latest_state,
        state_sequence, record_tcp_trace, recorded_states);
      if (!ptp_verification.success)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "Stage C-B 分段门禁: PTP 未通过，LIN dispatch_count=%zu",
          lin_dispatch_count);
        exit_code = 3;
      }
      else if (test_mode == "reject_after_ptp")
      {
        RCLCPP_INFO(
          node->get_logger(),
          "STAGE C-B FAILURE INJECTION: PASS: 已注入 PTP 后验收拒绝，"
          "LIN dispatch_count=%zu；第二段未发送",
          lin_dispatch_count);
        exit_code = 0;
      }
      else
      {
        sensor_msgs::msg::JointState state_before_lin;
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          state_before_lin = latest_state;
        }
        const auto & lin_start = lin_plan.trajectory.joint_trajectory.points.front();
        const auto lin_start_error = massage_motion::calculate_joint_target_error(
          lin_plan.trajectory.joint_trajectory.joint_names,
          lin_start.positions, state_before_lin);
        if (!massage_motion::joint_target_reached(
            lin_start_error, endpoint_tolerance))
        {
          RCLCPP_ERROR(
            node->get_logger(),
            "Stage C-B 分段门禁: PTP 实际终点与预规划 LIN 起点不一致，"
            "max_error=%.9f rad, LIN dispatch_count=%zu；第二段未发送",
            lin_start_error.max_absolute_error, lin_dispatch_count);
          exit_code = 3;
        }
        else
        {
          ++lin_dispatch_count;
          const auto lin_verification = execute_and_verify(
            node, trajectory_executor, "LIN", lin_plan, robot_model,
            planner_config.end_effector_link, endpoint_pose,
            execution_timeout_margin, endpoint_tolerance,
            cartesian_position_tolerance, cartesian_orientation_tolerance, true,
            joint_state_timeout, state_mutex, state_condition, latest_state,
            state_sequence, record_tcp_trace, recorded_states);
          if (!lin_verification.success)
          {
            exit_code = 4;
          }
          else
          {
            massage_motion::CartesianLineTraceConfig trace_config;
            trace_config.maximum_endpoint_position_error =
              cartesian_position_tolerance;
            trace_config.maximum_endpoint_orientation_error =
              cartesian_orientation_tolerance;
            trace_config.maximum_transverse_error =
              maximum_lin_transverse_error;
            trace_config.maximum_height_error = maximum_lin_height_error;
            trace_config.maximum_orientation_error =
              maximum_lin_orientation_error;
            trace_config.maximum_longitudinal_overshoot =
              maximum_lin_longitudinal_overshoot;
            const auto trace = massage_motion::evaluate_cartesian_line_trace(
              lin_verification.tcp_trace, profile.start_pose.pose,
              endpoint_pose, trace_config);
            RCLCPP_INFO(
              node->get_logger(),
              "Stage C-B LIN 世界系轨迹验收: %s: valid=%s, samples=%zu, "
              "expected_length=%.9f m, final_progress=%.9f m, "
              "progress_range=[%.9f, %.9f] m, endpoint_position=%.9f m, "
              "endpoint_orientation=%.9f rad, max_transverse=%.9f m, "
              "max_height=%.9f m, max_orientation=%.9f rad, message=%s",
              trace.accepted ? "ACCEPTED" : "REJECTED",
              trace.valid ? "true" : "false", trace.sample_count,
              trace.expected_length, trace.final_directed_progress,
              trace.minimum_directed_progress,
              trace.maximum_directed_progress,
              trace.endpoint_position_error,
              trace.endpoint_orientation_error,
              trace.maximum_transverse_error,
              trace.maximum_height_error,
              trace.maximum_orientation_error, trace.message.c_str());
            if (!trace.accepted)
            {
              exit_code = 4;
            }
            else
            {
              RCLCPP_INFO(
                node->get_logger(),
                "PUSH TRAJECTORY SIM EXECUTION: PASS: free-space=true, "
                "contact=false, compliance=false, direction=world +Y, "
                "length=0.050 m, LIN dispatch_count=%zu",
                lin_dispatch_count);
              exit_code = 0;
            }
          }
        }
      }
    }
#endif
    }
  }
  catch (const YAML::Exception & exception)
  {
    RCLCPP_ERROR(node->get_logger(), "读取 Stage B YAML 失败: %s", exception.what());
    exit_code = 5;
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(node->get_logger(), "Stage B 推轨迹失败: %s", exception.what());
    exit_code = 6;
  }

  (void)state_subscription;
  ros_executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
