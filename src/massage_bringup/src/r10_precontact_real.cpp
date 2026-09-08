#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "jaka_msgs/msg/robot_msg.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/execution_timing.hpp"
#include "massage_motion/guarded_trajectory_executor.hpp"
#include "massage_motion/link_trajectory_geometry.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/pose_ik_candidate_generator.hpp"
#include "massage_motion/tool_orientation.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"

namespace
{

using SteadyClock = std::chrono::steady_clock;

bool robot_ready(const jaka_msgs::msg::RobotMsg & state)
{
  return state.motion_state == 0 && state.power_state == 1 &&
         state.servo_state == 1 && state.collision_state == 0;
}

moveit_msgs::msg::RobotState terminal_state(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  moveit_msgs::msg::RobotState state;
  if (trajectory.joint_trajectory.points.empty())
  {
    return state;
  }
  state.joint_state.name = trajectory.joint_trajectory.joint_names;
  state.joint_state.position =
    trajectory.joint_trajectory.points.back().positions;
  state.is_diff = true;
  return state;
}

void require_alignment(
  const moveit::core::RobotModelConstPtr & model,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const geometry_msgs::msg::Quaternion & expected_orientation,
  bool inspect_all_samples, double maximum_axis_error,
  const std::string & phase, const rclcpp::Logger & logger)
{
  massage_motion::ToolAlignmentGateConfig gate;
  gate.desired_tool_z_world = {0.0, 0.0, -1.0};
  gate.expected_orientation = expected_orientation;
  gate.maximum_axis_error = maximum_axis_error;
  gate.inspect_all_samples = inspect_all_samples;
  const auto metrics = massage_motion::evaluate_tool_alignment_trajectory(
    model, trajectory, "massage_tool_tip", gate);
  if (!metrics.valid || !metrics.accepted)
  {
    throw std::runtime_error(
            phase + " 姿态门禁失败: " + metrics.message);
  }
  RCLCPP_INFO(
    logger,
    "R10 PRECONTACT ALIGNMENT [%s]: PASS: samples=%zu, "
    "maximum_axis_error_deg=%.6f, orientation_drift_deg=%.6f",
    phase.c_str(), metrics.sample_count,
    metrics.maximum_axis_error * 180.0 / M_PI,
    metrics.maximum_orientation_drift * 180.0 / M_PI);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "r10_precontact_real",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  bool execute = false;
  bool parameters_confirmed = false;
  int planning_attempts = 3;
  int maximum_ik_attempts = 32;
  int maximum_unique_ik_candidates = 8;
  double contact_x = -0.471238630147741;
  double contact_y = 0.156275068796867;
  double contact_z = 0.281381721182422;
  double precontact_clearance = 0.002;
  double work_ready_clearance = 0.050;
  double velocity_scale = 0.05;
  double acceleration_scale = 0.05;
  double planning_timeout = 30.0;
  double execution_timeout_margin = 15.0;
  double maximum_joint_travel = 3.5;
  double endpoint_tolerance = 0.002;
  double maximum_axis_error_degrees = 3.0;
  double state_timeout = 5.0;
  double feedback_timeout = 0.5;

  node->get_parameter_or("execute", execute, execute);
  node->get_parameter_or(
    "parameters_confirmed", parameters_confirmed, parameters_confirmed);
  node->get_parameter_or("planning_attempts", planning_attempts, planning_attempts);
  node->get_parameter_or(
    "maximum_ik_attempts", maximum_ik_attempts, maximum_ik_attempts);
  node->get_parameter_or(
    "maximum_unique_ik_candidates", maximum_unique_ik_candidates,
    maximum_unique_ik_candidates);
  node->get_parameter_or("contact_x", contact_x, contact_x);
  node->get_parameter_or("contact_y", contact_y, contact_y);
  node->get_parameter_or("contact_z", contact_z, contact_z);
  node->get_parameter_or(
    "precontact_clearance", precontact_clearance, precontact_clearance);
  node->get_parameter_or(
    "work_ready_clearance", work_ready_clearance, work_ready_clearance);
  node->get_parameter_or("velocity_scale", velocity_scale, velocity_scale);
  node->get_parameter_or(
    "acceleration_scale", acceleration_scale, acceleration_scale);
  node->get_parameter_or("planning_timeout", planning_timeout, planning_timeout);
  node->get_parameter_or(
    "execution_timeout_margin", execution_timeout_margin,
    execution_timeout_margin);
  node->get_parameter_or(
    "maximum_joint_travel", maximum_joint_travel, maximum_joint_travel);
  node->get_parameter_or(
    "endpoint_tolerance", endpoint_tolerance, endpoint_tolerance);
  node->get_parameter_or(
    "maximum_axis_error_degrees", maximum_axis_error_degrees,
    maximum_axis_error_degrees);
  node->get_parameter_or("state_timeout", state_timeout, state_timeout);
  node->get_parameter_or("feedback_timeout", feedback_timeout, feedback_timeout);

  if (execute && !parameters_confirmed)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "R10 预接触真机执行需要 execute:=true 和 parameters_confirmed:=true");
    rclcpp::shutdown();
    return 2;
  }
  const bool valid = planning_attempts >= 1 && planning_attempts <= 10 &&
    maximum_ik_attempts >= 1 && maximum_ik_attempts <= 128 &&
    maximum_unique_ik_candidates >= 1 && maximum_unique_ik_candidates <= 32 &&
    std::isfinite(contact_x) && std::isfinite(contact_y) &&
    std::isfinite(contact_z) && std::isfinite(precontact_clearance) &&
    precontact_clearance >= 0.001 && precontact_clearance <= 0.020 &&
    std::isfinite(work_ready_clearance) && work_ready_clearance >= 0.020 &&
    work_ready_clearance <= 0.150 && std::isfinite(velocity_scale) &&
    velocity_scale > 0.0 && velocity_scale <= 0.20 &&
    std::isfinite(acceleration_scale) && acceleration_scale > 0.0 &&
    acceleration_scale <= 0.20 && std::isfinite(planning_timeout) &&
    planning_timeout > 0.0 && std::isfinite(execution_timeout_margin) &&
    execution_timeout_margin >= 10.0 &&
    std::isfinite(maximum_joint_travel) && maximum_joint_travel > 0.0 &&
    std::isfinite(endpoint_tolerance) && endpoint_tolerance > 0.0 &&
    endpoint_tolerance <= 0.005 &&
    std::isfinite(maximum_axis_error_degrees) &&
    maximum_axis_error_degrees > 0.0 && maximum_axis_error_degrees <= 5.0 &&
    std::isfinite(state_timeout) && state_timeout > 0.0 &&
    std::isfinite(feedback_timeout) && feedback_timeout > 0.0;
  if (!valid)
  {
    RCLCPP_ERROR(node->get_logger(), "R10 预接触规划参数无效或超过硬边界");
    rclcpp::shutdown();
    return 2;
  }

  std::mutex state_mutex;
  std::condition_variable state_condition;
  sensor_msgs::msg::JointState latest_joint_state;
  jaka_msgs::msg::RobotMsg latest_robot_state;
  std::uint64_t joint_sequence = 0U;
  std::uint64_t robot_sequence = 0U;
  SteadyClock::time_point joint_received_at{};
  SteadyClock::time_point robot_received_at{};
  auto joint_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::JointState::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      latest_joint_state = *message;
      ++joint_sequence;
      joint_received_at = SteadyClock::now();
      state_condition.notify_all();
    });
  auto robot_subscription = node->create_subscription<jaka_msgs::msg::RobotMsg>(
    "/jaka_driver/robot_states", rclcpp::SensorDataQoS(),
    [&](const jaka_msgs::msg::RobotMsg::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      latest_robot_state = *message;
      ++robot_sequence;
      robot_received_at = SteadyClock::now();
      state_condition.notify_all();
    });

  rclcpp::executors::MultiThreadedExecutor ros_executor;
  ros_executor.add_node(node);
  std::thread spin_thread([&]() {ros_executor.spin();});
  int exit_code = 1;
  try
  {
    sensor_msgs::msg::JointState planning_start;
    {
      std::unique_lock<std::mutex> lock(state_mutex);
      if (!state_condition.wait_for(
          lock, std::chrono::duration<double>(state_timeout),
          [&]() {return joint_sequence > 0U && (!execute || robot_sequence > 0U);} ))
      {
        throw std::runtime_error(
                execute ? "等待关节状态或机器人状态超时" :
                "等待关节状态超时");
      }
      if (execute && !robot_ready(latest_robot_state))
      {
        throw std::runtime_error(
                "机器人不满足静止、上电、使能、无碰撞执行条件");
      }
      planning_start = latest_joint_state;
    }

    massage_motion::ToolOrientationRequest orientation_request;
    orientation_request.surface_normal_world = {0.0, 0.0, 1.0};
    orientation_request.tangent_direction_world = {0.0, 1.0, 0.0};
    const auto orientation =
      massage_motion::make_surface_aligned_tool_orientation(orientation_request);
    if (!orientation.valid)
    {
      throw std::runtime_error("无法构造 R10 工具姿态: " + orientation.message);
    }

    geometry_msgs::msg::PoseStamped precontact;
    precontact.header.frame_id = "world";
    precontact.pose.position.x = contact_x;
    precontact.pose.position.y = contact_y;
    precontact.pose.position.z = contact_z + precontact_clearance;
    precontact.pose.orientation = orientation.orientation;
    auto work_ready = precontact;
    work_ready.pose.position.z += work_ready_clearance;
    RCLCPP_INFO(
      node->get_logger(),
      "R10 PRECONTACT TARGETS: execute=%s, contact=[%.6f %.6f %.6f] m, "
      "precontact=[%.6f %.6f %.6f] m, work_ready_z=%.6f m, "
      "tool_z_world=[%.3f %.3f %.3f]",
      execute ? "true" : "false", contact_x, contact_y, contact_z,
      precontact.pose.position.x, precontact.pose.position.y,
      precontact.pose.position.z, work_ready.pose.position.z,
      orientation.tool_z_world[0], orientation.tool_z_world[1],
      orientation.tool_z_world[2]);

    massage_motion::PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "massage_tool_tip";
    planner_config.reference_frame = "world";
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";
    auto sdk = std::make_shared<massage_motion::MotionPlanningSdk>(
      node, planner_config);
    massage_motion::PlanCompetitionConfig competition_config;
    competition_config.maximum_joint_travel = maximum_joint_travel;
    auto competition = std::make_shared<massage_motion::CompetitiveMotionPlanner>(
      std::vector<massage_motion::PlanningSource>{
        {"motion_sdk", sdk, static_cast<std::size_t>(planning_attempts)}},
      competition_config);
    robot_model_loader::RobotModelLoader loader(node, "robot_description", true);
    const auto robot_model = loader.getModel();
    if (!robot_model)
    {
      throw std::runtime_error("无法加载 R10 预接触 RobotModel");
    }
    const auto * group = robot_model->getJointModelGroup("jaka_s5");
    if (!group || !group->getSolverInstance())
    {
      throw std::runtime_error(
              "R10 预接触 RobotModel 未加载 jaka_s5 IK 求解器");
    }
    massage_motion::PoseIkCandidateGeneratorConfig ik_config;
    ik_config.maximum_attempts =
      static_cast<std::size_t>(maximum_ik_attempts);
    ik_config.maximum_unique_candidates =
      static_cast<std::size_t>(maximum_unique_ik_candidates);
    massage_motion::PoseIkCandidateGenerator ik_generator(
      robot_model, "jaka_s5", "massage_tool_tip", ik_config);
    const auto ik_report = ik_generator.generate(planning_start, work_ready);
    RCLCPP_INFO(
      node->get_logger(),
      "R10 PRECONTACT IK: success=%s, attempts=%zu, unique=%zu, "
      "translation=%.6f m, rotation=%.6f rad, tip=%s, solver_tip=%s",
      ik_report.success ? "true" : "false", ik_report.attempts.size(),
      ik_report.candidates.size(), ik_report.target_distance.translation,
      ik_report.target_distance.rotation, ik_report.requested_tip_link.c_str(),
      ik_report.solver_tip_link.c_str());
    if (!ik_report.success)
    {
      throw std::runtime_error("R10 工作准备 IK 失败: " + ik_report.message);
    }
    const auto ordered_start = massage_motion::order_joint_state_positions(
      ik_report.variable_names, planning_start);
    if (!ordered_start.valid)
    {
      throw std::runtime_error(
              "R10 无法固定规划起点: " + ordered_start.message);
    }

    massage_motion::MotionRequest ptp_request;
    ptp_request.request_id = "r10_work_ready";
    ptp_request.motion_type = massage_motion::MotionType::kPtp;
    ptp_request.velocity_scale = velocity_scale;
    ptp_request.acceleration_scale = acceleration_scale;
    ptp_request.planning_timeout = planning_timeout;
    moveit_msgs::msg::RobotState explicit_start;
    explicit_start.joint_state.name = ik_report.variable_names;
    explicit_start.joint_state.position = ordered_start.positions;
    explicit_start.is_diff = true;
    ptp_request.start_state = explicit_start;

    massage_motion::MotionRequest lin_request;
    lin_request.request_id = "r10_precontact";
    lin_request.motion_type = massage_motion::MotionType::kLin;
    lin_request.target = massage_motion::PoseTarget{precontact};
    lin_request.velocity_scale = velocity_scale;
    lin_request.acceleration_scale = acceleration_scale;
    lin_request.planning_timeout = planning_timeout;
    const double maximum_axis_error = maximum_axis_error_degrees * M_PI / 180.0;
    massage_motion::ToolAlignmentGateConfig endpoint_gate;
    endpoint_gate.desired_tool_z_world = {0.0, 0.0, -1.0};
    endpoint_gate.expected_orientation = orientation.orientation;
    endpoint_gate.maximum_axis_error = maximum_axis_error;
    endpoint_gate.inspect_all_samples = false;
    auto lin_gate = endpoint_gate;
    lin_gate.inspect_all_samples = true;

    massage_motion::PlanResult ptp_plan;
    massage_motion::PlanResult lin_plan;
    double best_pair_score = std::numeric_limits<double>::infinity();
    double selected_minimum_tool_z = 0.0;
    double selected_required_tool_z = 0.0;
    std::size_t selected_ik = std::numeric_limits<std::size_t>::max();
    std::size_t complete_pairs = 0U;
    for (std::size_t index = 0; index < ik_report.candidates.size(); ++index)
    {
      auto candidate_planner = std::make_shared<
        massage_motion::CompetitiveMotionPlanner>(
        std::vector<massage_motion::PlanningSource>{
          {"motion_sdk", sdk, static_cast<std::size_t>(planning_attempts)}},
        competition_config);
      ptp_request.request_id = "r10_work_ready_ik_" + std::to_string(index + 1U);
      ptp_request.target = massage_motion::JointTarget{
        ik_report.candidates[index].positions};
      const auto candidate_ptp = candidate_planner->plan(ptp_request);
      if (!candidate_ptp.success ||
        candidate_ptp.trajectory.joint_trajectory.points.empty())
      {
        RCLCPP_INFO(
          node->get_logger(),
          "R10 PRECONTACT PAIR[%zu]: REJECTED: PTP=%s",
          index, candidate_ptp.message.c_str());
        continue;
      }
      const auto ptp_alignment =
        massage_motion::evaluate_tool_alignment_trajectory(
        robot_model, candidate_ptp.trajectory, "massage_tool_tip",
        endpoint_gate);
      const auto ptp_height = massage_motion::calculate_link_height_metrics(
        robot_model, candidate_ptp.trajectory, "massage_tool_tip");
      const double required_tool_z = ptp_height.valid ?
        std::min(ptp_height.start_z, precontact.pose.position.z) - 1.0e-4 :
        std::numeric_limits<double>::infinity();
      if (!ptp_alignment.valid || !ptp_alignment.accepted ||
        !ptp_height.valid || ptp_height.minimum_z < required_tool_z)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "R10 PRECONTACT PAIR[%zu]: REJECTED: PTP gate, alignment=%s, "
          "minimum_tool_z=%.6f m, required_z>=%.6f m",
          index, ptp_alignment.message.c_str(), ptp_height.minimum_z,
          required_tool_z);
        continue;
      }
      lin_request.start_state = terminal_state(candidate_ptp.trajectory);
      const auto candidate_lin = competition->plan(lin_request);
      if (!candidate_lin.success ||
        candidate_lin.trajectory.joint_trajectory.points.empty())
      {
        RCLCPP_INFO(
          node->get_logger(),
          "R10 PRECONTACT PAIR[%zu]: REJECTED: downstream LIN=%s",
          index, candidate_lin.message.c_str());
        continue;
      }
      const auto lin_alignment =
        massage_motion::evaluate_tool_alignment_trajectory(
        robot_model, candidate_lin.trajectory, "massage_tool_tip", lin_gate);
      if (!lin_alignment.valid || !lin_alignment.accepted)
      {
        RCLCPP_INFO(
          node->get_logger(),
          "R10 PRECONTACT PAIR[%zu]: REJECTED: LIN alignment=%s",
          index, lin_alignment.message.c_str());
        continue;
      }
      const auto ptp_metrics = massage_motion::calculate_trajectory_metrics(
        candidate_ptp.trajectory);
      const auto lin_metrics = massage_motion::calculate_trajectory_metrics(
        candidate_lin.trajectory);
      if (!ptp_metrics.valid || !lin_metrics.valid)
      {
        continue;
      }
      const double pair_score = ptp_metrics.joint_path_length +
        lin_metrics.joint_path_length +
        0.05 * (ptp_metrics.duration + lin_metrics.duration);
      ++complete_pairs;
      RCLCPP_INFO(
        node->get_logger(),
        "R10 PRECONTACT PAIR[%zu]: ACCEPTED: score=%.6f, "
        "ptp_duration=%.3f s, lin_duration=%.3f s, minimum_tool_z=%.6f m",
        index, pair_score, ptp_metrics.duration, lin_metrics.duration,
        ptp_height.minimum_z);
      if (pair_score < best_pair_score)
      {
        best_pair_score = pair_score;
        selected_ik = index;
        selected_minimum_tool_z = ptp_height.minimum_z;
        selected_required_tool_z = required_tool_z;
        ptp_plan = candidate_ptp;
        lin_plan = candidate_lin;
      }
    }
    if (selected_ik == std::numeric_limits<std::size_t>::max())
    {
      throw std::runtime_error(
              "R10 工作准备 PTP 没有可继续完成垂直 LIN 的完整候选方案");
    }
    RCLCPP_INFO(
      node->get_logger(),
      "R10 PRECONTACT PAIR COMPETITION: PASS: selected_ik=%zu, "
      "complete_pairs=%zu/%zu, score=%.6f",
      selected_ik, complete_pairs, ik_report.candidates.size(), best_pair_score);
    require_alignment(
      robot_model, ptp_plan.trajectory, orientation.orientation, false,
      maximum_axis_error, "work_ready_endpoint", node->get_logger());
    RCLCPP_INFO(
      node->get_logger(),
      "R10 PRECONTACT FREE-SPACE HEIGHT: PASS: tool=massage_tool_tip, "
      "minimum_z=%.6f m, required_z>=%.6f m, work_ready_z=%.6f m",
      selected_minimum_tool_z, selected_required_tool_z,
      work_ready.pose.position.z);
    require_alignment(
      robot_model, lin_plan.trajectory, orientation.orientation, true,
      maximum_axis_error, "precontact_lin", node->get_logger());

    if (!execute)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "R10 PRECONTACT PLAN-ONLY: PASS: PTP 和 LIN 均已规划并通过姿态门禁；"
        "未创建执行器，未发送轨迹 Goal");
      exit_code = 0;
    }
    else
    {
      auto backend = std::make_shared<massage_motion::MoveItTrajectoryExecutor>(node);
      auto executor = std::make_shared<massage_motion::GuardedTrajectoryExecutor>(
        backend,
        [&]() -> massage_motion::ExecutionValidationResult
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          const auto now = SteadyClock::now();
          if (joint_sequence == 0U || robot_sequence == 0U ||
            std::chrono::duration<double>(now - joint_received_at).count() >
            feedback_timeout ||
            std::chrono::duration<double>(now - robot_received_at).count() >
            feedback_timeout || !robot_ready(latest_robot_state))
          {
            return {
              false, massage_motion::ExecutionError::kRejected,
              "关节或机器人状态失效"};
          }
          return {true, massage_motion::ExecutionError::kNone, "ready"};
        });

      const auto execute_plan = [&](const massage_motion::PlanResult & plan,
          const std::string & phase)
        {
          massage_motion::ExecutionTimingPolicy policy;
          policy.margin = execution_timeout_margin;
          const auto timing = massage_motion::calculate_execution_timing(
            plan.trajectory, policy);
          if (!timing.valid)
          {
            throw std::runtime_error(phase + " 执行时限无效: " + timing.message);
          }
          massage_motion::ExecutionRequest request;
          request.request_id = "r10_" + phase;
          request.robot_trajectory = plan.trajectory;
          request.timeout = timing.timeout;
          const auto result = executor->execute(request);
          if (!result.success)
          {
            throw std::runtime_error(phase + " 执行失败: " + result.message);
          }
          sensor_msgs::msg::JointState actual;
          {
            std::unique_lock<std::mutex> lock(state_mutex);
            const auto sequence = joint_sequence;
            if (!state_condition.wait_for(
                lock, std::chrono::duration<double>(state_timeout),
                [&]() {return joint_sequence > sequence;}))
            {
              throw std::runtime_error(phase + " 执行后等待关节状态超时");
            }
            actual = latest_joint_state;
          }
          const auto endpoint =
            massage_motion::calculate_trajectory_endpoint_error(
            plan.trajectory, actual);
          if (!massage_motion::joint_target_reached(endpoint, endpoint_tolerance))
          {
            std::ostringstream message;
            message << phase << " 终点误差超限: max_error="
                    << endpoint.max_absolute_error << " rad, tolerance="
                    << endpoint_tolerance << " rad, detail=" << endpoint.message;
            throw std::runtime_error(message.str());
          }
          RCLCPP_INFO(
            node->get_logger(),
            "R10 PRECONTACT EXECUTION [%s]: PASS: %s, max_joint_error=%.9f rad",
            phase.c_str(), timing.message.c_str(), endpoint.max_absolute_error);
        };

      execute_plan(ptp_plan, "work_ready_ptp");

      // Replan the short vertical approach from the measured post-PTP state.
      lin_request.start_state.reset();
      lin_plan = competition->plan(lin_request);
      if (!lin_plan.success || lin_plan.trajectory.joint_trajectory.points.empty())
      {
        throw std::runtime_error(
                "PTP 执行后 R10 预接触 LIN 重规划失败: " + lin_plan.message);
      }
      require_alignment(
        robot_model, lin_plan.trajectory, orientation.orientation, true,
        maximum_axis_error, "precontact_lin_replanned", node->get_logger());
      execute_plan(lin_plan, "precontact_lin");
      RCLCPP_INFO(
        node->get_logger(),
        "R10 PRECONTACT EXECUTION: PASS: 已到达固定软块上方预接触位；"
        "本节点未启用导纳，确认软块间隙后再单独运行 R10-C");
      exit_code = 0;
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      node->get_logger(), "R10 PRECONTACT: FAIL: %s", exception.what());
    exit_code = 4;
  }

  (void)joint_subscription;
  (void)robot_subscription;
  ros_executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
