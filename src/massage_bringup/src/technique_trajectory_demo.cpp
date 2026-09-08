#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "moveit/robot_model_loader/robot_model_loader.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_bringup/default_targets.hpp"
#include "massage_motion/cartesian_path_verification.hpp"
#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/elbow_posture_geometry.hpp"
#include "massage_motion/execution_timing.hpp"
#include "massage_motion/link_trajectory_geometry.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/pose_ik_candidate_generator.hpp"
#include "massage_motion/technique_path_generator.hpp"
#include "massage_motion/tool_orientation.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"

namespace
{

geometry_msgs::msg::PoseStamped default_work_pose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = -0.471238630147741;
  pose.pose.position.y = 0.156275068796867;
  pose.pose.position.z = 0.281381721182422;
  massage_motion::ToolOrientationRequest orientation_request;
  const auto orientation =
    massage_motion::make_surface_aligned_tool_orientation(orientation_request);
  if (!orientation.valid)
  {
    throw std::runtime_error(
            "无法构造默认推拿工具姿态: " + orientation.message);
  }
  pose.pose.orientation = orientation.orientation;
  return pose;
}

moveit_msgs::msg::RobotState robot_state(
  const std::vector<std::string> & names,
  const std::vector<double> & positions)
{
  moveit_msgs::msg::RobotState state;
  state.joint_state.name = names;
  state.joint_state.position = positions;
  state.is_diff = true;
  return state;
}

moveit_msgs::msg::RobotState terminal_state(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  if (trajectory.joint_trajectory.joint_names.empty() ||
    trajectory.joint_trajectory.points.empty())
  {
    throw std::runtime_error("无法从空轨迹构造下一段起点");
  }
  return robot_state(
    trajectory.joint_trajectory.joint_names,
    trajectory.joint_trajectory.points.back().positions);
}

geometry_msgs::msg::PoseStamped stamped_pose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = frame;
  stamped.pose = pose;
  return stamped;
}

struct PlannedSegment
{
  std::string name;
  massage_motion::PlanResult plan;
  geometry_msgs::msg::Pose expected_pose;
};

double pose_translation_error(
  const geometry_msgs::msg::Pose & expected,
  const geometry_msgs::msg::Pose & actual)
{
  return std::sqrt(
    std::pow(expected.position.x - actual.position.x, 2) +
    std::pow(expected.position.y - actual.position.y, 2) +
    std::pow(expected.position.z - actual.position.z, 2));
}

double pose_rotation_error(
  const geometry_msgs::msg::Pose & expected,
  const geometry_msgs::msg::Pose & actual)
{
  const double dot = std::abs(
    expected.orientation.x * actual.orientation.x +
    expected.orientation.y * actual.orientation.y +
    expected.orientation.z * actual.orientation.z +
    expected.orientation.w * actual.orientation.w);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "technique_trajectory_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  std::string technique = "press";
  std::string execution_environment = "simulation";
  bool parameters_confirmed = false;
  bool execute = false;
  double velocity_scale = 0.10;
  double acceleration_scale = 0.10;
  double planning_timeout = 8.0;
  double execution_timeout_margin = 5.0;
  double endpoint_joint_tolerance = 0.006;
  double endpoint_position_tolerance = 0.006;
  double endpoint_orientation_tolerance = 0.04;
  double joint_state_timeout = 5.0;
  double press_stroke = 0.004;
  double press_cycle_duration = 2.0;
  int press_cycles = 2;
  double knead_radius = 0.010;
  double knead_cycle_duration = 4.0;
  int knead_cycles = 2;
  node->get_parameter_or("technique", technique, std::string{"press"});
  node->get_parameter_or(
    "execution_environment", execution_environment,
    std::string{"simulation"});
  node->get_parameter_or("parameters_confirmed", parameters_confirmed, false);
  node->get_parameter_or("execute", execute, false);
  node->get_parameter_or("velocity_scale", velocity_scale, 0.10);
  node->get_parameter_or("acceleration_scale", acceleration_scale, 0.10);
  node->get_parameter_or("planning_timeout", planning_timeout, 8.0);
  node->get_parameter_or(
    "execution_timeout_margin", execution_timeout_margin, 5.0);
  node->get_parameter_or(
    "endpoint_joint_tolerance", endpoint_joint_tolerance, 0.006);
  node->get_parameter_or(
    "endpoint_position_tolerance", endpoint_position_tolerance, 0.006);
  node->get_parameter_or(
    "endpoint_orientation_tolerance", endpoint_orientation_tolerance, 0.04);
  node->get_parameter_or("joint_state_timeout", joint_state_timeout, 5.0);
  node->get_parameter_or("press_stroke", press_stroke, 0.004);
  node->get_parameter_or(
    "press_cycle_duration", press_cycle_duration, 2.0);
  node->get_parameter_or("press_cycles", press_cycles, 2);
  node->get_parameter_or("knead_radius", knead_radius, 0.010);
  node->get_parameter_or(
    "knead_cycle_duration", knead_cycle_duration, 4.0);
  node->get_parameter_or("knead_cycles", knead_cycles, 2);
  const bool real = execution_environment == "real";
  if ((technique != "press" && technique != "knead") ||
    (execution_environment != "simulation" && !real) ||
    (real && execute && !parameters_confirmed) ||
    !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
    velocity_scale > 1.0 || !std::isfinite(acceleration_scale) ||
    acceleration_scale <= 0.0 || acceleration_scale > 1.0 ||
    !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
    !std::isfinite(execution_timeout_margin) || execution_timeout_margin < 0.0 ||
    !std::isfinite(endpoint_joint_tolerance) || endpoint_joint_tolerance <= 0.0 ||
    !std::isfinite(endpoint_position_tolerance) ||
    endpoint_position_tolerance <= 0.0 ||
    !std::isfinite(endpoint_orientation_tolerance) ||
    endpoint_orientation_tolerance <= 0.0 ||
    press_cycles <= 0 || knead_cycles <= 0)
  {
    RCLCPP_ERROR(node->get_logger(), "手法轨迹参数或真机执行权限无效");
    rclcpp::shutdown();
    return 2;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "手法轨迹权限门禁: technique=%s, environment=%s, execute=%s, "
    "parameters_confirmed=%s, contact=false, compliance=false",
    technique.c_str(), execution_environment.c_str(), execute ? "true" : "false",
    parameters_confirmed ? "true" : "false");

  std::mutex state_mutex;
  std::condition_variable state_condition;
  sensor_msgs::msg::JointState latest_state;
  std::uint64_t state_sequence = 0U;
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::JointState::SharedPtr message)
    {
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        latest_state = *message;
        ++state_sequence;
      }
      state_condition.notify_all();
    });
  rclcpp::executors::SingleThreadedExecutor ros_executor;
  ros_executor.add_node(node);
  std::thread spin_thread([&]() {ros_executor.spin();});
  int exit_code = 1;
  try
  {
    sensor_msgs::msg::JointState start_joint_state;
    {
      std::unique_lock<std::mutex> lock(state_mutex);
      if (!state_condition.wait_for(
          lock, std::chrono::duration<double>(joint_state_timeout),
          [&]() {return state_sequence > 0U;}))
      {
        throw std::runtime_error("等待 /joint_states 超时");
      }
      start_joint_state = latest_state;
    }

    auto work_pose = default_work_pose();
    work_pose.header.stamp = node->now();
    massage_motion::TechniquePath technique_path;
    if (technique == "press")
    {
      massage_motion::PressPathRequest request;
      request.contact_pose = work_pose;
      request.stroke = press_stroke;
      request.cycle_duration = press_cycle_duration;
      request.cycles = static_cast<std::size_t>(press_cycles);
      request.sample_period = 0.02;
      request.maximum_speed = 0.02;
      const auto generated =
        massage_motion::TechniquePathGenerator::generate_press(request);
      if (!generated.success)
      {
        throw std::runtime_error("按法几何路径失败: " + generated.message);
      }
      technique_path = generated.path;
    }
    else
    {
      massage_motion::KneadPathRequest request;
      request.center_pose = work_pose;
      request.radius = knead_radius;
      request.cycle_duration = knead_cycle_duration;
      request.cycles = static_cast<std::size_t>(knead_cycles);
      request.sample_period = 0.02;
      request.maximum_speed = 0.02;
      const auto generated =
        massage_motion::TechniquePathGenerator::generate_knead(request);
      if (!generated.success)
      {
        throw std::runtime_error("揉法几何路径失败: " + generated.message);
      }
      technique_path = generated.path;
    }
    const auto first_pose = technique_path.points.front().pose;
    RCLCPP_INFO(
      node->get_logger(),
      "%s 几何路径: points=%zu, duration=%.3f s, nominal_max_speed=%.6f m/s, "
      "start=[%.6f %.6f %.6f] m",
      technique.c_str(), technique_path.points.size(), technique_path.duration,
      technique_path.nominal_speed, first_pose.position.x,
      first_pose.position.y, first_pose.position.z);

    massage_motion::PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "massage_tool_tip";
    planner_config.reference_frame = "world";
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";
    auto sdk = std::make_shared<massage_motion::MotionPlanningSdk>(
      node, planner_config);
    robot_model_loader::RobotModelLoader loader(node, "robot_description", true);
    const auto robot_model = loader.getModel();
    if (!robot_model)
    {
      throw std::runtime_error("无法加载 RobotModel");
    }

    massage_motion::PoseIkCandidateGeneratorConfig ik_config;
    ik_config.maximum_attempts = 24U;
    ik_config.maximum_unique_candidates = 8U;
    ik_config.random_seed = 684U;
    massage_motion::PoseIkCandidateGenerator ik_generator(
      robot_model, planner_config.planning_group,
      planner_config.end_effector_link, ik_config);
    const auto ik_report = ik_generator.generate(
      start_joint_state, stamped_pose(first_pose, "world"));
    if (!ik_report.success)
    {
      throw std::runtime_error("手法首点 IK 失败: " + ik_report.message);
    }
    const auto ordered_start = massage_motion::order_joint_state_positions(
      ik_report.variable_names, start_joint_state);
    if (!ordered_start.valid)
    {
      throw std::runtime_error("无法固定规划起点: " + ordered_start.message);
    }
    const auto explicit_start = robot_state(
      ik_report.variable_names, ordered_start.positions);

    std::vector<std::string> standby_names;
    for (const auto * name : massage_bringup::kJakaS5JointNames)
    {
      standby_names.emplace_back(name);
    }
    const std::vector<double> standby_positions(
      massage_bringup::kMassageHomeJointPositions.begin(),
      massage_bringup::kMassageHomeJointPositions.end());
    const auto standby_height =
      massage_motion::calculate_link_height_at_joint_target(
      robot_model, standby_names, standby_positions, "Link_03");
    if (!standby_height.valid)
    {
      throw std::runtime_error("无法计算业务待机 Link_03 高度");
    }
    massage_motion::LinkHeightGateConfig height_config;
    height_config.minimum_z = 0.0;
    height_config.reference_z = standby_height.z;
    height_config.maximum_drop_below_reference = 0.20;
    massage_motion::ElbowPostureGeometryConfig elbow_config;

    const auto validate_plan = [&](const std::string & name,
        const massage_motion::PlanResult & plan)
      {
        if (!plan.success)
        {
          throw std::runtime_error(name + " 规划失败: " + plan.message);
        }
        const auto height = massage_motion::calculate_link_height_metrics(
          robot_model, plan.trajectory, "Link_03");
        const auto gate = massage_motion::evaluate_link_height_gate(
          height, height_config);
        if (!gate.accepted)
        {
          throw std::runtime_error(name + " 高度门禁失败: " + gate.message);
        }
        const auto elbow = massage_motion::calculate_elbow_posture_metrics(
          robot_model, plan.trajectory, elbow_config);
        if (!elbow.valid || elbow.side_change_count != 0U)
        {
          throw std::runtime_error(name + " 肘部构型无效或发生分支翻转");
        }
        const auto metrics = massage_motion::calculate_trajectory_metrics(
          plan.trajectory);
        RCLCPP_INFO(
          node->get_logger(),
          "%s 门禁: ACCEPTED: points=%zu, duration=%.3f s, "
          "max_joint=%.6f rad, Link_03_min_z=%.6f m, elbow_side_changes=%zu",
          name.c_str(), plan.trajectory.joint_trajectory.points.size(),
          metrics.duration, metrics.maximum_joint_travel,
          height.minimum_z, elbow.side_change_count);
      };

    massage_motion::PlanCompetitionConfig approach_competition_config;
    approach_competition_config.maximum_joint_travel = 3.5;
    massage_motion::PlanResult approach_plan;
    double approach_score = std::numeric_limits<double>::infinity();
    for (const auto & candidate : ik_report.candidates)
    {
      massage_motion::MotionRequest request;
      request.request_id = technique + "_approach";
      request.motion_type = massage_motion::MotionType::kPtp;
      request.target = massage_motion::JointTarget{candidate.positions};
      request.velocity_scale = velocity_scale;
      request.acceleration_scale = acceleration_scale;
      request.planning_timeout = planning_timeout;
      request.start_state = explicit_start;
      massage_motion::CompetitiveMotionPlanner planner(
        {{"motion_sdk", sdk, 2U}}, approach_competition_config);
      const auto candidate_plan = planner.plan(request);
      if (!candidate_plan.success)
      {
        continue;
      }
      try
      {
        validate_plan(technique + " PTP candidate", candidate_plan);
      }
      catch (const std::exception & exception)
      {
        RCLCPP_WARN(node->get_logger(), "%s", exception.what());
        continue;
      }
      const auto report = planner.last_report();
      const double score = report.candidates.at(report.selected_candidate).score;
      if (score < approach_score)
      {
        approach_score = score;
        approach_plan = candidate_plan;
      }
    }
    if (!approach_plan.success)
    {
      throw std::runtime_error("所有手法首点 PTP 候选均未通过门禁");
    }
    validate_plan(technique + " PTP selected", approach_plan);

    std::vector<PlannedSegment> segments;
    moveit_msgs::msg::RobotState next_start = terminal_state(
      approach_plan.trajectory);
    const auto plan_motion = [&](const std::string & name,
        massage_motion::MotionType motion_type,
        const massage_motion::MotionTarget & target,
        const geometry_msgs::msg::Pose & expected)
      {
        massage_motion::MotionRequest request;
        request.request_id = name;
        request.motion_type = motion_type;
        request.target = target;
        request.velocity_scale = velocity_scale;
        request.acceleration_scale = acceleration_scale;
        request.planning_timeout = planning_timeout;
        request.start_state = next_start;
        massage_motion::CompetitiveMotionPlanner planner(
          {{"motion_sdk", sdk, 2U}}, {});
        auto plan = planner.plan(request);
        validate_plan(name, plan);
        next_start = terminal_state(plan.trajectory);
        segments.push_back({name, std::move(plan), expected});
      };

    if (technique == "press")
    {
      for (int cycle = 0; cycle < press_cycles; ++cycle)
      {
        auto bottom = work_pose.pose;
        bottom.position.z -= press_stroke;
        plan_motion(
          "press_down_" + std::to_string(cycle + 1),
          massage_motion::MotionType::kLin,
          massage_motion::PoseTarget{stamped_pose(bottom, "world")}, bottom);
        plan_motion(
          "press_up_" + std::to_string(cycle + 1),
          massage_motion::MotionType::kLin,
          massage_motion::PoseTarget{stamped_pose(work_pose.pose, "world")},
          work_pose.pose);
      }
    }
    else
    {
      const double radius = knead_radius;
      for (int cycle = 0; cycle < knead_cycles; ++cycle)
      {
        auto left = work_pose.pose;
        left.position.x -= radius;
        auto top = work_pose.pose;
        top.position.y += radius;
        plan_motion(
          "knead_half_a_" + std::to_string(cycle + 1),
          massage_motion::MotionType::kCirc,
          massage_motion::CircularTarget{
            stamped_pose(top, "world"), stamped_pose(left, "world")}, left);
        auto right = work_pose.pose;
        right.position.x += radius;
        auto bottom = work_pose.pose;
        bottom.position.y -= radius;
        plan_motion(
          "knead_half_b_" + std::to_string(cycle + 1),
          massage_motion::MotionType::kCirc,
          massage_motion::CircularTarget{
            stamped_pose(bottom, "world"), stamped_pose(right, "world")}, right);
      }
    }
    RCLCPP_INFO(
      node->get_logger(), "%s 连续规划完成: approach=1, technique_segments=%zu",
      technique.c_str(), segments.size());

    if (!execute)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "TECHNIQUE TRAJECTORY PLAN-ONLY: PASS: technique=%s, segments=%zu, "
        "environment=%s；未发送运动命令",
        technique.c_str(), segments.size(), execution_environment.c_str());
      exit_code = 0;
    }
    else
    {
      massage_motion::MoveItTrajectoryExecutor executor(node);
      std::vector<PlannedSegment> all_segments;
      all_segments.push_back({technique + "_approach", approach_plan, first_pose});
      all_segments.insert(all_segments.end(), segments.begin(), segments.end());
      for (std::size_t index = 0; index < all_segments.size(); ++index)
      {
        auto & segment = all_segments[index];
        massage_motion::ExecutionTimingPolicy policy;
        policy.margin = execution_timeout_margin;
        const auto timing = massage_motion::calculate_execution_timing(
          segment.plan.trajectory, policy);
        if (!timing.valid)
        {
          throw std::runtime_error(segment.name + " 执行时限无效");
        }
        massage_motion::ExecutionRequest request;
        request.request_id = segment.name;
        request.robot_trajectory = segment.plan.trajectory;
        request.timeout = timing.timeout;
        const auto execution = executor.execute(request);
        if (!execution.success)
        {
          throw std::runtime_error(
            segment.name + " 执行失败: " + execution.message);
        }
        sensor_msgs::msg::JointState final_state;
        std::uint64_t before = 0U;
        {
          std::unique_lock<std::mutex> lock(state_mutex);
          before = state_sequence;
          state_condition.wait_for(
            lock, std::chrono::duration<double>(joint_state_timeout),
            [&]() {return state_sequence > before;});
          final_state = latest_state;
        }
        const auto joint_error =
          massage_motion::calculate_trajectory_endpoint_error(
          segment.plan.trajectory, final_state);
        const auto actual_pose = massage_motion::calculate_link_pose(
          robot_model, final_state, planner_config.end_effector_link);
        const double translation = actual_pose.valid ?
          pose_translation_error(segment.expected_pose, actual_pose.pose) :
          std::numeric_limits<double>::infinity();
        const double rotation = actual_pose.valid ?
          pose_rotation_error(segment.expected_pose, actual_pose.pose) :
          std::numeric_limits<double>::infinity();
        if (!massage_motion::joint_target_reached(
            joint_error, endpoint_joint_tolerance) ||
          translation > endpoint_position_tolerance ||
          rotation > endpoint_orientation_tolerance)
        {
          throw std::runtime_error(segment.name + " 自动终点验收失败");
        }
        RCLCPP_INFO(
          node->get_logger(),
          "%s EXECUTION GATE: PASS: index=%zu/%zu, joint_error=%.9f rad, "
          "tcp_translation=%.9f m, tcp_rotation=%.9f rad",
          segment.name.c_str(), index + 1U, all_segments.size(),
          joint_error.max_absolute_error, translation, rotation);
      }
      RCLCPP_INFO(
        node->get_logger(),
        "TECHNIQUE TRAJECTORY SIM EXECUTION: PASS: technique=%s, "
        "segments=%zu, contact=false, compliance=false",
        technique.c_str(), segments.size());
      exit_code = 0;
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      node->get_logger(), "%s 手法轨迹失败: %s",
      technique.c_str(), exception.what());
    exit_code = 6;
  }
  (void)subscription;
  ros_executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
