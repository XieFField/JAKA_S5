#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_bringup/default_targets.hpp"
#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/joint_target_interpolation.hpp"
#include "massage_motion/motion_planning_sdk.hpp"

namespace
{

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
  const std::set<std::string> unique_names(
    state.name.begin(), state.name.end());
  if (unique_names.size() != state.name.size())
  {
    error = "JointState 包含重复关节名称";
    return false;
  }

  positions.clear();
  positions.reserve(expected_joint_names.size());
  for (const auto & joint_name : expected_joint_names)
  {
    const auto iterator = std::find(
      state.name.begin(), state.name.end(), joint_name);
    if (iterator == state.name.end())
    {
      error = "JointState 缺少 " + joint_name;
      return false;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(state.name.begin(), iterator));
    if (!std::isfinite(state.position[index]))
    {
      error = joint_name + " 的位置不是有限数值";
      return false;
    }
    positions.push_back(state.position[index]);
  }
  return true;
}

void log_competition(
  const rclcpp::Logger & logger,
  const massage_motion::PlanCompetitionReport & report)
{
  for (std::size_t index = 0; index < report.candidates.size(); ++index)
  {
    const auto & candidate = report.candidates[index];
    RCLCPP_INFO(
      logger,
      "分段规划候选[%zu]: source=%s attempt=%zu accepted=%s score=%.6f "
      "path=%.6f rad duration=%.3f s max_joint=%.6f rad message=%s",
      index, candidate.source_name.c_str(), candidate.attempt,
      candidate.accepted ? "true" : "false", candidate.score,
      candidate.metrics.joint_path_length, candidate.metrics.duration,
      candidate.metrics.maximum_joint_travel, candidate.message.c_str());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "home_segment_planning_real",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  double segment_ratio = 0.25;
  int planning_attempts = 3;
  double velocity_scale = 0.02;
  double acceleration_scale = 0.02;
  double planning_timeout = 5.0;
  double joint_state_timeout = 3.0;
  double maximum_joint_travel = 0.15;
  node->get_parameter_or("segment_ratio", segment_ratio, 0.25);
  node->get_parameter_or("planning_attempts", planning_attempts, 3);
  node->get_parameter_or("velocity_scale", velocity_scale, 0.02);
  node->get_parameter_or("acceleration_scale", acceleration_scale, 0.02);
  node->get_parameter_or("planning_timeout", planning_timeout, 5.0);
  node->get_parameter_or("joint_state_timeout", joint_state_timeout, 3.0);
  node->get_parameter_or("maximum_joint_travel", maximum_joint_travel, 0.15);

  if (!std::isfinite(segment_ratio) || segment_ratio <= 0.0 ||
    segment_ratio > 1.0 || planning_attempts < 1 || planning_attempts > 10 ||
    !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
    velocity_scale > 1.0 || !std::isfinite(acceleration_scale) ||
    acceleration_scale <= 0.0 || acceleration_scale > 1.0 ||
    !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
    !std::isfinite(joint_state_timeout) || joint_state_timeout <= 0.0 ||
    !std::isfinite(maximum_joint_travel) || maximum_joint_travel <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "中距离只规划参数无效");
    rclcpp::shutdown();
    return 2;
  }

  const std::vector<std::string> joint_names(
    massage_bringup::kJakaS5JointNames.begin(),
    massage_bringup::kJakaS5JointNames.end());
  const std::vector<double> home_positions(
    massage_bringup::kMassageHomeJointPositions.begin(),
    massage_bringup::kMassageHomeJointPositions.end());

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
  std::thread spin_thread([&ros_executor]() {ros_executor.spin();});

  int exit_code = 1;
  try
  {
    sensor_msgs::msg::JointState start_state;
    {
      std::unique_lock<std::mutex> lock(state_mutex);
      if (!state_condition.wait_for(
          lock, std::chrono::duration<double>(joint_state_timeout),
          [&]() {return state_sequence > 0U;}))
      {
        throw std::runtime_error("等待 /joint_states 超时");
      }
      start_state = latest_state;
    }

    std::vector<double> start_positions;
    std::string state_error;
    if (!extract_ordered_positions(
        start_state, joint_names, start_positions, state_error))
    {
      throw std::runtime_error("起始关节状态无效: " + state_error);
    }
    const auto segment = massage_motion::interpolate_joint_target(
      start_positions, home_positions, segment_ratio, maximum_joint_travel);
    if (!segment.valid)
    {
      throw std::runtime_error("分段目标无效: " + segment.message);
    }

    RCLCPP_INFO(
      node->get_logger(),
      "中距离固定目标已生成: ratio=%.3f, maximum_joint_travel=%.9f rad, "
      "planning_attempts=%d；本节点没有轨迹执行器",
      segment_ratio, segment.maximum_joint_travel, planning_attempts);
    for (std::size_t joint = 0; joint < joint_names.size(); ++joint)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "%s: start=%.9f rad, home=%.9f rad, target=%.9f rad, travel=%.9f rad",
        joint_names[joint].c_str(), start_positions[joint], home_positions[joint],
        segment.target_positions[joint], segment.joint_travels[joint]);
    }

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

    massage_motion::MotionRequest request;
    request.request_id = "home_segment_plan_only";
    request.motion_type = massage_motion::MotionType::kPtp;
    request.target = massage_motion::JointTarget{segment.target_positions};
    request.velocity_scale = velocity_scale;
    request.acceleration_scale = acceleration_scale;
    request.planning_timeout = planning_timeout;

    const auto plan = planner->plan(request);
    const auto report = planner->last_report();
    log_competition(node->get_logger(), report);
    if (!plan.success || !report.success ||
      report.selected_candidate >= report.candidates.size())
    {
      throw std::runtime_error("中距离竞争规划失败: " + plan.message);
    }
    const auto & selected = report.candidates[report.selected_candidate];
    RCLCPP_INFO(
      node->get_logger(),
      "HOME SEGMENT PLAN-ONLY: PASS: selected=%zu, score=%.6f, "
      "points=%zu, duration=%.3f s, path=%.6f rad, max_joint=%.6f rad；"
      "未发送运动命令",
      report.selected_candidate, selected.score,
      plan.trajectory.joint_trajectory.points.size(), selected.metrics.duration,
      selected.metrics.joint_path_length,
      selected.metrics.maximum_joint_travel);
    exit_code = 0;
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      node->get_logger(), "中距离只规划验证失败: %s", exception.what());
    exit_code = 3;
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
