#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/admittance_controller_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "ros_gz_interfaces/msg/entity_wrench.hpp"

#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/execution_timing.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/ros2_control_compliance_controller.hpp"
#include "massage_motion/tool_orientation.hpp"

namespace
{

double seconds(const builtin_interfaces::msg::Duration & value)
{
  return static_cast<double>(value.sec) +
         static_cast<double>(value.nanosec) * 1.0e-9;
}

massage_motion::PoseTarget work_pose(
  double z, const geometry_msgs::msg::Quaternion & orientation)
{
  massage_motion::PoseTarget target;
  target.pose.header.frame_id = "world";
  target.pose.pose.position.x = -0.471238630147741;
  target.pose.pose.position.y = 0.156275068796867;
  target.pose.pose.position.z = z;
  target.pose.pose.orientation = orientation;
  return target;
}

std::vector<double> ordered_positions(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const trajectory_msgs::msg::JointTrajectoryPoint & point,
  const std::vector<std::string> & names)
{
  if (trajectory.joint_names.size() != point.positions.size())
  {
    return {};
  }
  std::vector<double> result;
  result.reserve(names.size());
  for (const auto & name : names)
  {
    const auto found = std::find(
      trajectory.joint_names.begin(), trajectory.joint_names.end(), name);
    if (found == trajectory.joint_names.end())
    {
      return {};
    }
    result.push_back(point.positions[static_cast<std::size_t>(
      std::distance(trajectory.joint_names.begin(), found))]);
  }
  return result;
}

massage_motion::ExecutionResult execute(
  massage_motion::MoveItTrajectoryExecutor & executor,
  const massage_motion::PlanResult & plan,
  const std::string & id,
  double margin)
{
  massage_motion::ExecutionTimingPolicy policy;
  policy.margin = margin;
  const auto timing = massage_motion::calculate_execution_timing(
    plan.trajectory, policy);
  if (!timing.valid)
  {
    throw std::runtime_error("无法计算轨迹执行时限: " + timing.message);
  }
  massage_motion::ExecutionRequest request;
  request.request_id = id;
  request.robot_trajectory = plan.trajectory;
  request.timeout = timing.timeout;
  return executor.execute(request);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "world_z_admittance_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  double precontact_z = 0.281381721182422;
  double search_depth = 0.020;
  double contact_threshold = 0.15;
  double force_limit = 5.0;
  double hold_duration = 1.0;
  bool skip_approach = false;
  bool enable_simulated_contact_wrench = false;
  double simulated_contact_trigger_fraction = 0.35;
  double simulated_contact_force_z = -1.0;
  std::string simulated_contact_entity = "jaka_s5::massage_head_link";
  node->get_parameter_or("precontact_z", precontact_z, precontact_z);
  node->get_parameter_or("search_depth", search_depth, search_depth);
  node->get_parameter_or("contact_threshold", contact_threshold, contact_threshold);
  node->get_parameter_or("force_limit", force_limit, force_limit);
  node->get_parameter_or("hold_duration", hold_duration, hold_duration);
  node->get_parameter_or("skip_approach", skip_approach, skip_approach);
  node->get_parameter_or(
    "enable_simulated_contact_wrench", enable_simulated_contact_wrench,
    enable_simulated_contact_wrench);
  node->get_parameter_or(
    "simulated_contact_trigger_fraction", simulated_contact_trigger_fraction,
    simulated_contact_trigger_fraction);
  node->get_parameter_or(
    "simulated_contact_force_z", simulated_contact_force_z,
    simulated_contact_force_z);
  node->get_parameter_or(
    "simulated_contact_entity", simulated_contact_entity,
    simulated_contact_entity);
  if (!std::isfinite(precontact_z) || !std::isfinite(search_depth) ||
    search_depth <= 0.0 || !std::isfinite(contact_threshold) ||
    contact_threshold <= 0.0 || !std::isfinite(force_limit) ||
    force_limit <= contact_threshold || !std::isfinite(hold_duration) ||
    hold_duration < 0.0 ||
    (enable_simulated_contact_wrench &&
    (!std::isfinite(simulated_contact_trigger_fraction) ||
    simulated_contact_trigger_fraction <= 0.0 ||
    simulated_contact_trigger_fraction >= 1.0 ||
    !std::isfinite(simulated_contact_force_z) ||
    std::abs(simulated_contact_force_z) < contact_threshold ||
    std::abs(simulated_contact_force_z) >= force_limit ||
    simulated_contact_entity.empty())))
  {
    RCLCPP_ERROR(node->get_logger(), "world-Z 导纳测试参数无效");
    rclcpp::shutdown();
    return 2;
  }

  std::mutex status_mutex;
  control_msgs::msg::AdmittanceControllerState latest_status;
  bool has_status = false;
  double peak_admittance_joint_offset = 0.0;
  double peak_controller_wrench_z = 0.0;
  double peak_admittance_velocity_z = 0.0;
  double peak_admittance_acceleration_z = 0.0;
  auto status_subscription =
    node->create_subscription<control_msgs::msg::AdmittanceControllerState>(
      "/massage_admittance_controller/status", rclcpp::SensorDataQoS(),
      [&](control_msgs::msg::AdmittanceControllerState::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(status_mutex);
        latest_status = *message;
        has_status = true;
        for (const double offset : message->joint_state.position)
        {
          peak_admittance_joint_offset = std::max(
            peak_admittance_joint_offset, std::abs(offset));
        }
        peak_controller_wrench_z = std::max(
          peak_controller_wrench_z,
          std::abs(message->wrench_base.wrench.force.z));
        peak_admittance_velocity_z = std::max(
          peak_admittance_velocity_z,
          std::abs(message->admittance_velocity.twist.linear.z));
        peak_admittance_acceleration_z = std::max(
          peak_admittance_acceleration_z,
          std::abs(message->admittance_acceleration.twist.linear.z));
      });

  auto simulated_wrench_publisher =
    node->create_publisher<ros_gz_interfaces::msg::EntityWrench>(
    "/world/massage_bed_contact_test/wrench/persistent", rclcpp::QoS(1).reliable());
  auto simulated_wrench_clear_publisher =
    node->create_publisher<ros_gz_interfaces::msg::Entity>(
    "/world/massage_bed_contact_test/wrench/clear", rclcpp::QoS(1).reliable());
  bool simulated_wrench_active = false;
  const auto clear_simulated_wrench = [&]()
    {
      if (!simulated_wrench_active)
      {
        return;
      }
      ros_gz_interfaces::msg::Entity entity;
      entity.name = simulated_contact_entity;
      entity.type = ros_gz_interfaces::msg::Entity::LINK;
      simulated_wrench_clear_publisher->publish(entity);
      simulated_wrench_active = false;
    };

  rclcpp::executors::MultiThreadedExecutor ros_executor;
  ros_executor.add_node(node);
  std::thread spin_thread([&]() {ros_executor.spin();});
  int exit_code = 1;
  try
  {
    massage_motion::PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "massage_tool_tip";
    planner_config.reference_frame = "world";
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";
    massage_motion::MotionPlanningSdk planner(node, planner_config);
    massage_motion::MoveItTrajectoryExecutor trajectory_executor(node);
    massage_motion::ToolOrientationRequest orientation_request;
    const auto tool_orientation =
      massage_motion::make_surface_aligned_tool_orientation(orientation_request);
    if (!tool_orientation.valid)
    {
      throw std::runtime_error(
              "无法构造 world-Z 导纳测试工具姿态: " +
              tool_orientation.message);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "WORLD-Z TOOL ORIENTATION: tool_z_world=[%.6f %.6f %.6f], "
      "tool_x_world=[%.6f %.6f %.6f]",
      tool_orientation.tool_z_world[0], tool_orientation.tool_z_world[1],
      tool_orientation.tool_z_world[2], tool_orientation.tool_x_world[0],
      tool_orientation.tool_x_world[1], tool_orientation.tool_x_world[2]);

    massage_motion::MotionRequest approach;
    approach.request_id = "world_z_precontact";
    approach.motion_type = massage_motion::MotionType::kPtp;
    approach.target = work_pose(precontact_z, tool_orientation.orientation);
    approach.velocity_scale = 0.10;
    approach.acceleration_scale = 0.05;
    approach.planning_timeout = 8.0;
    if (!skip_approach)
    {
      const auto approach_plan = planner.plan(approach);
      if (!approach_plan.success)
      {
        throw std::runtime_error("预接触 PTP 规划失败: " + approach_plan.message);
      }
      const auto approach_execution = execute(
        trajectory_executor, approach_plan, "world_z_precontact_execution", 5.0);
      if (!approach_execution.success)
      {
        throw std::runtime_error(
          "预接触 PTP 执行失败: " + approach_execution.message);
      }
    }
    else
    {
      RCLCPP_INFO(
        node->get_logger(),
        "导纳专用测试从预接触工作位启动；本节点跳过自由空间 PTP");
    }

    massage_motion::MotionRequest search;
    search.request_id = "world_z_guarded_search";
    search.motion_type = massage_motion::MotionType::kLin;
    search.target = work_pose(
      precontact_z - search_depth, tool_orientation.orientation);
    search.velocity_scale = 0.01;
    search.acceleration_scale = 0.01;
    search.planning_timeout = 8.0;
    const auto search_plan = planner.plan(search);
    if (!search_plan.success)
    {
      throw std::runtime_error("接触搜索 LIN 规划失败: " + search_plan.message);
    }
    const auto & trajectory = search_plan.trajectory.joint_trajectory;
    if (trajectory.points.empty())
    {
      throw std::runtime_error("接触搜索轨迹为空");
    }
    const auto search_metrics = massage_motion::calculate_trajectory_metrics(
      search_plan.trajectory);
    RCLCPP_INFO(
      node->get_logger(),
      "接触搜索轨迹: points=%zu, duration=%.3f s, max_joint=%.6f rad",
      trajectory.points.size(), search_metrics.duration,
      search_metrics.maximum_joint_travel);

    const std::vector<std::string> joint_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    massage_motion::Ros2ControlComplianceConfig backend_config;
    backend_config.joint_names = joint_names;
    backend_config.wrench_topic = "/massage/ft_sensor/wrench_world";
    backend_config.subtract_startup_wrench_bias = true;
    backend_config.wrench_limit_arming_delay = 0.5;
    backend_config.startup_max_absolute_wrench = 6.0;
    massage_motion::Ros2ControlComplianceController compliance(
      node, backend_config);

    massage_motion::ComplianceRequest request;
    request.request_id = "world_z_admittance_contact";
    request.enabled_axes[2] = true;
    request.max_absolute_wrench[2] = force_limit;
    request.max_joint_displacement = 0.20;
    request.max_linear_displacement = 0.03;
    request.timeout = seconds(trajectory.points.back().time_from_start) +
      hold_duration + 5.0;
    const auto started = compliance.start(request);
    if (!started.success)
    {
      throw std::runtime_error("导纳后端启动失败: " + started.message);
    }
    const auto initial_feedback = compliance.feedback();
    if (initial_feedback.stale || initial_feedback.joint_positions.size() !=
      joint_names.size())
    {
      throw std::runtime_error("无法建立接触搜索关节基线");
    }

    RCLCPP_INFO(
      node->get_logger(),
      "WORLD-Z GUARDED CONTACT: active, precontact_z=%.6f m, "
      "search_depth=%.6f m, threshold=%.3f N, force_limit=%.3f N",
      precontact_z, search_depth, contact_threshold, force_limit);
    bool contact_detected = false;
    double peak_force_delta = 0.0;
    double peak_actual_joint_delta = 0.0;
    const auto search_started = std::chrono::steady_clock::now();
    const double search_duration = seconds(trajectory.points.back().time_from_start);
    auto next_diagnostic = search_started;
    for (const auto & point : trajectory.points)
    {
      std::this_thread::sleep_until(
        search_started + std::chrono::duration<double>(
          seconds(point.time_from_start)));
      const double progress = seconds(point.time_from_start) / search_duration;
      if (enable_simulated_contact_wrench && !simulated_wrench_active &&
        progress >= simulated_contact_trigger_fraction)
      {
        ros_gz_interfaces::msg::EntityWrench injected;
        injected.entity.name = simulated_contact_entity;
        injected.entity.type = ros_gz_interfaces::msg::Entity::LINK;
        injected.wrench.force.z = simulated_contact_force_z;
        simulated_wrench_publisher->publish(injected);
        simulated_wrench_active = true;
        RCLCPP_INFO(
          node->get_logger(),
          "SIMULATED CONTACT WRENCH: active, progress=%.1f%%, "
          "entity=%s, world_Fz=%.3f N",
          progress * 100.0, simulated_contact_entity.c_str(),
          simulated_contact_force_z);
      }
      const auto feedback = compliance.feedback();
      if (feedback.stale ||
        feedback.status != massage_motion::ComplianceStatus::kActive)
      {
        throw std::runtime_error("接触搜索期间导纳反馈失效");
      }
      peak_force_delta = std::max(
        peak_force_delta, std::abs(feedback.wrench[2]));
      for (std::size_t index = 0; index < joint_names.size(); ++index)
      {
        peak_actual_joint_delta = std::max(
          peak_actual_joint_delta,
          std::abs(feedback.joint_positions[index] -
          initial_feedback.joint_positions[index]));
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_diagnostic)
      {
        std::lock_guard<std::mutex> lock(status_mutex);
        RCLCPP_INFO(
          node->get_logger(),
          "导纳诊断: elapsed=%.3f s, delta_Fz=%.6f N, "
          "controller_Fz=%.6f N, reported_admittance_z=%.9f m, "
          "actual_joint=%.6f rad",
          std::chrono::duration<double>(now - search_started).count(),
          feedback.wrench[2],
          has_status ? latest_status.wrench_base.wrench.force.z :
          std::numeric_limits<double>::quiet_NaN(),
          has_status ?
          latest_status.admittance_position.transform.translation.z :
          std::numeric_limits<double>::quiet_NaN(), peak_actual_joint_delta);
        next_diagnostic = now + std::chrono::seconds(1);
      }
      if (std::abs(feedback.wrench[2]) >= contact_threshold)
      {
        contact_detected = true;
        RCLCPP_INFO(
          node->get_logger(),
          "CONTACT CONFIRMED: delta_Fz=%.6f N", feedback.wrench[2]);
        break;
      }
      massage_motion::ComplianceReference reference;
      reference.joint_names = joint_names;
      reference.positions = ordered_positions(trajectory, point, joint_names);
      reference.time_from_start = seconds(point.time_from_start);
      if (reference.positions.empty() || !compliance.update_reference(reference))
      {
        throw std::runtime_error("导纳控制器拒绝接触搜索参考");
      }
    }

    const auto contact_deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(2);
    while (!contact_detected && std::chrono::steady_clock::now() < contact_deadline)
    {
      const auto feedback = compliance.feedback();
      peak_force_delta = std::max(
        peak_force_delta, std::abs(feedback.wrench[2]));
      contact_detected = !feedback.stale &&
        std::abs(feedback.wrench[2]) >= contact_threshold;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!contact_detected)
    {
      throw std::runtime_error(
        "接触搜索结束但 world-Z 力增量未越过阈值: peak_delta_Fz=" +
        std::to_string(peak_force_delta) + " N, actual_joint=" +
        std::to_string(peak_actual_joint_delta) + " rad");
    }

    const auto hold_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(hold_duration);
    while (std::chrono::steady_clock::now() < hold_deadline)
    {
      const auto feedback = compliance.feedback();
      if (feedback.stale ||
        feedback.status != massage_motion::ComplianceStatus::kActive)
      {
        throw std::runtime_error("接触保持期间导纳控制器提前退出");
      }
      peak_force_delta = std::max(
        peak_force_delta, std::abs(feedback.wrench[2]));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    clear_simulated_wrench();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto stopped = compliance.stop();
    if (!stopped.success)
    {
      throw std::runtime_error("导纳停止或控制权恢复失败: " + stopped.message);
    }

    double observed_joint_offset = 0.0;
    double observed_wrench_z = 0.0;
    double observed_peak_controller_wrench_z = 0.0;
    double observed_velocity_z = 0.0;
    double observed_acceleration_z = 0.0;
    bool selected_world_z = false;
    bool has_status_snapshot = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex);
      observed_joint_offset = peak_admittance_joint_offset;
      observed_peak_controller_wrench_z = peak_controller_wrench_z;
      observed_velocity_z = peak_admittance_velocity_z;
      observed_acceleration_z = peak_admittance_acceleration_z;
      has_status_snapshot = has_status;
      if (has_status_snapshot)
      {
        observed_wrench_z = latest_status.wrench_base.wrench.force.z;
        selected_world_z = latest_status.selected_axes.data.size() >= 3U &&
          latest_status.selected_axes.data[2] != 0;
      }
    }
    RCLCPP_INFO(
      node->get_logger(),
      "world-Z 验收量: has_status=%s, selected_z=%s, peak_delta_Fz=%.6f N, "
      "peak_admittance_joint=%.9f rad, peak_controller_Fz=%.6f N, "
      "latest_controller_Fz=%.6f N, peak_velocity_z=%.9f m/s, "
      "peak_acceleration_z=%.9f m/s^2, peak_actual_joint=%.6f rad",
      has_status_snapshot ? "true" : "false",
      selected_world_z ? "true" : "false",
      peak_force_delta, observed_joint_offset, observed_peak_controller_wrench_z,
      observed_wrench_z, observed_velocity_z, observed_acceleration_z,
      peak_actual_joint_delta);
    if (!has_status_snapshot || !selected_world_z ||
      !std::isfinite(observed_joint_offset) || observed_joint_offset < 1.0e-8 ||
      peak_force_delta < contact_threshold || peak_force_delta >= force_limit)
    {
      throw std::runtime_error("world-Z 导纳数值验收未通过");
    }

    const auto retreat_plan = planner.plan(approach);
    if (!retreat_plan.success)
    {
      throw std::runtime_error("接触退出规划失败: " + retreat_plan.message);
    }
    const auto retreat_execution = execute(
      trajectory_executor, retreat_plan, "world_z_retreat_execution", 5.0);
    if (!retreat_execution.success)
    {
      throw std::runtime_error("接触退出执行失败: " + retreat_execution.message);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "WORLD-Z ADMITTANCE SIM: PASS: contact=true, peak_delta_Fz=%.6f N, "
      "peak_admittance_joint=%.9f rad, peak_controller_Fz=%.6f N, "
      "trajectory_controller_restored=true",
      peak_force_delta, observed_joint_offset, observed_peak_controller_wrench_z);
    exit_code = 0;
  }
  catch (const std::exception & exception)
  {
    clear_simulated_wrench();
    RCLCPP_ERROR(node->get_logger(), "WORLD-Z ADMITTANCE SIM: FAIL: %s", exception.what());
  }
  (void)status_subscription;
  ros_executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
