#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"

#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/pose_ik_competitive_planner.hpp"
#include "massage_motion/ros2_control_compliance_controller.hpp"
#include "massage_task/massage_task_state_machine.hpp"
#include "massage_task/moveit_technique_trajectory_planner.hpp"
#include "massage_task/moveit_tool_alignment_validator.hpp"

namespace
{

class SimulationSceneManager final : public massage_task::IContactSceneManager
{
public:
  explicit SimulationSceneManager(rclcpp::Logger logger) : logger_(logger) {}

  bool prepare() override
  {
    RCLCPP_INFO(logger_, "SIM SCENE: free-space planning scene ready");
    return true;
  }

  bool allow_tool_contact(bool allowed) override
  {
    RCLCPP_INFO(
      logger_, "SIM SCENE: tool contact permission=%s",
      allowed ? "true" : "false");
    return true;
  }

  bool restore() override
  {
    RCLCPP_INFO(logger_, "SIM SCENE: contact permissions restored");
    return true;
  }

private:
  rclcpp::Logger logger_;
};

class SimulationFtManager final : public massage_task::IForceTorqueManager
{
public:
  SimulationFtManager(
    rclcpp::Node::SharedPtr node, const std::string & topic,
    double maximum_bias, std::size_t required_samples)
  : maximum_bias_(maximum_bias), required_samples_(required_samples)
  {
    subscription_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = *message;
        ++sample_count_;
        condition_.notify_all();
      });
  }

  massage_task::ForceTorqueResult zero_and_validate() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto starting_samples = sample_count_;
    if (!condition_.wait_for(
        lock, std::chrono::seconds(2),
        [&]() {return sample_count_ >= starting_samples + required_samples_;}))
    {
      return {false, "等待世界系六维力反馈超时", 0.0};
    }
    const auto & wrench = latest_.wrench;
    const std::array<double, 6> values{
      wrench.force.x, wrench.force.y, wrench.force.z,
      wrench.torque.x, wrench.torque.y, wrench.torque.z};
    double maximum = 0.0;
    for (double value : values)
    {
      if (!std::isfinite(value))
      {
        return {false, "世界系六维力反馈包含非有限数值", maximum};
      }
      maximum = std::max(maximum, std::abs(value));
    }
    return {
      maximum <= maximum_bias_,
      maximum <= maximum_bias_ ?
      "仿真 FT 基线采集和数值校验通过" : "仿真 FT 基线超过允许范围",
      maximum};
  }

private:
  double maximum_bias_;
  std::size_t required_samples_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
  std::mutex mutex_;
  std::condition_variable condition_;
  geometry_msgs::msg::WrenchStamped latest_;
  std::size_t sample_count_{0U};
};

geometry_msgs::msg::PoseStamped work_pose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = -0.471238630147741;
  pose.pose.position.y = 0.156275068796867;
  pose.pose.position.z = 0.281381721182422;
  // The state machine derives the authoritative orientation from the surface
  // normal and tangent; this identity is only a finite input placeholder.
  pose.pose.orientation.w = 1.0;
  return pose;
}

massage_motion::TechniquePathType technique_type(const std::string & value)
{
  if (value == "push") return massage_motion::TechniquePathType::kPush;
  if (value == "press") return massage_motion::TechniquePathType::kPress;
  if (value == "knead") return massage_motion::TechniquePathType::kKnead;
  throw std::invalid_argument("technique 必须是 push、press 或 knead");
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "massage_task_sim_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor ros_executor;
  ros_executor.add_node(node);
  std::thread spin_thread([&]() {ros_executor.spin();});

  int exit_code = 1;
  std::atomic_bool stop_injector{false};
  std::thread injector_thread;
  try
  {
    std::string technique_name = "push";
    std::string execution_mode_name = "compliant_contact";
    int planning_attempts = 5;
    double simulated_contact_force_z = -0.25;
    double simulated_contact_ramp_duration = 1.0;
    double target_normal_force = 0.20;
    double maximum_joint_displacement = 0.75;
    double maximum_linear_displacement = 0.12;
    double work_ready_clearance = 0.05;
    double maximum_tool_axis_error_degrees = 3.0;
    double push_length = 0.08;
    int push_repetitions = 3;
    int press_cycles = 3;
    int knead_cycles = 3;
    node->get_parameter_or("technique", technique_name, technique_name);
    node->get_parameter_or(
      "execution_mode", execution_mode_name, execution_mode_name);
    massage_task::MassageExecutionMode execution_mode;
    if (!massage_task::parse_execution_mode(execution_mode_name, &execution_mode))
    {
      throw std::invalid_argument(
              "execution_mode 必须是 plan_only、free_space 或 compliant_contact");
    }
    node->get_parameter_or("planning_attempts", planning_attempts, planning_attempts);
    node->get_parameter_or(
      "simulated_contact_force_z", simulated_contact_force_z,
      simulated_contact_force_z);
    node->get_parameter_or(
      "simulated_contact_ramp_duration", simulated_contact_ramp_duration,
      simulated_contact_ramp_duration);
    node->get_parameter_or(
      "target_normal_force", target_normal_force, target_normal_force);
    node->get_parameter_or(
      "maximum_joint_displacement", maximum_joint_displacement,
      maximum_joint_displacement);
    node->get_parameter_or(
      "maximum_linear_displacement", maximum_linear_displacement,
      maximum_linear_displacement);
    node->get_parameter_or(
      "work_ready_clearance", work_ready_clearance, work_ready_clearance);
    node->get_parameter_or(
      "maximum_tool_axis_error_degrees", maximum_tool_axis_error_degrees,
      maximum_tool_axis_error_degrees);
    node->get_parameter_or("push_length", push_length, push_length);
    node->get_parameter_or(
      "push_repetitions", push_repetitions, push_repetitions);
    node->get_parameter_or("press_cycles", press_cycles, press_cycles);
    node->get_parameter_or("knead_cycles", knead_cycles, knead_cycles);
    if (planning_attempts < 1 || planning_attempts > 10 ||
      !std::isfinite(simulated_contact_force_z) ||
      std::abs(simulated_contact_force_z) < 1.0e-6 ||
      !std::isfinite(simulated_contact_ramp_duration) ||
      simulated_contact_ramp_duration <= 0.0 ||
      !std::isfinite(target_normal_force) || target_normal_force < 0.15 ||
      target_normal_force >= 5.0 ||
      !std::isfinite(maximum_joint_displacement) ||
      maximum_joint_displacement <= 0.20 ||
      !std::isfinite(maximum_linear_displacement) ||
      maximum_linear_displacement <= 0.03 ||
      !std::isfinite(work_ready_clearance) || work_ready_clearance <= 0.0 ||
      !std::isfinite(maximum_tool_axis_error_degrees) ||
      maximum_tool_axis_error_degrees <= 0.0 ||
      maximum_tool_axis_error_degrees > 30.0 ||
      !std::isfinite(push_length) || push_length <= 0.0 ||
      push_repetitions < 1 || push_repetitions > 20 ||
      press_cycles < 1 || press_cycles > 20 ||
      knead_cycles < 1 || knead_cycles > 20)
    {
      throw std::invalid_argument(
              "仿真接触力必须是有限非零值，关节/TCP 位移上限必须覆盖名义行程");
    }

    massage_motion::PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "massage_tool_tip";
    planner_config.reference_frame = "world";
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";
    auto sdk = std::make_shared<massage_motion::MotionPlanningSdk>(
      node, planner_config);
    auto competitive_planner =
      std::make_shared<massage_motion::CompetitiveMotionPlanner>(
      std::vector<massage_motion::PlanningSource>{
        {"motion_sdk", sdk, static_cast<std::size_t>(planning_attempts)}});
    robot_model_loader::RobotModelLoader robot_model_loader(
      node, "robot_description", true);
    const auto robot_model = robot_model_loader.getModel();
    if (!robot_model)
    {
      throw std::runtime_error("无法加载带运动学插件的 RobotModel");
    }
    massage_motion::PoseIkCompetitivePlannerConfig pose_ik_config;
    pose_ik_config.planning_group = planner_config.planning_group;
    pose_ik_config.tip_link = planner_config.end_effector_link;
    pose_ik_config.ik.maximum_attempts = 32U;
    pose_ik_config.ik.maximum_unique_candidates = 8U;
    auto planner = std::make_shared<massage_motion::PoseIkCompetitivePlanner>(
      node, competitive_planner, robot_model, pose_ik_config);
    auto technique_planner =
      std::make_shared<massage_task::MoveItTechniqueTrajectoryPlanner>(
      node, planner, 0.08, 0.05, 8.0);
    auto alignment_validator =
      std::make_shared<massage_task::MoveItToolAlignmentValidator>(
      node, robot_model, planner_config.end_effector_link);
    auto executor = std::make_shared<massage_motion::MoveItTrajectoryExecutor>(node);
    massage_motion::Ros2ControlComplianceConfig compliance_config;
    compliance_config.joint_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    // Full-flow orchestration uses a deterministic world-frame wrench source.
    // The physical Gazebo FT/admittance response is verified independently by
    // world_z_admittance_demo, avoiding a free-space ApplyLinkWrench feedback loop.
    compliance_config.wrench_topic = "/massage/task_test/wrench_world";
    compliance_config.subtract_startup_wrench_bias = true;
    compliance_config.wrench_limit_arming_delay = 0.5;
    compliance_config.startup_max_absolute_wrench = 6.0;
    auto compliance =
      std::make_shared<massage_motion::Ros2ControlComplianceController>(
      node, compliance_config);
    auto ft_manager = std::make_shared<SimulationFtManager>(
      node, "/massage/task_test/wrench_world", 3.0, 5U);
    auto scene = std::make_shared<SimulationSceneManager>(node->get_logger());
    massage_task::MassageTaskStateMachine machine(
      planner, technique_planner, alignment_validator, alignment_validator,
      executor, compliance,
      ft_manager, scene);

    auto wrench_publisher = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/massage/task_test/wrench_world", rclcpp::SensorDataQoS());
    injector_thread = std::thread([&]()
      {
        bool ramp_started = false;
        bool ramp_logged = false;
        bool cycle_active = false;
        std::size_t ramp_step = 0U;
        const auto ramp_steps = static_cast<std::size_t>(std::ceil(
          simulated_contact_ramp_duration / 0.02));
        auto guarded_since = std::chrono::steady_clock::time_point{};
        auto previous_state = massage_task::MassageTaskState::kIdle;
        while (rclcpp::ok() && !stop_injector.load())
        {
          const auto state = machine.state();
          if (state == massage_task::MassageTaskState::kGuardedContactEntry &&
            previous_state != massage_task::MassageTaskState::kGuardedContactEntry)
          {
            guarded_since = std::chrono::steady_clock::now();
            ramp_started = false;
            ramp_logged = false;
            ramp_step = 0U;
            cycle_active = true;
          }
          if (cycle_active && !ramp_started &&
            std::chrono::steady_clock::now() - guarded_since >=
            std::chrono::milliseconds(800))
          {
            ramp_started = true;
          }
          if (state == massage_task::MassageTaskState::kReleaseNormalForce ||
            state == massage_task::MassageTaskState::kInterCycleRetreat ||
            state == massage_task::MassageTaskState::kInterCycleReturn ||
            state == massage_task::MassageTaskState::kRetreat ||
            state == massage_task::MassageTaskState::kReturnOverhead ||
            state == massage_task::MassageTaskState::kMoveSafeReturnExit ||
            state == massage_task::MassageTaskState::kComplete ||
            state == massage_task::MassageTaskState::kFault)
          {
            cycle_active = false;
          }
          geometry_msgs::msg::WrenchStamped wrench;
          wrench.header.stamp = node->now();
          wrench.header.frame_id = "massage_tool_tip_world_aligned";
          if (cycle_active && ramp_started)
          {
            ramp_step = std::min(ramp_step + 1U, ramp_steps);
            wrench.wrench.force.z = simulated_contact_force_z *
              static_cast<double>(ramp_step) / static_cast<double>(ramp_steps);
          }
          wrench_publisher->publish(wrench);
          if (ramp_started && ramp_step == ramp_steps && !ramp_logged)
          {
            RCLCPP_INFO(
              node->get_logger(), "SIM CONTACT: deterministic ramp complete, "
              "world_Fz=%.3f N, duration=%.3f s",
              simulated_contact_force_z, simulated_contact_ramp_duration);
            ramp_logged = true;
          }
          previous_state = state;
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      });

    massage_task::MassageTaskRequest request;
    request.task_id = "gazebo_" + technique_name;
    request.execution_mode = execution_mode;
    request.technique = technique_type(technique_name);
    request.standby_target = massage_motion::JointTarget{{
      -3.160921066038505,
      1.6080484013290657,
      -2.6790895150928824,
      2.6720179543589007,
      0.027910401738015497,
      -2.4753825550678896}};
    request.contact_pose = work_pose();
    request.surface_normal.z = 1.0;
    request.surface_tangent.y = 1.0;
    request.precontact_distance = 0.020;
    request.work_ready_clearance = work_ready_clearance;
    request.maximum_tool_axis_error =
      maximum_tool_axis_error_degrees * M_PI / 180.0;
    request.contact_search_depth = 0.010;
    request.free_space_velocity_scale = 0.15;
    request.contact_velocity_scale = 0.02;
    request.contact_wait_timeout = 2.0;
    request.force_ramp_timeout = 2.0;
    request.contact_threshold = 0.15;
    request.target_normal_force = target_normal_force;
    request.maximum_normal_force = 5.0;
    request.execution_environment = "simulation";
    request.execute =
      execution_mode != massage_task::MassageExecutionMode::kPlanOnly;
    request.return_to_standby = true;
    request.push_repetitions = static_cast<std::size_t>(push_repetitions);
    request.execution_timing.margin = 8.0;
    request.compliance_request.request_id = request.task_id + "_compliance";
    request.compliance_request.enabled_axes[2] = true;
    request.compliance_request.max_absolute_wrench[2] = 5.0;
    request.compliance_request.max_joint_displacement =
      maximum_joint_displacement;
    // Nominal pre-contact plus guarded-search travel is 30 mm. Keep a
    // separate tracking/admittance margin while still failing closed.
    request.compliance_request.max_linear_displacement =
      maximum_linear_displacement;
    request.compliance_request.timeout = 40.0;
    request.push.direction_y = 1.0;
    request.push.length = push_length;
    request.push.speed = 0.01;
    request.push.sample_period = 0.02;
    request.push.maximum_speed = 0.02;
    request.press.stroke = 0.004;
    request.press.cycle_duration = 2.0;
    request.press.cycles = static_cast<std::size_t>(press_cycles);
    request.press.sample_period = 0.02;
    request.press.maximum_speed = 0.02;
    request.knead.radius = 0.010;
    request.knead.cycle_duration = 4.0;
    request.knead.cycles = static_cast<std::size_t>(knead_cycles);
    request.knead.sample_period = 0.02;
    request.knead.maximum_speed = 0.02;

    RCLCPP_INFO(
      node->get_logger(),
      "MASSAGE TASK SIM: START mode=%s, technique=%s, standby=project_default, "
      "contact=[%.6f %.6f %.6f] m, target_force=%.3f N, "
      "joint_limit=%.3f rad, selected_tcp_limit=%.3f m, "
      "push_length=%.3f m, push_repetitions=%d, press_cycles=%d, knead_cycles=%d",
      execution_mode_name.c_str(), technique_name.c_str(),
      request.contact_pose.pose.position.x,
      request.contact_pose.pose.position.y, request.contact_pose.pose.position.z,
      request.target_normal_force,
      request.compliance_request.max_joint_displacement,
      request.compliance_request.max_linear_displacement,
      push_length, push_repetitions, press_cycles, knead_cycles);
    const auto result = machine.run(request);
    stop_injector.store(true);
    if (injector_thread.joinable()) injector_thread.join();

    std::string trace;
    for (const auto state : result.state_trace)
    {
      if (!trace.empty()) trace += " -> ";
      trace += massage_task::to_string(state);
    }
    RCLCPP_INFO(node->get_logger(), "MASSAGE TASK STATE TRACE: %s", trace.c_str());
    if (!result.success)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "MASSAGE TASK RESULT: success=false, contact=%s, target_force=%s, "
        "peak_normal_force=%.6f N, recovery_attempted=%s, recovery=%s, "
        "primary_error=%d, compliance_message=%s",
        result.contact_detected ? "true" : "false",
        result.target_force_reached ? "true" : "false",
        result.peak_normal_force,
        result.recovery_attempted ? "true" : "false",
        result.recovery_succeeded ? "true" : "false",
        static_cast<int>(result.primary_error),
        result.compliance_result.message.c_str());
      throw std::runtime_error(result.message);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "MASSAGE TASK SIM: PASS: mode=%s, technique=%s, contact=%s, target_force=%s, "
      "peak_normal_force=%.6f N, peak_joint=%.6f rad, "
      "peak_selected_tcp=%.6f m, ft_bias=%.6f, completed_sessions=%zu, "
      "completed_technique_cycles=%zu, "
      "recovery=%s",
      execution_mode_name.c_str(), technique_name.c_str(),
      result.contact_detected ? "true" : "false",
      result.target_force_reached ? "true" : "false",
      result.peak_normal_force,
      result.compliance_result.peak_joint_displacement,
      result.compliance_result.peak_selected_axis_translation,
      result.ft_result.maximum_absolute_bias,
      result.completed_repetitions,
      result.completed_technique_cycles,
      result.recovery_succeeded ? "true" : "false");
    exit_code = 0;
  }
  catch (const std::exception & exception)
  {
    stop_injector.store(true);
    if (injector_thread.joinable()) injector_thread.join();
    RCLCPP_ERROR(node->get_logger(), "MASSAGE TASK SIM: FAIL: %s", exception.what());
    exit_code = 6;
  }

  ros_executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
