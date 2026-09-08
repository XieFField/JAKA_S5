#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "jaka_msgs/msg/robot_msg.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "massage_jaka/jaka_compliance_controller.hpp"
#include "massage_jaka/jaka_native_cartesian_executor.hpp"
#include "massage_jaka/jaka_native_joint_executor.hpp"
#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/guarded_trajectory_executor.hpp"
#include "massage_motion/hybrid_trajectory_executor.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/pose_ik_competitive_planner.hpp"
#include "massage_task/massage_task_state_machine.hpp"
#include "massage_task/moveit_technique_trajectory_planner.hpp"
#include "massage_task/moveit_tool_alignment_validator.hpp"

namespace
{

class RealContactSceneManager final : public massage_task::IContactSceneManager
{
public:
  explicit RealContactSceneManager(rclcpp::Logger logger) : logger_(logger) {}

  bool prepare() override
  {
    RCLCPP_INFO(logger_, "REAL SCENE: 使用外部给定接触点；不创建虚构人体碰撞体");
    return true;
  }

  bool allow_tool_contact(bool allowed) override
  {
    RCLCPP_INFO(
      logger_, "REAL SCENE: contact phase=%s",
      allowed ? "enabled" : "disabled");
    return true;
  }

  bool restore() override
  {
    RCLCPP_INFO(logger_, "REAL SCENE: task-local contact state restored");
    return true;
  }

private:
  rclcpp::Logger logger_;
};

class RealFtManager final : public massage_task::IForceTorqueManager
{
public:
  RealFtManager(
    rclcpp::Node::SharedPtr node, std::string wrench_topic,
    std::string expected_frame, double maximum_bias,
    std::size_t required_samples, double timeout)
  : expected_frame_(std::move(expected_frame)), maximum_bias_(maximum_bias),
    required_samples_(required_samples), timeout_(timeout)
  {
    zero_client_ = node->create_client<std_srvs::srv::Trigger>(
      "/jaka_driver/zero_ft_sensor");
    subscription_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      std::move(wrench_topic), rclcpp::SensorDataQoS(),
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
    const auto timeout = std::chrono::duration<double>(timeout_);
    if (!zero_client_->wait_for_service(timeout))
    {
      return {false, "zero_ft_sensor 服务不可用", 0.0};
    }
    auto future = zero_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(timeout) != std::future_status::ready ||
      !future.get()->success)
    {
      return {false, "FT 清零失败或超时", 0.0};
    }

    std::unique_lock<std::mutex> lock(mutex_);
    const auto starting_samples = sample_count_;
    if (!condition_.wait_for(
        lock, timeout,
        [&]() {return sample_count_ >= starting_samples + required_samples_;}))
    {
      return {false, "FT 清零后等待世界系反馈超时", 0.0};
    }
    if (latest_.header.frame_id != expected_frame_)
    {
      return {
        false,
        "FT 表达坐标系错误: " + latest_.header.frame_id,
        0.0};
    }
    const auto & wrench = latest_.wrench;
    const double values[] = {
      wrench.force.x, wrench.force.y, wrench.force.z,
      wrench.torque.x, wrench.torque.y, wrench.torque.z};
    double maximum = 0.0;
    for (double value : values)
    {
      if (!std::isfinite(value))
      {
        return {false, "FT 清零后反馈包含非有限值", maximum};
      }
      maximum = std::max(maximum, std::abs(value));
    }
    return {
      maximum <= maximum_bias_,
      maximum <= maximum_bias_ ?
      "真机 FT 清零和世界系基线验证通过" : "真机 FT 清零后基线超限",
      maximum};
  }

private:
  std::string expected_frame_;
  double maximum_bias_;
  std::size_t required_samples_;
  double timeout_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zero_client_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
  std::mutex mutex_;
  std::condition_variable condition_;
  geometry_msgs::msg::WrenchStamped latest_;
  std::size_t sample_count_{0U};
};

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
    "massage_task_real_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor ros_executor;
  ros_executor.add_node(node);
  std::thread spin_thread([&]() {ros_executor.spin();});
  int exit_code = 1;
  try
  {
    bool execute = false;
    bool parameters_confirmed = false;
    std::string technique_name = "push";
    std::string execution_mode_name = "plan_only";
    std::string real_ptp_backend = "native_joint_move";
    node->get_parameter_or("execute", execute, false);
    node->get_parameter_or("parameters_confirmed", parameters_confirmed, false);
    node->get_parameter_or("technique", technique_name, technique_name);
    node->get_parameter_or(
      "execution_mode", execution_mode_name, execution_mode_name);
    node->get_parameter_or(
      "real_ptp_backend", real_ptp_backend, real_ptp_backend);
    if (real_ptp_backend != "native_joint_move" &&
      real_ptp_backend != "moveit_servo")
    {
      throw std::invalid_argument(
              "real_ptp_backend 必须是 native_joint_move 或 moveit_servo");
    }
    RCLCPP_INFO(
      node->get_logger(),
      "PTP BACKEND CONFIG: %s; execute=%s; automatic_fallback=false",
      real_ptp_backend.c_str(), execute ? "true" : "false");
    const auto technique = technique_type(technique_name);
    massage_task::MassageExecutionMode execution_mode;
    if (!massage_task::parse_execution_mode(execution_mode_name, &execution_mode))
    {
      throw std::invalid_argument(
              "execution_mode 必须是 plan_only、free_space 或 compliant_contact");
    }
    const bool plan_only =
      execution_mode == massage_task::MassageExecutionMode::kPlanOnly;
    const bool compliant_contact =
      execution_mode == massage_task::MassageExecutionMode::kCompliantContact;

    if (!plan_only && (!execute || !parameters_confirmed))
    {
      RCLCPP_INFO(
        node->get_logger(),
        "MASSAGE TASK REAL: SAFE IDLE: mode=%s, execute=%s, "
        "parameters_confirmed=%s; "
        "未创建规划器/执行器/JAKA 柔顺客户端，未调用登录、上电、使能、控制器切换或运动",
        execution_mode_name.c_str(),
        execute ? "true" : "false",
        parameters_confirmed ? "true" : "false");
      exit_code = 0;
    }
    else
    {
        int planning_attempts = 5;
        int maximum_ik_attempts = 32;
        int maximum_unique_ik_candidates = 8;
        double ik_base_timeout = 0.02;
        double ik_timeout_per_meter = 0.25;
        double ik_timeout_per_radian = 0.05;
        double ik_minimum_timeout = 0.02;
        double ik_maximum_timeout = 0.50;
        double ik_failure_backoff_factor = 1.35;
        double contact_x = -0.471238630147741;
        double contact_y = 0.156275068796867;
        double contact_z = 0.281381721182422;
        double push_length = 0.08;
        double free_space_velocity_scale = 0.05;
        double technique_velocity_scale = 0.02;
        double technique_acceleration_scale = 0.02;
        double planning_timeout = 60.0;
        double execution_timeout_margin = 15.0;
        double robot_state_timeout = 1.0;
        double native_cartesian_max_speed_mm_s = 100.0;
        double native_cartesian_max_acceleration_mm_s2 = 500.0;
        double native_cartesian_orientation_speed_rad_s = 0.5;
        double native_cartesian_orientation_acceleration_rad_s2 = 1.0;
        double native_cartesian_translation_tolerance_mm = 1.0;
        double native_cartesian_rotation_tolerance_rad = 0.01;
        int push_repetitions = 3;
        int press_cycles = 3;
        double knead_radius = 0.010;
        double knead_cycle_duration = 8.0;
        int knead_cycles = 3;
        double knead_maximum_speed = 0.010;
        node->get_parameter_or("planning_attempts", planning_attempts, 5);
        node->get_parameter_or(
          "maximum_ik_attempts", maximum_ik_attempts,
          maximum_ik_attempts);
        node->get_parameter_or(
          "maximum_unique_ik_candidates", maximum_unique_ik_candidates,
          maximum_unique_ik_candidates);
        node->get_parameter_or(
          "ik_base_timeout", ik_base_timeout, ik_base_timeout);
        node->get_parameter_or(
          "ik_timeout_per_meter", ik_timeout_per_meter,
          ik_timeout_per_meter);
        node->get_parameter_or(
          "ik_timeout_per_radian", ik_timeout_per_radian,
          ik_timeout_per_radian);
        node->get_parameter_or(
          "ik_minimum_timeout", ik_minimum_timeout, ik_minimum_timeout);
        node->get_parameter_or(
          "ik_maximum_timeout", ik_maximum_timeout, ik_maximum_timeout);
        node->get_parameter_or(
          "ik_failure_backoff_factor", ik_failure_backoff_factor,
          ik_failure_backoff_factor);
        node->get_parameter_or("contact_x", contact_x, contact_x);
        node->get_parameter_or("contact_y", contact_y, contact_y);
        node->get_parameter_or("contact_z", contact_z, contact_z);
        node->get_parameter_or("push_length", push_length, push_length);
        node->get_parameter_or(
          "free_space_velocity_scale", free_space_velocity_scale,
          free_space_velocity_scale);
        node->get_parameter_or(
          "technique_velocity_scale", technique_velocity_scale,
          technique_velocity_scale);
        node->get_parameter_or(
          "technique_acceleration_scale", technique_acceleration_scale,
          technique_acceleration_scale);
        node->get_parameter_or(
          "planning_timeout", planning_timeout, planning_timeout);
        node->get_parameter_or(
          "execution_timeout_margin", execution_timeout_margin,
          execution_timeout_margin);
        node->get_parameter_or(
          "robot_state_timeout", robot_state_timeout, robot_state_timeout);
        node->get_parameter_or(
          "native_cartesian_max_speed_mm_s",
          native_cartesian_max_speed_mm_s,
          native_cartesian_max_speed_mm_s);
        node->get_parameter_or(
          "native_cartesian_max_acceleration_mm_s2",
          native_cartesian_max_acceleration_mm_s2,
          native_cartesian_max_acceleration_mm_s2);
        node->get_parameter_or(
          "native_cartesian_orientation_speed_rad_s",
          native_cartesian_orientation_speed_rad_s,
          native_cartesian_orientation_speed_rad_s);
        node->get_parameter_or(
          "native_cartesian_orientation_acceleration_rad_s2",
          native_cartesian_orientation_acceleration_rad_s2,
          native_cartesian_orientation_acceleration_rad_s2);
        node->get_parameter_or(
          "native_cartesian_translation_tolerance_mm",
          native_cartesian_translation_tolerance_mm,
          native_cartesian_translation_tolerance_mm);
        node->get_parameter_or(
          "native_cartesian_rotation_tolerance_rad",
          native_cartesian_rotation_tolerance_rad,
          native_cartesian_rotation_tolerance_rad);
        node->get_parameter_or(
          "push_repetitions", push_repetitions, push_repetitions);
        node->get_parameter_or("press_cycles", press_cycles, press_cycles);
        node->get_parameter_or("knead_radius", knead_radius, knead_radius);
        node->get_parameter_or(
          "knead_cycle_duration", knead_cycle_duration, knead_cycle_duration);
        node->get_parameter_or("knead_cycles", knead_cycles, knead_cycles);
        node->get_parameter_or(
          "knead_maximum_speed", knead_maximum_speed, knead_maximum_speed);
        massage_motion::IkTimeoutPolicy ik_timeout_policy;
        ik_timeout_policy.base_timeout = ik_base_timeout;
        ik_timeout_policy.timeout_per_meter = ik_timeout_per_meter;
        ik_timeout_policy.timeout_per_radian = ik_timeout_per_radian;
        ik_timeout_policy.minimum_timeout = ik_minimum_timeout;
        ik_timeout_policy.maximum_timeout = ik_maximum_timeout;
        ik_timeout_policy.failure_backoff_factor = ik_failure_backoff_factor;
        if (planning_attempts < 1 || planning_attempts > 10 ||
          maximum_ik_attempts < 1 || maximum_ik_attempts > 256 ||
          maximum_unique_ik_candidates < 1 ||
          maximum_unique_ik_candidates > maximum_ik_attempts ||
          !massage_motion::valid_ik_timeout_policy(ik_timeout_policy) ||
          !std::isfinite(contact_x) || !std::isfinite(contact_y) ||
          !std::isfinite(contact_z) ||
          !std::isfinite(push_length) || push_length <= 0.0 ||
          !std::isfinite(free_space_velocity_scale) ||
          free_space_velocity_scale <= 0.0 || free_space_velocity_scale > 1.0 ||
          !std::isfinite(technique_velocity_scale) ||
          technique_velocity_scale <= 0.0 || technique_velocity_scale > 1.0 ||
          !std::isfinite(technique_acceleration_scale) ||
          technique_acceleration_scale <= 0.0 ||
          technique_acceleration_scale > 1.0 ||
          !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
          !std::isfinite(execution_timeout_margin) ||
          execution_timeout_margin <= 0.0 ||
          !std::isfinite(robot_state_timeout) || robot_state_timeout <= 0.0 ||
          !std::isfinite(native_cartesian_max_speed_mm_s) ||
          native_cartesian_max_speed_mm_s <= 0.0 ||
          !std::isfinite(native_cartesian_max_acceleration_mm_s2) ||
          native_cartesian_max_acceleration_mm_s2 <= 0.0 ||
          !std::isfinite(native_cartesian_orientation_speed_rad_s) ||
          native_cartesian_orientation_speed_rad_s <= 0.0 ||
          !std::isfinite(native_cartesian_orientation_acceleration_rad_s2) ||
          native_cartesian_orientation_acceleration_rad_s2 <= 0.0 ||
          !std::isfinite(native_cartesian_translation_tolerance_mm) ||
          native_cartesian_translation_tolerance_mm <= 0.0 ||
          !std::isfinite(native_cartesian_rotation_tolerance_rad) ||
          native_cartesian_rotation_tolerance_rad <= 0.0 ||
          push_repetitions < 1 || push_repetitions > 20 ||
          press_cycles < 1 || press_cycles > 20 ||
          !std::isfinite(knead_radius) || knead_radius <= 0.0 ||
          !std::isfinite(knead_cycle_duration) ||
          knead_cycle_duration <= 0.0 ||
          knead_cycles < 1 || knead_cycles > 20 ||
          !std::isfinite(knead_maximum_speed) ||
          knead_maximum_speed <= 0.0)
        {
          throw std::invalid_argument("真机任务规划次数或接触点参数无效");
        }
        RCLCPP_INFO(
          node->get_logger(),
          "真机任务参数: mode=%s, technique=%s, contact=[%.6f %.6f %.6f] m, "
          "push=[length=%.6f m repetitions=%d], press_cycles=%d, "
          "knead=[radius=%.6f m cycle_duration=%.6f s cycles=%d "
          "maximum_speed=%.6f m/s]",
          execution_mode_name.c_str(), technique_name.c_str(), contact_x,
          contact_y, contact_z, push_length, push_repetitions, press_cycles,
          knead_radius, knead_cycle_duration, knead_cycles,
          knead_maximum_speed);
        RCLCPP_INFO(
          node->get_logger(),
          "动态 IK 配置: attempts=%d, unique=%d, timeout=[base=%.6f s "
          "per_meter=%.6f s/m per_radian=%.6f s/rad min=%.6f s max=%.6f s "
          "backoff=%.6f]",
          maximum_ik_attempts, maximum_unique_ik_candidates,
          ik_base_timeout, ik_timeout_per_meter, ik_timeout_per_radian,
          ik_minimum_timeout, ik_maximum_timeout,
          ik_failure_backoff_factor);

        std::shared_ptr<massage_motion::IComplianceController> compliance;
        std::shared_ptr<massage_task::IForceTorqueManager> ft_manager;
        std::shared_ptr<massage_task::IContactSceneManager> scene;
        if (compliant_contact)
        {
          massage_jaka::JakaComplianceConfig jaka_config;
          jaka_config.wrench_topic = "/massage/ft_sensor/wrench_world";
          jaka_config.wrench_frame = "massage_tool_tip_world_aligned";
          compliance = std::make_shared<massage_jaka::JakaComplianceController>(
            node, jaka_config);
          const auto capabilities = compliance->capabilities();
          if (!capabilities.reference_tracking)
          {
            throw std::runtime_error(
                    "BLOCKED BEFORE MOTION: JAKA 柔顺后端 "
                    "reference_tracking=false；禁止在导纳状态混用未经验证的 "
                    "servo_p/servo_j");
          }
          ft_manager = std::make_shared<RealFtManager>(
            node, "/massage/ft_sensor/wrench_world",
            "massage_tool_tip_world_aligned", 0.5, 10U, 5.0);
          scene = std::make_shared<RealContactSceneManager>(node->get_logger());
        }

        std::mutex robot_state_mutex;
        std::condition_variable robot_state_condition;
        jaka_msgs::msg::RobotMsg latest_robot_state;
        std::chrono::steady_clock::time_point robot_state_received_at;
        std::size_t robot_state_sequence = 0U;
        auto robot_state_subscription =
          node->create_subscription<jaka_msgs::msg::RobotMsg>(
          "/jaka_driver/robot_states", rclcpp::QoS(10),
          [&](const jaka_msgs::msg::RobotMsg::SharedPtr message)
          {
            std::lock_guard<std::mutex> lock(robot_state_mutex);
            latest_robot_state = *message;
            robot_state_received_at = std::chrono::steady_clock::now();
            ++robot_state_sequence;
            robot_state_condition.notify_all();
          });

        if (!plan_only)
        {
          std::unique_lock<std::mutex> lock(robot_state_mutex);
          if (!robot_state_condition.wait_for(
              lock, std::chrono::seconds(5),
              [&]() {return robot_state_sequence > 0U;}))
          {
            throw std::runtime_error("等待 /jaka_driver/robot_states 超时");
          }
          if (latest_robot_state.motion_state != 0 ||
            latest_robot_state.power_state != 1 ||
            latest_robot_state.servo_state != 1 ||
            latest_robot_state.collision_state != 0)
          {
            throw std::runtime_error(
                    "机器人执行前状态不安全: motion=" +
                    std::to_string(latest_robot_state.motion_state) +
                    ", power=" + std::to_string(latest_robot_state.power_state) +
                    ", servo=" + std::to_string(latest_robot_state.servo_state) +
                    ", collision=" +
                    std::to_string(latest_robot_state.collision_state));
          }
        }

        massage_motion::PlannerConfig planner_config;
        planner_config.planning_group = "jaka_s5";
        planner_config.end_effector_link = "massage_tool_tip";
        planner_config.reference_frame = "world";
        planner_config.planning_pipeline = "pilz_industrial_motion_planner";
        auto sdk = std::make_shared<massage_motion::MotionPlanningSdk>(
          node, planner_config);
        auto competition =
          std::make_shared<massage_motion::CompetitiveMotionPlanner>(
          std::vector<massage_motion::PlanningSource>{
            {"motion_sdk", sdk, static_cast<std::size_t>(planning_attempts)}});
        robot_model_loader::RobotModelLoader loader(
          node, "robot_description", true);
        const auto robot_model = loader.getModel();
        if (!robot_model) throw std::runtime_error("无法加载真机 RobotModel");
        const auto * joint_model_group = robot_model->getJointModelGroup(
          planner_config.planning_group);
        if (!joint_model_group)
        {
          throw std::runtime_error(
                  "RobotModel 缺少规划组: " + planner_config.planning_group);
        }
        const auto kinematics_solver = joint_model_group->getSolverInstance();
        if (!kinematics_solver || kinematics_solver->getTipFrames().empty())
        {
          throw std::runtime_error(
                  "本地 IK 求解器未加载；检查任务节点参数 "
                  "robot_description_kinematics.jaka_s5.kinematics_solver");
        }
        RCLCPP_INFO(
          node->get_logger(),
          "LOCAL IK BACKEND: READY: group=%s, base=%s, tip=%s",
          planner_config.planning_group.c_str(),
          kinematics_solver->getBaseFrame().c_str(),
          kinematics_solver->getTipFrame().c_str());
        massage_motion::PoseIkCompetitivePlannerConfig ik_config;
        ik_config.planning_group = planner_config.planning_group;
        ik_config.tip_link = planner_config.end_effector_link;
        ik_config.ik.maximum_attempts =
          static_cast<std::size_t>(maximum_ik_attempts);
        ik_config.ik.maximum_unique_candidates =
          static_cast<std::size_t>(maximum_unique_ik_candidates);
        ik_config.ik.timeout_policy = ik_timeout_policy;
        auto planner = std::make_shared<massage_motion::PoseIkCompetitivePlanner>(
          node, competition, robot_model, ik_config);
        auto technique_planner =
          std::make_shared<massage_task::MoveItTechniqueTrajectoryPlanner>(
          node, planner, technique_velocity_scale,
          technique_acceleration_scale, planning_timeout);
        auto alignment_validator =
          std::make_shared<massage_task::MoveItToolAlignmentValidator>(
          node, robot_model, planner_config.end_effector_link);
        std::shared_ptr<massage_motion::ITrajectoryExecutor> trajectory_executor;
        if (!plan_only)
        {
          auto moveit_backend =
            std::make_shared<massage_motion::MoveItTrajectoryExecutor>(node);
          std::shared_ptr<massage_motion::ITrajectoryExecutor> backend =
            moveit_backend;
          if (real_ptp_backend == "native_joint_move")
          {
            massage_jaka::JakaNativeJointExecutorConfig native_config;
            native_config.joint_state_timeout = robot_state_timeout;
            native_config.ptp.maximum_start_error = 0.002;
            native_config.ptp.maximum_path_deviation = 0.002;
            native_config.ptp.maximum_speed = 0.20;
            native_config.ptp.maximum_acceleration = 0.50;
            native_config.ptp.endpoint_tolerance = 0.002;
            native_config.ptp.timeout_margin = execution_timeout_margin;
            auto native_backend =
              std::make_shared<massage_jaka::JakaNativeJointExecutor>(
                node, native_config);
            massage_jaka::JakaNativeCartesianExecutorConfig cartesian_config;
            cartesian_config.joint_state_timeout = robot_state_timeout;
            cartesian_config.maximum_start_error = 0.002;
            cartesian_config.maximum_linear_speed_mm_s =
              native_cartesian_max_speed_mm_s;
            cartesian_config.maximum_linear_acceleration_mm_s2 =
              native_cartesian_max_acceleration_mm_s2;
            cartesian_config.orientation_speed_rad_s =
              native_cartesian_orientation_speed_rad_s;
            cartesian_config.orientation_acceleration_rad_s2 =
              native_cartesian_orientation_acceleration_rad_s2;
            cartesian_config.translation_tolerance_mm =
              native_cartesian_translation_tolerance_mm;
            cartesian_config.rotation_tolerance_rad =
              native_cartesian_rotation_tolerance_rad;
            auto native_cartesian_backend =
              std::make_shared<massage_jaka::JakaNativeCartesianExecutor>(
                node, cartesian_config);
            backend = std::make_shared<massage_motion::HybridTrajectoryExecutor>(
              native_backend, native_cartesian_backend, moveit_backend);
          }
          RCLCPP_INFO(
            node->get_logger(),
            "PTP BACKEND: %s; LIN/CIRC BACKEND: %s; automatic_fallback=false",
            real_ptp_backend.c_str(),
            real_ptp_backend == "native_joint_move" ?
            "native_cartesian" : "moveit_servo");
          trajectory_executor =
            std::make_shared<massage_motion::GuardedTrajectoryExecutor>(
            backend,
            [&]() -> massage_motion::ExecutionValidationResult
            {
              std::lock_guard<std::mutex> lock(robot_state_mutex);
              if (robot_state_sequence == 0U)
              {
                return {
                  false, massage_motion::ExecutionError::kRejected,
                  "尚未收到机器人状态"};
              }
              const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                robot_state_received_at).count();
              if (age > robot_state_timeout)
              {
                return {
                  false, massage_motion::ExecutionError::kRejected,
                  "机器人状态已过期"};
              }
              if (latest_robot_state.motion_state != 0 ||
                latest_robot_state.power_state != 1 ||
                latest_robot_state.servo_state != 1 ||
                latest_robot_state.collision_state != 0)
              {
                return {
                  false, massage_motion::ExecutionError::kRejected,
                  "机器人不满足静止、上电、使能、无碰撞条件"};
              }
              return {
                true, massage_motion::ExecutionError::kNone, "ready"};
            });
        }
        massage_task::MassageTaskStateMachine machine(
          planner, technique_planner, alignment_validator, alignment_validator,
          trajectory_executor,
          compliance, ft_manager, scene);

        massage_task::MassageTaskRequest request;
        request.task_id = "real_" + technique_name;
        request.execution_mode = execution_mode;
        request.technique = technique;
        request.standby_target = massage_motion::JointTarget{{
          -3.160921066038505, 1.6080484013290657,
          -2.6790895150928824, 2.6720179543589007,
          0.027910401738015497, -2.4753825550678896}};
        request.contact_pose.header.frame_id = "world";
        request.contact_pose.pose.position.x = contact_x;
        request.contact_pose.pose.position.y = contact_y;
        request.contact_pose.pose.position.z = contact_z;
        request.contact_pose.pose.orientation.w = 1.0;
        request.surface_normal.z = 1.0;
        request.surface_tangent.y = 1.0;
        request.execution_environment = "real";
        request.execute = execute;
        request.parameters_confirmed = parameters_confirmed;
        request.free_space_velocity_scale = free_space_velocity_scale;
        request.contact_velocity_scale = 0.005;
        request.precontact_distance = 0.020;
        request.contact_search_depth = 0.010;
        request.contact_threshold = 0.5;
        request.target_normal_force = 1.0;
        request.maximum_normal_force = 5.0;
        request.contact_wait_timeout = 2.0;
        request.force_ramp_timeout = 3.0;
        request.planning_timeout = planning_timeout;
        request.execution_timing.margin = execution_timeout_margin;
        request.compliance_request.request_id = request.task_id + "_compliance";
        request.compliance_request.enabled_axes[2] = true;
        request.compliance_request.target_wrench[2] = -1.0;
        request.compliance_request.max_absolute_wrench = {
          5.0, 5.0, 5.0, 1.0, 1.0, 1.0};
        request.compliance_request.max_joint_displacement = 0.20;
        request.compliance_request.max_linear_displacement = 0.12;
        request.compliance_request.timeout = 30.0;
        request.push.direction_y = 1.0;
        request.push.length = push_length;
        request.push_repetitions = static_cast<std::size_t>(push_repetitions);
        request.push.speed = 0.005;
        request.push.sample_period = 0.02;
        request.push.maximum_speed = 0.01;
        request.press.stroke = 0.004;
        request.press.cycle_duration = 3.0;
        request.press.cycles = static_cast<std::size_t>(press_cycles);
        request.press.sample_period = 0.02;
        request.press.maximum_speed = 0.01;
        request.knead.radius = knead_radius;
        request.knead.cycle_duration = knead_cycle_duration;
        request.knead.cycles = static_cast<std::size_t>(knead_cycles);
        request.knead.sample_period = 0.02;
        request.knead.maximum_speed = knead_maximum_speed;

        if (execution_mode == massage_task::MassageExecutionMode::kFreeSpace)
        {
          auto preflight_request = request;
          preflight_request.task_id += "_preflight";
          preflight_request.execution_mode =
            massage_task::MassageExecutionMode::kPlanOnly;
          preflight_request.execute = false;
          preflight_request.parameters_confirmed = false;
          RCLCPP_INFO(
            node->get_logger(),
            "FREE-SPACE PRE-EXECUTION PREFLIGHT: START: "
            "将在任何运动前规划并校验完整路线");
          const auto preflight_result = machine.run(preflight_request);
          if (!preflight_result.success)
          {
            throw std::runtime_error(
                    "BLOCKED BEFORE MOTION: 全流程预规划失败；未发送运动 Goal: " +
                    preflight_result.message);
          }
          if (!machine.reset())
          {
            throw std::runtime_error(
                    "BLOCKED BEFORE MOTION: 全流程预规划后状态机无法复位；未发送运动 Goal");
          }
          RCLCPP_INFO(
            node->get_logger(),
            "FREE-SPACE PRE-EXECUTION PREFLIGHT: PASS: route_states=%zu, "
            "planned_sessions=%zu, planned_technique_cycles=%zu；"
            "现在才允许进入真机执行",
            preflight_result.state_trace.size(),
            preflight_result.completed_repetitions,
            preflight_result.completed_technique_cycles);
        }

        const auto result = machine.run(request);
        if (!result.success)
        {
          throw std::runtime_error(result.message);
        }
        RCLCPP_INFO(
          node->get_logger(),
          "MASSAGE TASK REAL: PASS: mode=%s, technique=%s, peak_force=%.6f N, "
          "completed_sessions=%zu, completed_technique_cycles=%zu, recovery=%s",
          execution_mode_name.c_str(), technique_name.c_str(),
          result.peak_normal_force,
          result.completed_repetitions, result.completed_technique_cycles,
          result.recovery_succeeded ? "true" : "false");
        exit_code = 0;
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(node->get_logger(), "MASSAGE TASK REAL: FAIL: %s", exception.what());
    exit_code = 6;
  }

  ros_executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
