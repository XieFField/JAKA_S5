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

#include "jaka_msgs/msg/robot_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_bringup/default_targets.hpp"
#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/guarded_trajectory_executor.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"
#include "massage_task/minimal_task_state_machine.hpp"

namespace
{

bool robot_ready(const jaka_msgs::msg::RobotMsg & state)
{
  return state.motion_state == 0 && state.power_state == 1 &&
         state.servo_state == 1 && state.collision_state == 0;
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
      "规划候选[%zu]: source=%s attempt=%zu accepted=%s score=%.6f "
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
    "return_home_real",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  bool execute = false;
  bool parameters_confirmed = false;
  int planning_attempts = 3;
  double velocity_scale = 0.1;
  double acceleration_scale = 0.1;
  double planning_timeout = 5.0;
  double execution_timeout_margin = 15.0;
  double state_timeout = 3.0;
  double readiness_timeout = 60.0;
  double feedback_timeout = 1.0;
  double maximum_joint_travel = 3.5;
  double endpoint_tolerance = 0.002;
  node->get_parameter_or("execute", execute, false);
  node->get_parameter_or("parameters_confirmed", parameters_confirmed, false);
  node->get_parameter_or("planning_attempts", planning_attempts, 3);
  node->get_parameter_or("velocity_scale", velocity_scale, 0.02);
  node->get_parameter_or("acceleration_scale", acceleration_scale, 0.02);
  node->get_parameter_or("planning_timeout", planning_timeout, 5.0);
  node->get_parameter_or(
    "execution_timeout_margin", execution_timeout_margin, 15.0);
  node->get_parameter_or("state_timeout", state_timeout, 3.0);
  node->get_parameter_or("readiness_timeout", readiness_timeout, 60.0);
  node->get_parameter_or("feedback_timeout", feedback_timeout, 1.0);
  node->get_parameter_or("maximum_joint_travel", maximum_joint_travel, 3.5);
  node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.002);

  if (execute && !parameters_confirmed)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "真机回待机执行需要 execute:=true 和 parameters_confirmed:=true");
    rclcpp::shutdown();
    return 2;
  }
  if (planning_attempts <= 0 || !std::isfinite(execution_timeout_margin) ||
    execution_timeout_margin < 0.0 || !std::isfinite(state_timeout) ||
    state_timeout <= 0.0 || !std::isfinite(readiness_timeout) ||
    readiness_timeout <= 0.0 || !std::isfinite(feedback_timeout) ||
    feedback_timeout <= 0.0 || !std::isfinite(endpoint_tolerance) ||
    endpoint_tolerance <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "真机回待机参数无效");
    rclcpp::shutdown();
    return 2;
  }

  std::mutex state_mutex;
  std::condition_variable state_condition;
  sensor_msgs::msg::JointState latest_joint_state;
  jaka_msgs::msg::RobotMsg latest_robot_state;
  std::uint64_t joint_sequence = 0;
  std::uint64_t robot_sequence = 0;
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
  try
  {
    {
      std::unique_lock<std::mutex> lock(state_mutex);
      if (!state_condition.wait_for(
          lock, std::chrono::duration<double>(state_timeout),
          [&]() {return joint_sequence > 0 && robot_sequence > 0;}))
      {
        throw std::runtime_error(
                "等待 /joint_states 或 /jaka_driver/robot_states 超时");
      }
      if (execute && !robot_ready(latest_robot_state))
      {
        RCLCPP_INFO(
          node->get_logger(),
          "等待机器人进入静止、上电、使能、无碰撞状态，最长 %.1f 秒",
          readiness_timeout);
        if (!state_condition.wait_for(
            lock, std::chrono::duration<double>(readiness_timeout),
            [&]() {return robot_ready(latest_robot_state);} ))
        {
          throw std::runtime_error("等待真机可执行状态超时");
        }
      }
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

    std::shared_ptr<massage_motion::ITrajectoryExecutor> trajectory_executor;
    if (execute)
    {
      auto backend =
        std::make_shared<massage_motion::MoveItTrajectoryExecutor>(node);
      trajectory_executor =
        std::make_shared<massage_motion::GuardedTrajectoryExecutor>(
        backend,
        [&]() -> massage_motion::ExecutionValidationResult
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          if (joint_sequence == 0 || robot_sequence == 0)
          {
            return {
              false, massage_motion::ExecutionError::kRejected,
              "尚未收到关节状态或机器人状态"};
          }
          const auto now = std::chrono::steady_clock::now();
          const auto joint_age =
            std::chrono::duration<double>(now - joint_received_at).count();
          const auto robot_age =
            std::chrono::duration<double>(now - robot_received_at).count();
          if (joint_age > feedback_timeout || robot_age > feedback_timeout)
          {
            return {
              false, massage_motion::ExecutionError::kRejected,
              "关节状态或机器人状态已过期"};
          }
          if (!robot_ready(latest_robot_state))
          {
            return {
              false, massage_motion::ExecutionError::kRejected,
              "机器人执行状态在规划期间发生变化"};
          }
          return {true, massage_motion::ExecutionError::kNone, "ready"};
        });
    }

    massage_task::MinimalTaskStateMachine task(planner, trajectory_executor);
    massage_task::TaskRequest task_request;
    task_request.task_id = "return_home_real";
    task_request.execute = execute;
    task_request.execution_timing.margin = execution_timeout_margin;
    task_request.motion_request.request_id = "massage_home";
    task_request.motion_request.motion_type = massage_motion::MotionType::kPtp;
    task_request.motion_request.target =
      massage_bringup::make_massage_home_target();
    task_request.motion_request.velocity_scale = velocity_scale;
    task_request.motion_request.acceleration_scale = acceleration_scale;
    task_request.motion_request.planning_timeout = planning_timeout;

    const auto task_result = task.run(task_request);
    log_competition(node->get_logger(), planner->last_report());
    if (!task_result.success)
    {
      RCLCPP_ERROR(
        node->get_logger(), "真机回待机任务失败: %s",
        task_result.message.c_str());
      exit_code = 3;
    }
    else if (!execute)
    {
      RCLCPP_INFO(node->get_logger(), "真机回待机只规划完成，未发送运动命令");
      exit_code = 0;
    }
    else
    {
      RCLCPP_INFO(
        node->get_logger(), "真机回待机执行结果: %s",
        task_result.message.c_str());
      sensor_msgs::msg::JointState final_state;
      {
        std::unique_lock<std::mutex> lock(state_mutex);
        const auto previous_sequence = joint_sequence;
        if (!state_condition.wait_for(
            lock, std::chrono::duration<double>(state_timeout),
            [&]() {return joint_sequence > previous_sequence;}))
        {
          throw std::runtime_error("等待回待机执行后的 /joint_states 超时");
        }
        final_state = latest_joint_state;
      }
      const auto endpoint = massage_motion::calculate_trajectory_endpoint_error(
        task_result.plan_result.trajectory, final_state);
      if (!endpoint.valid || endpoint.max_absolute_error > endpoint_tolerance)
      {
        throw std::runtime_error("回待机终点关节误差超过容差");
      }
      RCLCPP_INFO(
        node->get_logger(), "真机回待机完成，最大终点误差 %.9f rad",
        endpoint.max_absolute_error);
      exit_code = 0;
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(node->get_logger(), "真机回待机入口异常: %s", exception.what());
    exit_code = 4;
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
