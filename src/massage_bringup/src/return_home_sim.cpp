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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "massage_bringup/default_targets.hpp"
#include "massage_motion/competitive_motion_planner.hpp"
#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/trajectory_endpoint_error.hpp"
#include "massage_task/minimal_task_state_machine.hpp"

namespace
{

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
    "return_home_sim",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  bool execute = false;
  int planning_attempts = 3;
  double velocity_scale = 0.1;
  double acceleration_scale = 0.1;
  double planning_timeout = 5.0;
  double execution_timeout_margin = 5.0;
  double joint_state_timeout = 3.0;
  double maximum_joint_travel = 3.5;
  double endpoint_tolerance = 0.002;
  node->get_parameter_or("execute", execute, false);
  node->get_parameter_or("planning_attempts", planning_attempts, 3);
  node->get_parameter_or("velocity_scale", velocity_scale, 0.1);
  node->get_parameter_or("acceleration_scale", acceleration_scale, 0.1);
  node->get_parameter_or("planning_timeout", planning_timeout, 5.0);
  node->get_parameter_or(
    "execution_timeout_margin", execution_timeout_margin, 5.0);
  node->get_parameter_or("joint_state_timeout", joint_state_timeout, 3.0);
  node->get_parameter_or("maximum_joint_travel", maximum_joint_travel, 3.5);
  node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.002);

  if (planning_attempts <= 0 || !std::isfinite(execution_timeout_margin) ||
    execution_timeout_margin < 0.0 || !std::isfinite(joint_state_timeout) ||
    joint_state_timeout <= 0.0 || !std::isfinite(endpoint_tolerance) ||
    endpoint_tolerance <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "仿真回待机参数无效");
    rclcpp::shutdown();
    return 2;
  }

  std::mutex state_mutex;
  std::condition_variable state_condition;
  sensor_msgs::msg::JointState latest_state;
  std::uint64_t state_sequence = 0;
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
    {
      std::unique_lock<std::mutex> lock(state_mutex);
      if (!state_condition.wait_for(
          lock, std::chrono::duration<double>(joint_state_timeout),
          [&]() {return state_sequence > 0;}))
      {
        throw std::runtime_error("等待仿真 /joint_states 超时");
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
      trajectory_executor =
        std::make_shared<massage_motion::MoveItTrajectoryExecutor>(node);
    }
    massage_task::MinimalTaskStateMachine task(planner, trajectory_executor);
    massage_task::TaskRequest task_request;
    task_request.task_id = "return_home_sim";
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
        node->get_logger(), "仿真回待机任务失败: %s",
        task_result.message.c_str());
      exit_code = 3;
    }
    else if (!execute)
    {
      RCLCPP_INFO(node->get_logger(), "仿真回待机只规划完成，未发送运动命令");
      exit_code = 0;
    }
    else
    {
      RCLCPP_INFO(
        node->get_logger(), "仿真回待机执行结果: %s",
        task_result.message.c_str());
      sensor_msgs::msg::JointState final_state;
      {
        std::unique_lock<std::mutex> lock(state_mutex);
        const auto previous_sequence = state_sequence;
        if (!state_condition.wait_for(
            lock, std::chrono::duration<double>(joint_state_timeout),
            [&]() {return state_sequence > previous_sequence;}))
        {
          throw std::runtime_error("等待回待机执行后的 /joint_states 超时");
        }
        final_state = latest_state;
      }
      const auto endpoint = massage_motion::calculate_trajectory_endpoint_error(
        task_result.plan_result.trajectory, final_state);
      if (!endpoint.valid || endpoint.max_absolute_error > endpoint_tolerance)
      {
        throw std::runtime_error("回待机终点关节误差超过容差");
      }
      RCLCPP_INFO(
        node->get_logger(), "仿真回待机完成，最大终点误差 %.9f rad",
        endpoint.max_absolute_error);
      exit_code = 0;
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(node->get_logger(), "仿真回待机入口异常: %s", exception.what());
    exit_code = 4;
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
