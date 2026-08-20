#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
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
#include "massage_motion/trajectory_execution_contract.hpp"
#include "massage_task/minimal_task_state_machine.hpp"

namespace
{

bool robot_ready(const jaka_msgs::msg::RobotMsg & state)
{
  return state.motion_state == 0 && state.power_state == 1 &&
         state.servo_state == 1 && state.collision_state == 0;
}

std::string default_output_csv()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    now).count();
  return "/tmp/massage_auto_home_" + std::to_string(milliseconds) + ".csv";
}

void log_target_error(
  const rclcpp::Logger & logger,
  const char * phase,
  const massage_motion::JointTargetErrorResult & result)
{
  for (const auto & error : result.joint_errors)
  {
    RCLCPP_INFO(
      logger,
      "%s %s: target=%.9f rad, actual=%.9f rad, error=%.9f rad",
      phase, error.joint_name.c_str(), error.target_position,
      error.actual_position, error.absolute_error);
  }
}

void log_robot_state(
  const rclcpp::Logger & logger,
  const char * phase,
  const jaka_msgs::msg::RobotMsg & state)
{
  RCLCPP_INFO(
    logger,
    "%s机器人状态: motion=%d, power=%d, servo=%d, collision=%d",
    phase, state.motion_state, state.power_state, state.servo_state,
    state.collision_state);
}

void write_result_csv(
  const std::string & output_csv,
  const char * mode,
  const massage_motion::JointTargetErrorResult & result,
  const jaka_msgs::msg::RobotMsg & robot_state)
{
  std::ofstream stream(output_csv, std::ios::out | std::ios::trunc);
  if (!stream)
  {
    throw std::runtime_error("无法创建自动回待机结果 CSV: " + output_csv);
  }

  stream << "mode,joint,target,actual,absolute_error,max_absolute_error,"
            "motion_state,power_state,servo_state,collision_state\n";
  stream << std::setprecision(12);
  for (const auto & error : result.joint_errors)
  {
    stream << mode << ',' << error.joint_name << ','
           << error.target_position << ',' << error.actual_position << ','
           << error.absolute_error << ',' << result.max_absolute_error << ','
           << robot_state.motion_state << ',' << robot_state.power_state << ','
           << robot_state.servo_state << ',' << robot_state.collision_state
           << '\n';
  }
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
  double maximum_joint_travel = 0.55;
  double endpoint_tolerance = 0.002;
  std::string output_csv;
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
  node->get_parameter_or("maximum_joint_travel", maximum_joint_travel, 0.55);
  node->get_parameter_or("endpoint_tolerance", endpoint_tolerance, 0.002);
  node->get_parameter_or("output_csv", output_csv, std::string{});
  if (output_csv.empty())
  {
    output_csv = default_output_csv();
  }

  if (execute && !parameters_confirmed)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "真机回待机执行需要 execute:=true 和 parameters_confirmed:=true");
    rclcpp::shutdown();
    return 2;
  }
  if (planning_attempts < 1 || planning_attempts > 10 ||
    !std::isfinite(velocity_scale) || velocity_scale <= 0.0 ||
    velocity_scale > 1.0 || !std::isfinite(acceleration_scale) ||
    acceleration_scale <= 0.0 || acceleration_scale > 1.0 ||
    !std::isfinite(planning_timeout) || planning_timeout <= 0.0 ||
    !std::isfinite(execution_timeout_margin) ||
    execution_timeout_margin < 0.0 || !std::isfinite(state_timeout) ||
    state_timeout <= 0.0 || !std::isfinite(readiness_timeout) ||
    readiness_timeout <= 0.0 || !std::isfinite(feedback_timeout) ||
    feedback_timeout <= 0.0 || !std::isfinite(maximum_joint_travel) ||
    maximum_joint_travel <= 0.0 || !std::isfinite(endpoint_tolerance) ||
    endpoint_tolerance <= 0.0 || endpoint_tolerance >= maximum_joint_travel)
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
    sensor_msgs::msg::JointState initial_joint_state;
    jaka_msgs::msg::RobotMsg initial_robot_state;
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

      const auto now = std::chrono::steady_clock::now();
      const auto joint_age =
        std::chrono::duration<double>(now - joint_received_at).count();
      const auto robot_age =
        std::chrono::duration<double>(now - robot_received_at).count();
      if (execute && (joint_age > feedback_timeout || robot_age > feedback_timeout))
      {
        throw std::runtime_error("自动回待机初始关节状态或机器人状态已过期");
      }
      initial_joint_state = latest_joint_state;
      initial_robot_state = latest_robot_state;
    }

    const std::vector<std::string> home_joint_names(
      massage_bringup::kJakaS5JointNames.begin(),
      massage_bringup::kJakaS5JointNames.end());
    const std::vector<double> home_joint_positions(
      massage_bringup::kMassageHomeJointPositions.begin(),
      massage_bringup::kMassageHomeJointPositions.end());
    const auto initial_error = massage_motion::calculate_joint_target_error(
      home_joint_names, home_joint_positions, initial_joint_state);
    if (!initial_error.valid)
    {
      throw std::runtime_error(
              "无法计算自动回待机初始误差: " + initial_error.message);
    }
    log_target_error(node->get_logger(), "启动", initial_error);
    log_robot_state(node->get_logger(), "启动", initial_robot_state);
    RCLCPP_INFO(
      node->get_logger(),
      "启动待机误差: max_error=%.9f rad, tolerance=%.9f rad",
      initial_error.max_absolute_error, endpoint_tolerance);

    const bool already_at_target = execute &&
      massage_motion::joint_target_reached(initial_error, endpoint_tolerance);
    if (already_at_target)
    {
      write_result_csv(
        output_csv, "already_at_target", initial_error, initial_robot_state);
      RCLCPP_INFO(
        node->get_logger(),
        "AUTO HOME ALREADY-AT-TARGET: PASS: max_error=%.9f rad, "
        "tolerance=%.9f rad；未创建规划器，未发送轨迹 Goal，csv=%s",
        initial_error.max_absolute_error, endpoint_tolerance,
        output_csv.c_str());
      exit_code = 0;
    }

    if (!already_at_target)
    {
      if (execute)
      {
        auto parameter_node = std::make_shared<rclcpp::Node>(
          "return_home_driver_parameter_client");
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
          message << "驱动终点参数与自动回待机执行门槛不一致: "
                  << execution_contract.message << "; tolerance="
                  << driver_goal_tolerance << " rad, endpoint_tolerance="
                  << endpoint_tolerance << " rad, driver_margin="
                  << driver_goal_timeout << " s, execution_margin="
                  << execution_timeout_margin << " s";
          throw std::runtime_error(message.str());
        }
        RCLCPP_INFO(
          node->get_logger(),
          "驱动终点参数读回通过: tolerance=%.9f rad, "
          "driver_margin=%.3f s, execution_margin=%.3f s",
          driver_goal_tolerance, driver_goal_timeout,
          execution_timeout_margin);
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
        sensor_msgs::msg::JointState final_joint_state;
        jaka_msgs::msg::RobotMsg final_robot_state;
        std::chrono::steady_clock::time_point final_joint_received_at;
        std::chrono::steady_clock::time_point final_robot_received_at;
        {
          std::unique_lock<std::mutex> lock(state_mutex);
          const auto previous_joint_sequence = joint_sequence;
          const auto previous_robot_sequence = robot_sequence;
          if (!state_condition.wait_for(
              lock, std::chrono::duration<double>(state_timeout),
              [&]() {
                return joint_sequence > previous_joint_sequence &&
                       robot_sequence > previous_robot_sequence;
              }))
          {
            throw std::runtime_error(
                    "等待回待机执行后的关节状态或机器人状态超时");
          }
          final_joint_state = latest_joint_state;
          final_robot_state = latest_robot_state;
          final_joint_received_at = joint_received_at;
          final_robot_received_at = robot_received_at;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto final_joint_age = std::chrono::duration<double>(
          now - final_joint_received_at).count();
        const auto final_robot_age = std::chrono::duration<double>(
          now - final_robot_received_at).count();
        if (final_joint_age > feedback_timeout ||
          final_robot_age > feedback_timeout)
        {
          throw std::runtime_error("回待机完成后的状态反馈已过期");
        }
        if (!robot_ready(final_robot_state))
        {
          throw std::runtime_error("回待机完成后的机器人状态不可执行");
        }

        const auto endpoint = massage_motion::calculate_joint_target_error(
          home_joint_names, home_joint_positions, final_joint_state);
        if (!endpoint.valid)
        {
          throw std::runtime_error("无法计算回待机终点误差: " + endpoint.message);
        }
        log_target_error(node->get_logger(), "终点", endpoint);
        log_robot_state(node->get_logger(), "终点", final_robot_state);
        if (!massage_motion::joint_target_reached(
            endpoint, endpoint_tolerance))
        {
          throw std::runtime_error("回待机终点关节误差超过容差");
        }
        write_result_csv(
          output_csv, "executed", endpoint, final_robot_state);
        RCLCPP_INFO(
          node->get_logger(),
          "AUTO HOME EXECUTION: PASS: max_error=%.9f rad, "
          "tolerance=%.9f rad, csv=%s",
          endpoint.max_absolute_error, endpoint_tolerance,
          output_csv.c_str());
        exit_code = 0;
      }
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
