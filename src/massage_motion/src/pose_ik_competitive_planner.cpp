#include "massage_motion/pose_ik_competitive_planner.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "massage_motion/motion_types.hpp"

namespace massage_motion
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

moveit_msgs::msg::RobotState robot_state_from_joint_state(
  const sensor_msgs::msg::JointState & joint_state)
{
  moveit_msgs::msg::RobotState result;
  result.joint_state = joint_state;
  result.is_diff = true;
  return result;
}

PlanResult failed_result(MotionError error, const std::string & message)
{
  PlanResult result;
  result.error = error;
  result.message = message;
  result.planner_id = "pose_ik_competition";
  return result;
}

}  // namespace

PoseIkCompetitivePlanner::PoseIkCompetitivePlanner(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<IMotionPlanner> planner,
  moveit::core::RobotModelConstPtr robot_model,
  PoseIkCompetitivePlannerConfig config)
: node_(std::move(node)),
  logger_(node_ ? node_->get_logger() : rclcpp::get_logger("pose_ik_competition")),
  planner_(std::move(planner)),
  config_(std::move(config)),
  generator_()
{
  if (!node_ || !planner_ || !robot_model || config_.planning_group.empty() ||
    config_.tip_link.empty() || config_.joint_state_topic.empty() ||
    !finite_positive(config_.joint_state_timeout))
  {
    throw std::invalid_argument("位姿 IK 竞争规划器依赖或配置无效");
  }
  auto generator = std::make_shared<PoseIkCandidateGenerator>(
    std::move(robot_model), config_.planning_group, config_.tip_link, config_.ik);
  generator_ = [generator](
    const sensor_msgs::msg::JointState & state,
    const geometry_msgs::msg::PoseStamped & pose)
    {
      return generator->generate(state, pose);
    };
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_state_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      latest_joint_state_ = *message;
      latest_joint_state_received_ = std::chrono::steady_clock::now();
    });
}

PoseIkCompetitivePlanner::PoseIkCompetitivePlanner(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<IMotionPlanner> planner,
  PoseIkGeneratorFunction generator,
  PoseIkCompetitivePlannerConfig config)
: node_(std::move(node)),
  logger_(node_ ? node_->get_logger() : rclcpp::get_logger("pose_ik_competition")),
  planner_(std::move(planner)),
  config_(std::move(config)),
  generator_(std::move(generator))
{
  if (!node_ || !planner_ || !generator_ || config_.planning_group.empty() ||
    config_.tip_link.empty() || config_.joint_state_topic.empty() ||
    !finite_positive(config_.joint_state_timeout))
  {
    throw std::invalid_argument("位姿 IK 竞争规划器依赖或配置无效");
  }
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_state_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      latest_joint_state_ = *message;
      latest_joint_state_received_ = std::chrono::steady_clock::now();
    });
}

sensor_msgs::msg::JointState PoseIkCompetitivePlanner::fixed_start_joint_state(
  const MotionRequest & request, std::string * error) const
{
  sensor_msgs::msg::JointState state;
  if (request.start_state.has_value())
  {
    state = request.start_state->joint_state;
  }
  else
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    state = latest_joint_state_;
    if (latest_joint_state_received_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_joint_state_received_).count() >
      config_.joint_state_timeout)
    {
      if (error) *error = "没有收到新鲜的 /joint_states";
      return sensor_msgs::msg::JointState();
    }
  }
  if (state.name.empty() || state.name.size() != state.position.size())
  {
    if (error) *error = "起始关节状态为空或名称/位置数量不一致";
    return sensor_msgs::msg::JointState();
  }
  return state;
}

PlanResult PoseIkCompetitivePlanner::plan(const MotionRequest & request)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  const auto * pose_target = std::get_if<PoseTarget>(&request.target);
  if (request.motion_type != MotionType::kPtp || !pose_target)
  {
    return planner_->plan(request);
  }

  std::string state_error;
  const auto start_joint_state = fixed_start_joint_state(request, &state_error);
  if (!state_error.empty())
  {
    return failed_result(
      MotionError::kPlanningFailed,
      "PTP 位姿动态 IK 无法固定起始状态: " + state_error);
  }
  const auto ik_report = generator_(start_joint_state, pose_target->pose);
  RCLCPP_INFO(
    logger_,
    "PTP 动态 IK: success=%s, attempts=%zu, unique=%zu, "
    "translation=%.6f m, rotation=%.6f rad, tip=%s, solver_tip=%s",
    ik_report.success ? "true" : "false", ik_report.attempts.size(),
    ik_report.candidates.size(), ik_report.target_distance.translation,
    ik_report.target_distance.rotation, ik_report.requested_tip_link.c_str(),
    ik_report.solver_tip_link.c_str());
  if (!ik_report.success)
  {
    return failed_result(
      MotionError::kPlanningFailed,
      "PTP 位姿动态 IK 失败: " + ik_report.message);
  }

  PlanResult best;
  double best_score = std::numeric_limits<double>::infinity();
  std::size_t accepted = 0U;
  std::size_t selected = std::numeric_limits<std::size_t>::max();
  const auto fixed_start = robot_state_from_joint_state(start_joint_state);
  for (std::size_t index = 0U; index < ik_report.candidates.size(); ++index)
  {
    MotionRequest candidate_request = request;
    candidate_request.request_id = request.request_id + "_ik_" +
      std::to_string(index + 1U);
    candidate_request.target = JointTarget{ik_report.candidates[index].positions};
    candidate_request.start_state = fixed_start;
    const auto candidate = planner_->plan(candidate_request);
    const auto metrics = calculate_trajectory_metrics(candidate.trajectory);
    const bool within_limit = candidate.success && metrics.valid &&
      metrics.maximum_joint_travel <= config_.competition.maximum_joint_travel;
    const double score = within_limit ?
      config_.competition.joint_path_length_weight * metrics.joint_path_length +
      config_.competition.trajectory_duration_weight * metrics.duration :
      std::numeric_limits<double>::infinity();
    std::string candidate_message = candidate.message;
    if (!candidate.success)
    {
      candidate_message =
        "该 IK 解的内部规划尝试均失败，仅淘汰此 IK 候选";
    }
    else if (!metrics.valid)
    {
      candidate_message = "该 IK 解的轨迹指标无效，仅淘汰此 IK 候选";
    }
    else if (!within_limit)
    {
      candidate_message =
        "该 IK 解超过单关节行程限制，仅淘汰此 IK 候选";
    }
    RCLCPP_INFO(
      logger_,
      "PTP IK 单候选[%zu]: source_attempt=%zu, planned=%s, accepted=%s, "
      "score=%.6f, path=%.6f rad, duration=%.3f s, max_joint=%.6f rad, message=%s",
      index, ik_report.candidates[index].source_attempt,
      candidate.success ? "true" : "false", within_limit ? "true" : "false",
      score, metrics.joint_path_length, metrics.duration,
      metrics.maximum_joint_travel, candidate_message.c_str());
    if (within_limit)
    {
      ++accepted;
      if (score < best_score)
      {
        best = candidate;
        best_score = score;
        selected = index;
      }
    }
  }
  if (selected == std::numeric_limits<std::size_t>::max())
  {
    RCLCPP_ERROR(
      logger_,
      "PTP IK 竞争汇总: FAIL: accepted=0/%zu；本次 PTP 不得执行",
      ik_report.candidates.size());
    return failed_result(
      MotionError::kPlanningFailed,
      "PTP 位姿 IK 候选均未生成可接受轨迹");
  }
  std::ostringstream message;
  message << "PTP 位姿 IK 竞争规划成功: selected=" << selected
          << ", accepted=" << accepted << "/" << ik_report.candidates.size()
          << ", score=" << best_score;
  best.message = message.str();
  best.planner_id = "pose_ik_competition";
  RCLCPP_INFO(
    logger_,
    "PTP IK 竞争汇总: PASS: selected=%zu, accepted=%zu/%zu, "
    "score=%.6f；只有此汇总为 PASS 才允许执行",
    selected, accepted, ik_report.candidates.size(), best_score);
  return best;
}

}  // namespace massage_motion
