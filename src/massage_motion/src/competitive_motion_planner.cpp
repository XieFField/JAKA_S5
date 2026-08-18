#include "massage_motion/competitive_motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace massage_motion
{

namespace
{

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

bool valid_config(const PlanCompetitionConfig & config)
{
  const bool weights_valid =
    std::isfinite(config.joint_path_length_weight) &&
    std::isfinite(config.trajectory_duration_weight) &&
    config.joint_path_length_weight >= 0.0 &&
    config.trajectory_duration_weight >= 0.0 &&
    (config.joint_path_length_weight > 0.0 ||
    config.trajectory_duration_weight > 0.0);
  const bool travel_valid =
    (std::isfinite(config.maximum_joint_travel) &&
    config.maximum_joint_travel > 0.0) ||
    (std::isinf(config.maximum_joint_travel) &&
    config.maximum_joint_travel > 0.0);
  return weights_valid && travel_valid;
}

}  // namespace

TrajectoryMetrics calculate_trajectory_metrics(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty())
  {
    return {false, "轨迹缺少关节名称或轨迹点", 0.0, 0.0, 0.0};
  }

  const std::size_t joint_count = joint_trajectory.joint_names.size();
  double path_length = 0.0;
  double previous_time = -1.0;

  for (std::size_t point_index = 0;
    point_index < joint_trajectory.points.size(); ++point_index)
  {
    const auto & point = joint_trajectory.points[point_index];
    if (point.positions.size() != joint_count)
    {
      return {false, "轨迹点位置数量与关节数量不匹配", 0.0, 0.0, 0.0};
    }

    const double current_time = duration_seconds(point.time_from_start);
    if (!std::isfinite(current_time) || current_time < 0.0 ||
      current_time < previous_time)
    {
      return {false, "轨迹时间不是有限单调序列", 0.0, 0.0, 0.0};
    }

    for (const double position : point.positions)
    {
      if (!std::isfinite(position))
      {
        return {false, "轨迹包含非有限关节位置", 0.0, 0.0, 0.0};
      }
    }

    if (point_index > 0)
    {
      const auto & previous = joint_trajectory.points[point_index - 1];
      double squared_segment_length = 0.0;
      for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index)
      {
        const double delta = point.positions[joint_index] -
          previous.positions[joint_index];
        squared_segment_length += delta * delta;
      }
      path_length += std::sqrt(squared_segment_length);
    }
    previous_time = current_time;
  }

  const auto & start = joint_trajectory.points.front().positions;
  const auto & goal = joint_trajectory.points.back().positions;
  double maximum_joint_travel = 0.0;
  for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index)
  {
    maximum_joint_travel = std::max(
      maximum_joint_travel,
      std::abs(goal[joint_index] - start[joint_index]));
  }

  return {
    true,
    "轨迹指标计算成功",
    path_length,
    duration_seconds(joint_trajectory.points.back().time_from_start),
    maximum_joint_travel};
}

CompetitiveMotionPlanner::CompetitiveMotionPlanner(
  std::vector<PlanningSource> sources,
  PlanCompetitionConfig config)
: sources_(std::move(sources)), config_(config)
{
}

PlanResult CompetitiveMotionPlanner::plan(const MotionRequest & request)
{
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  PlanCompetitionReport report;

  if (sources_.empty() || !valid_config(config_))
  {
    PlanResult invalid_result;
    invalid_result.success = false;
    invalid_result.error = MotionError::kInvalidRequest;
    invalid_result.message = "竞争规划器没有有效规划源或评分配置无效";
    invalid_result.planner_id = "competition";
    return invalid_result;
  }

  PlanResult best_result;
  double best_score = std::numeric_limits<double>::infinity();

  for (const auto & source : sources_)
  {
    if (!source.planner || source.name.empty() || source.attempts == 0)
    {
      PlanCandidateReport candidate;
      candidate.source_name = source.name;
      candidate.message = "规划源名称、实例或尝试次数无效";
      report.candidates.push_back(std::move(candidate));
      continue;
    }

    for (std::size_t attempt = 0; attempt < source.attempts; ++attempt)
    {
      MotionRequest candidate_request = request;
      candidate_request.request_id = request.request_id + "_" + source.name +
        "_" + std::to_string(attempt + 1);
      const auto candidate_result = source.planner->plan(candidate_request);

      PlanCandidateReport candidate;
      candidate.source_name = source.name;
      candidate.attempt = attempt + 1;
      candidate.planning_success = candidate_result.success;
      candidate.message = candidate_result.message;

      if (candidate_result.success)
      {
        candidate.metrics = calculate_trajectory_metrics(candidate_result.trajectory);
        if (candidate.metrics.valid &&
          candidate.metrics.maximum_joint_travel <= config_.maximum_joint_travel)
        {
          candidate.accepted = true;
          candidate.score =
            config_.joint_path_length_weight * candidate.metrics.joint_path_length +
            config_.trajectory_duration_weight * candidate.metrics.duration;
          if (candidate.score < best_score)
          {
            best_score = candidate.score;
            best_result = candidate_result;
            report.selected_candidate = report.candidates.size();
          }
        }
        else if (candidate.metrics.valid)
        {
          candidate.message = "候选轨迹超过最大单关节行程";
        }
        else
        {
          candidate.message = candidate.metrics.message;
        }
      }
      report.candidates.push_back(std::move(candidate));
    }
  }

  report.success = report.selected_candidate !=
    std::numeric_limits<std::size_t>::max();
  if (report.success)
  {
    const auto & selected = report.candidates[report.selected_candidate];
    std::ostringstream message;
    message << "竞争规划选择 " << selected.source_name << " 第 "
            << selected.attempt << " 次候选，score=" << selected.score;
    best_result.message = message.str();
  }
  else
  {
    best_result.success = false;
    best_result.error = MotionError::kPlanningFailed;
    best_result.message = "所有竞争规划候选均失败或未通过约束";
    best_result.planner_id = "competition";
  }

  {
    std::lock_guard<std::mutex> report_lock(report_mutex_);
    last_report_ = report;
  }
  return best_result;
}

PlanCompetitionReport CompetitiveMotionPlanner::last_report() const
{
  std::lock_guard<std::mutex> lock(report_mutex_);
  return last_report_;
}

}  // namespace massage_motion
