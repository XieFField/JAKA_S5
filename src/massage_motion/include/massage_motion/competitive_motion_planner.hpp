#ifndef MASSAGE_MOTION__COMPETITIVE_MOTION_PLANNER_HPP_
#define MASSAGE_MOTION__COMPETITIVE_MOTION_PLANNER_HPP_

#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "massage_motion/motion_planner.hpp"

namespace massage_motion
{

struct PlanningSource
{
  std::string name;
  std::shared_ptr<IMotionPlanner> planner;
  std::size_t attempts{1};
};

struct PlanCompetitionConfig
{
  double joint_path_length_weight{1.0};
  double trajectory_duration_weight{0.05};
  double maximum_joint_travel{std::numeric_limits<double>::infinity()};
};

struct JointTravelMetric
{
  std::string joint_name;
  double start_position{0.0};
  double goal_position{0.0};
  double signed_travel{0.0};
  double absolute_travel{0.0};
};

struct TrajectoryMetrics
{
  bool valid{false};
  std::string message;
  double joint_path_length{0.0};
  double duration{0.0};
  double maximum_joint_travel{0.0};
  std::string maximum_joint_travel_name;
  std::vector<JointTravelMetric> joint_travels;
};

struct PlanCandidateReport
{
  std::string source_name;
  std::size_t attempt{0};
  bool planning_success{false};
  bool accepted{false};
  double score{std::numeric_limits<double>::infinity()};
  TrajectoryMetrics metrics;
  std::string message;
};

struct PlanCompetitionReport
{
  bool success{false};
  std::size_t selected_candidate{std::numeric_limits<std::size_t>::max()};
  std::vector<PlanCandidateReport> candidates;
};

TrajectoryMetrics calculate_trajectory_metrics(
  const moveit_msgs::msg::RobotTrajectory & trajectory);

class CompetitiveMotionPlanner final : public IMotionPlanner
{
public:
  CompetitiveMotionPlanner(
    std::vector<PlanningSource> sources,
    PlanCompetitionConfig config = {});

  PlanResult plan(const MotionRequest & request) override;
  PlanCompetitionReport last_report() const;

private:
  std::vector<PlanningSource> sources_;
  PlanCompetitionConfig config_;
  mutable std::mutex report_mutex_;
  std::mutex plan_mutex_;
  PlanCompetitionReport last_report_;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__COMPETITIVE_MOTION_PLANNER_HPP_
