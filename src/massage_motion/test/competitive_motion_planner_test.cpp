#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "massage_motion/competitive_motion_planner.hpp"

namespace massage_motion
{

namespace
{

moveit_msgs::msg::RobotTrajectory make_trajectory(
  const std::vector<std::vector<double>> & positions,
  double time_step)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2"};
  for (std::size_t index = 0; index < positions.size(); ++index)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions[index];
    const auto nanoseconds = static_cast<std::int64_t>(
      static_cast<double>(index) * time_step * 1e9);
    point.time_from_start.sec = static_cast<std::int32_t>(nanoseconds / 1000000000);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000);
    trajectory.joint_trajectory.points.push_back(point);
  }
  return trajectory;
}

PlanResult successful_plan(
  const std::vector<std::vector<double>> & positions,
  double time_step)
{
  PlanResult result;
  result.success = true;
  result.error = MotionError::kNone;
  result.message = "planned";
  result.trajectory = make_trajectory(positions, time_step);
  return result;
}

class SequencePlanner : public IMotionPlanner
{
public:
  explicit SequencePlanner(std::vector<PlanResult> results)
  : results_(std::move(results))
  {
  }

  PlanResult plan(const MotionRequest & request) override
  {
    request_ids.push_back(request.request_id);
    if (next_ >= results_.size())
    {
      PlanResult result;
      result.success = false;
      result.error = MotionError::kPlanningFailed;
      result.message = "no result";
      return result;
    }
    return results_[next_++];
  }

  std::vector<std::string> request_ids;

private:
  std::vector<PlanResult> results_;
  std::size_t next_{0};
};

}  // namespace

TEST(CompetitiveMotionPlannerTest, CalculatesReusableTrajectoryMetrics)
{
  const auto metrics = calculate_trajectory_metrics(make_trajectory(
      {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, 0.5));

  ASSERT_TRUE(metrics.valid);
  EXPECT_DOUBLE_EQ(metrics.joint_path_length, 2.0);
  EXPECT_DOUBLE_EQ(metrics.duration, 1.0);
  EXPECT_DOUBLE_EQ(metrics.maximum_joint_travel, 1.0);
}

TEST(CompetitiveMotionPlannerTest, SelectsLowestScoreAcrossSourcesAndAttempts)
{
  auto first = std::make_shared<SequencePlanner>(std::vector<PlanResult>{
      successful_plan({{0, 0}, {2, 0}}, 1.0),
      successful_plan({{0, 0}, {1, 0}}, 1.0)});
  auto second = std::make_shared<SequencePlanner>(std::vector<PlanResult>{
      successful_plan({{0, 0}, {0.5, 0}}, 1.0)});
  CompetitiveMotionPlanner planner(
    {{"sdk_a", first, 2}, {"sdk_b", second, 1}},
    PlanCompetitionConfig{1.0, 0.0, 3.0});

  MotionRequest request;
  request.request_id = "target";
  const auto result = planner.plan(request);
  const auto report = planner.last_report();

  ASSERT_TRUE(result.success);
  ASSERT_TRUE(report.success);
  ASSERT_EQ(report.candidates.size(), 3U);
  EXPECT_EQ(report.selected_candidate, 2U);
  EXPECT_EQ(report.candidates[2].source_name, "sdk_b");
  EXPECT_EQ(first->request_ids[0], "target_sdk_a_1");
}

TEST(CompetitiveMotionPlannerTest, IgnoresFailuresAndTravelLimitViolations)
{
  PlanResult failed;
  failed.success = false;
  failed.error = MotionError::kPlanningFailed;
  failed.message = "failed";
  auto source = std::make_shared<SequencePlanner>(std::vector<PlanResult>{
      failed,
      successful_plan({{0, 0}, {2, 0}}, 1.0),
      successful_plan({{0, 0}, {0.4, 0}}, 1.0)});
  CompetitiveMotionPlanner planner(
    {{"sdk", source, 3}},
    PlanCompetitionConfig{1.0, 0.0, 0.5});

  const auto result = planner.plan(MotionRequest{});
  const auto report = planner.last_report();

  ASSERT_TRUE(result.success);
  EXPECT_FALSE(report.candidates[0].accepted);
  EXPECT_FALSE(report.candidates[1].accepted);
  EXPECT_TRUE(report.candidates[2].accepted);
}

TEST(CompetitiveMotionPlannerTest, FailsWhenNoCandidateIsAccepted)
{
  auto source = std::make_shared<SequencePlanner>(std::vector<PlanResult>{
      successful_plan({{0, 0}, {2, 0}}, 1.0)});
  CompetitiveMotionPlanner planner(
    {{"sdk", source, 1}},
    PlanCompetitionConfig{1.0, 0.0, 0.5});

  const auto result = planner.plan(MotionRequest{});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, MotionError::kPlanningFailed);
}

}  // namespace massage_motion
