#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "massage_motion/pose_ik_competitive_planner.hpp"

namespace
{

class RecordingPlanner final : public massage_motion::IMotionPlanner
{
public:
  massage_motion::PlanResult plan(
    const massage_motion::MotionRequest & request) override
  {
    requests.push_back(request);
    massage_motion::PlanResult result;
    result.success = true;
    result.error = massage_motion::MotionError::kNone;
    result.message = "planned";
    result.trajectory.joint_trajectory.joint_names = {"joint_1"};
    trajectory_msgs::msg::JointTrajectoryPoint start;
    start.positions = {0.0};
    result.trajectory.joint_trajectory.points.push_back(start);
    trajectory_msgs::msg::JointTrajectoryPoint goal;
    const auto * target = std::get_if<massage_motion::JointTarget>(&request.target);
    goal.positions = target && !target->positions.empty() ?
      target->positions : std::vector<double>{0.0};
    goal.time_from_start.sec = goal.positions.front() < 0.5 ? 1 : 2;
    result.trajectory.joint_trajectory.points.push_back(goal);
    return result;
  }

  std::vector<massage_motion::MotionRequest> requests;
};

class PoseIkCompetitivePlannerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  massage_motion::PoseIkCompetitivePlannerConfig config() const
  {
    massage_motion::PoseIkCompetitivePlannerConfig result;
    result.planning_group = "test_group";
    result.tip_link = "test_tip";
    result.joint_state_topic = "/pose_ik_competition_test/joint_states";
    result.joint_state_timeout = 0.1;
    result.competition.maximum_joint_travel = 1.0;
    return result;
  }

  geometry_msgs::msg::PoseStamped pose() const
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "world";
    result.pose.orientation.w = 1.0;
    return result;
  }
};

TEST_F(PoseIkCompetitivePlannerTest, ForwardsNonPtpPoseRequestUnchanged)
{
  auto node = std::make_shared<rclcpp::Node>("pose_ik_route_test");
  auto backend = std::make_shared<RecordingPlanner>();
  int generator_calls = 0;
  massage_motion::PoseIkCompetitivePlanner planner(
    node, backend,
    [&](const auto &, const auto &)
    {
      ++generator_calls;
      return massage_motion::PoseIkCandidateReport{};
    },
    config());
  massage_motion::MotionRequest request;
  request.request_id = "lin";
  request.motion_type = massage_motion::MotionType::kLin;
  request.target = massage_motion::PoseTarget{pose()};

  const auto result = planner.plan(request);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(generator_calls, 0);
  ASSERT_EQ(backend->requests.size(), 1U);
  EXPECT_EQ(backend->requests.front().request_id, "lin");
  EXPECT_EQ(backend->requests.front().motion_type, massage_motion::MotionType::kLin);
}

TEST_F(PoseIkCompetitivePlannerTest, RejectsPtpPoseWithoutFreshStartState)
{
  auto node = std::make_shared<rclcpp::Node>("pose_ik_stale_state_test");
  auto backend = std::make_shared<RecordingPlanner>();
  int generator_calls = 0;
  massage_motion::PoseIkCompetitivePlanner planner(
    node, backend,
    [&](const auto &, const auto &)
    {
      ++generator_calls;
      return massage_motion::PoseIkCandidateReport{};
    },
    config());
  massage_motion::MotionRequest request;
  request.request_id = "ptp";
  request.motion_type = massage_motion::MotionType::kPtp;
  request.target = massage_motion::PoseTarget{pose()};

  const auto result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, massage_motion::MotionError::kPlanningFailed);
  EXPECT_NE(result.message.find("没有收到新鲜"), std::string::npos);
  EXPECT_EQ(generator_calls, 0);
  EXPECT_TRUE(backend->requests.empty());
}

TEST_F(PoseIkCompetitivePlannerTest, SelectsLowestScoringAcceptedIkTrajectory)
{
  auto node = std::make_shared<rclcpp::Node>("pose_ik_selection_test");
  auto backend = std::make_shared<RecordingPlanner>();
  massage_motion::PoseIkCompetitivePlanner planner(
    node, backend,
    [](const auto &, const auto &)
    {
      massage_motion::PoseIkCandidateReport report;
      report.success = true;
      report.requested_tip_link = "test_tip";
      report.solver_tip_link = "test_tip";
      report.candidates = {
        {1U, 0.02, {0.8}},
        {2U, 0.02, {0.2}}};
      return report;
    },
    config());
  massage_motion::MotionRequest request;
  request.request_id = "ptp";
  request.motion_type = massage_motion::MotionType::kPtp;
  request.target = massage_motion::PoseTarget{pose()};
  moveit_msgs::msg::RobotState start;
  start.joint_state.name = {"joint_1"};
  start.joint_state.position = {0.0};
  request.start_state = start;

  const auto result = planner.plan(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.planner_id, "pose_ik_competition");
  ASSERT_EQ(backend->requests.size(), 2U);
  EXPECT_TRUE(backend->requests[0].start_state.has_value());
  EXPECT_TRUE(backend->requests[1].start_state.has_value());
  ASSERT_EQ(result.trajectory.joint_trajectory.points.size(), 2U);
  EXPECT_DOUBLE_EQ(
    result.trajectory.joint_trajectory.points.back().positions.front(), 0.2);
  EXPECT_NE(result.message.find("selected=1"), std::string::npos);
}

}  // namespace
