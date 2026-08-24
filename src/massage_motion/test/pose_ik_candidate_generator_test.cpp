#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "massage_motion/pose_ik_candidate_generator.hpp"

namespace massage_motion
{

namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

JointVariableInfo revolute_variable(
  const std::string & name, double minimum, double maximum)
{
  return {name, true, true, minimum, maximum};
}

}  // namespace

TEST(PoseIkCandidateGeneratorTest, DynamicTimeoutGrowsWithTaskDistance)
{
  IkTimeoutPolicy policy;
  policy.base_timeout = 0.02;
  policy.timeout_per_meter = 0.20;
  policy.timeout_per_radian = 0.05;
  policy.minimum_timeout = 0.02;
  policy.maximum_timeout = 0.50;

  const double near_timeout = calculate_ik_timeout({0.05, 0.10}, 0U, policy);
  const double far_timeout = calculate_ik_timeout({0.80, 1.20}, 0U, policy);

  EXPECT_GT(far_timeout, near_timeout);
  EXPECT_NEAR(near_timeout, 0.035, 1.0e-12);
  EXPECT_NEAR(far_timeout, 0.24, 1.0e-12);
}

TEST(PoseIkCandidateGeneratorTest, TimeoutBackoffAndClampAreBounded)
{
  IkTimeoutPolicy policy;
  policy.base_timeout = 0.01;
  policy.timeout_per_meter = 0.0;
  policy.timeout_per_radian = 0.0;
  policy.minimum_timeout = 0.02;
  policy.maximum_timeout = 0.10;
  policy.failure_backoff_factor = 2.0;

  EXPECT_DOUBLE_EQ(calculate_ik_timeout({0.0, 0.0}, 0U, policy), 0.02);
  EXPECT_DOUBLE_EQ(calculate_ik_timeout({0.0, 0.0}, 2U, policy), 0.08);
  EXPECT_DOUBLE_EQ(calculate_ik_timeout({0.0, 0.0}, 20U, policy), 0.10);
}

TEST(PoseIkCandidateGeneratorTest, RejectsInvalidTimeoutPolicy)
{
  IkTimeoutPolicy policy;
  policy.maximum_timeout = policy.minimum_timeout / 2.0;
  EXPECT_FALSE(valid_ik_timeout_policy(policy));
  EXPECT_TRUE(std::isnan(calculate_ik_timeout({0.1, 0.1}, 0U, policy)));

  policy = IkTimeoutPolicy{};
  policy.failure_backoff_factor = 0.9;
  EXPECT_FALSE(valid_ik_timeout_policy(policy));
}

TEST(PoseIkCandidateGeneratorTest, DeterministicAttemptSeedsAreStableAndDistinct)
{
  EXPECT_EQ(ik_seed_for_attempt(684U, 3U), ik_seed_for_attempt(684U, 3U));
  EXPECT_NE(ik_seed_for_attempt(684U, 3U), ik_seed_for_attempt(684U, 4U));
  EXPECT_NE(ik_seed_for_attempt(684U, 3U), ik_seed_for_attempt(685U, 3U));
}

TEST(PoseIkCandidateGeneratorTest, OrdersJointStateAndRejectsMalformedInput)
{
  sensor_msgs::msg::JointState state;
  state.name = {"joint_2", "unused", "joint_1"};
  state.position = {2.0, 9.0, 1.0};
  const auto ordered = order_joint_state_positions(
    {"joint_1", "joint_2"}, state);
  ASSERT_TRUE(ordered.valid) << ordered.message;
  EXPECT_EQ(ordered.positions, std::vector<double>({1.0, 2.0}));

  state.name = {"joint_1", "joint_1"};
  state.position = {1.0, 2.0};
  EXPECT_FALSE(order_joint_state_positions({"joint_1"}, state).valid);

  state.name = {"joint_1"};
  state.position = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(order_joint_state_positions({"joint_1"}, state).valid);
}

TEST(PoseIkCandidateGeneratorTest, NormalizesEquivalentRevoluteValueNearReference)
{
  const auto variables = std::vector<JointVariableInfo>{
    revolute_variable("joint_1", -6.28, 6.28),
    {"linear", false, true, -10.0, 10.0}};
  const auto normalized = normalize_revolute_positions_near_reference(
    {-3.980851494, 2.0}, {2.302333813, 1.0}, variables);

  ASSERT_EQ(normalized.size(), 2U);
  EXPECT_NEAR(normalized[0], -3.980851494 + kTwoPi, 1.0e-12);
  EXPECT_DOUBLE_EQ(normalized[1], 2.0);
}

TEST(PoseIkCandidateGeneratorTest, DeduplicatesModuloTwoPiButKeepsOtherBranches)
{
  const auto variables = std::vector<JointVariableInfo>{
    revolute_variable("joint_1", -6.28, 6.28),
    revolute_variable("joint_2", -6.28, 6.28)};
  EXPECT_TRUE(equivalent_joint_solution(
      {2.302333813, -0.7},
      {2.302333813 - kTwoPi, -0.7 + kTwoPi},
      variables, 1.0e-6));
  EXPECT_FALSE(equivalent_joint_solution(
      {2.302333813, -0.7},
      {2.302333813, 0.4}, variables, 1.0e-6));
}

TEST(PoseIkCandidateGeneratorTest, ConvertsBusinessTipTargetToNativeSolverTip)
{
  geometry_msgs::msg::Pose business_target;
  business_target.position.x = 0.4;
  business_target.position.y = -0.2;
  business_target.position.z = 0.3;
  business_target.orientation.w = 1.0;
  geometry_msgs::msg::Pose solver_to_business;
  solver_to_business.position.z = 0.08;
  solver_to_business.orientation.w = 1.0;

  const auto solver_target = convert_business_tip_target_to_solver_tip(
    business_target, solver_to_business);

  EXPECT_NEAR(solver_target.position.x, 0.4, 1.0e-12);
  EXPECT_NEAR(solver_target.position.y, -0.2, 1.0e-12);
  EXPECT_NEAR(solver_target.position.z, 0.22, 1.0e-12);
  EXPECT_NEAR(solver_target.orientation.w, 1.0, 1.0e-12);
}

}  // namespace massage_motion
