#include <limits>

#include "gtest/gtest.h"

#include "massage_motion/execution_timing.hpp"

namespace
{

builtin_interfaces::msg::Duration duration(double seconds)
{
  builtin_interfaces::msg::Duration value;
  value.sec = static_cast<std::int32_t>(seconds);
  value.nanosec = static_cast<std::uint32_t>(
    (seconds - static_cast<double>(value.sec)) * 1e9);
  return value;
}

}  // namespace

TEST(ExecutionTimingTest, UsesJointTrajectoryFinalDurationAndMargin)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.time_from_start = duration(1.0);
  trajectory_msgs::msg::JointTrajectoryPoint last;
  last.time_from_start = duration(79.684);
  trajectory.joint_trajectory.points = {first, last};

  massage_motion::ExecutionTimingPolicy policy;
  policy.margin = 15.0;
  const auto result = massage_motion::calculate_execution_timing(trajectory, policy);

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_NEAR(result.expected_duration, 79.684, 1e-6);
  EXPECT_NEAR(result.timeout, 94.684, 1e-6);
  EXPECT_FALSE(result.used_override);
  EXPECT_NE(result.message.find("执行超时判定上限=94.684"), std::string::npos);
}

TEST(ExecutionTimingTest, UsesLargestFinalDurationAcrossTrajectoryTypes)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint joint_point;
  joint_point.time_from_start = duration(2.0);
  trajectory.joint_trajectory.points.push_back(joint_point);
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint multi_dof_point;
  multi_dof_point.time_from_start = duration(3.5);
  trajectory.multi_dof_joint_trajectory.points.push_back(multi_dof_point);

  massage_motion::ExecutionTimingPolicy policy;
  policy.margin = 1.0;
  const auto result = massage_motion::calculate_execution_timing(trajectory, policy);

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_DOUBLE_EQ(result.expected_duration, 3.5);
  EXPECT_DOUBLE_EQ(result.timeout, 4.5);
}

TEST(ExecutionTimingTest, RejectsEmptyAndMalformedTrajectories)
{
  massage_motion::ExecutionTimingPolicy policy;
  moveit_msgs::msg::RobotTrajectory empty;
  EXPECT_FALSE(massage_motion::calculate_execution_timing(empty, policy).valid);

  moveit_msgs::msg::RobotTrajectory malformed;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start.sec = -1;
  malformed.joint_trajectory.points.push_back(point);
  EXPECT_FALSE(massage_motion::calculate_execution_timing(malformed, policy).valid);

  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 1000000000U;
  malformed.joint_trajectory.points = {point};
  EXPECT_FALSE(massage_motion::calculate_execution_timing(malformed, policy).valid);
}

TEST(ExecutionTimingTest, RejectsNonFinitePolicyValues)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = duration(1.0);
  trajectory.joint_trajectory.points.push_back(point);

  massage_motion::ExecutionTimingPolicy policy;
  policy.margin = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(massage_motion::calculate_execution_timing(trajectory, policy).valid);

  policy.margin = 1.0;
  policy.timeout_override = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(massage_motion::calculate_execution_timing(trajectory, policy).valid);
}

TEST(ExecutionTimingTest, RejectsShortOverrideByDefault)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = duration(5.0);
  trajectory.joint_trajectory.points.push_back(point);

  massage_motion::ExecutionTimingPolicy policy;
  policy.timeout_override = 0.05;
  const auto result = massage_motion::calculate_execution_timing(trajectory, policy);

  EXPECT_FALSE(result.valid);
}

TEST(ExecutionTimingTest, AllowsShortOverrideOnlyForFaultInjection)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = duration(5.0);
  trajectory.joint_trajectory.points.push_back(point);

  massage_motion::ExecutionTimingPolicy policy;
  policy.timeout_override = 0.05;
  policy.allow_shorter_timeout_for_testing = true;
  const auto result = massage_motion::calculate_execution_timing(trajectory, policy);

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_DOUBLE_EQ(result.expected_duration, 5.0);
  EXPECT_DOUBLE_EQ(result.timeout, 0.05);
  EXPECT_TRUE(result.used_override);
}
