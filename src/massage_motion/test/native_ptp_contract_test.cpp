#include <vector>

#include "gtest/gtest.h"

#include "massage_motion/native_ptp_contract.hpp"

namespace
{

const std::vector<std::string> kNames{
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

trajectory_msgs::msg::JointTrajectoryPoint point(
  const std::vector<double> & positions, double time)
{
  trajectory_msgs::msg::JointTrajectoryPoint result;
  result.positions = positions;
  result.time_from_start.sec = static_cast<std::int32_t>(time);
  result.time_from_start.nanosec = static_cast<std::uint32_t>(
    (time - static_cast<double>(result.time_from_start.sec)) * 1e9);
  return result;
}

moveit_msgs::msg::RobotTrajectory straight_trajectory()
{
  moveit_msgs::msg::RobotTrajectory result;
  result.joint_trajectory.joint_names = kNames;
  result.joint_trajectory.points = {
    point({0, 0, 0, 0, 0, 0}, 0.0),
    point({0.05, -0.025, 0, 0, 0, 0}, 1.0),
    point({0.10, -0.05, 0, 0, 0, 0}, 2.0)};
  result.joint_trajectory.points[0].velocities = {0, 0, 0, 0, 0, 0};
  result.joint_trajectory.points[0].accelerations = {0, 0, 0, 0, 0, 0};
  result.joint_trajectory.points[1].velocities = {0.05, -0.025, 0, 0, 0, 0};
  result.joint_trajectory.points[1].accelerations = {0.10, -0.05, 0, 0, 0, 0};
  result.joint_trajectory.points[2].velocities = {0, 0, 0, 0, 0, 0};
  result.joint_trajectory.points[2].accelerations = {0, 0, 0, 0, 0, 0};
  return result;
}

sensor_msgs::msg::JointState current_state(double joint_1 = 0.0)
{
  sensor_msgs::msg::JointState result;
  result.name = kNames;
  result.position = {joint_1, 0, 0, 0, 0, 0};
  return result;
}

TEST(NativePtpContractTest, AcceptsStraightMonotonicJointPath)
{
  const auto result = massage_motion::make_native_ptp_command(
    straight_trajectory(), current_state(), 10.0);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_EQ(result.target_positions.size(), 6U);
  EXPECT_NEAR(result.speed, 0.05, 1e-9);
  EXPECT_LE(result.acceleration, 0.50);
  EXPECT_GE(result.effective_timeout, 15.0);
}

TEST(NativePtpContractTest, RejectsStaleStartAndBentPath)
{
  auto start_mismatch = massage_motion::make_native_ptp_command(
    straight_trajectory(), current_state(0.01), 10.0);
  EXPECT_FALSE(start_mismatch.valid);

  auto bent = straight_trajectory();
  bent.joint_trajectory.points[1].positions[1] = 0.02;
  const auto bent_result = massage_motion::make_native_ptp_command(
    bent, current_state(), 10.0);
  EXPECT_FALSE(bent_result.valid);
  EXPECT_NE(bent_result.message.find("等价"), std::string::npos);
}

TEST(NativePtpContractTest, RejectsReversal)
{
  auto reversed = straight_trajectory();
  reversed.joint_trajectory.points.insert(
    reversed.joint_trajectory.points.begin() + 2,
    point({0.02, -0.01, 0, 0, 0, 0}, 1.5));
  const auto result = massage_motion::make_native_ptp_command(
    reversed, current_state(), 10.0);
  EXPECT_FALSE(result.valid);
}

TEST(NativePtpContractTest, AcceptsAlreadyAtTargetWithoutMotionProfile)
{
  auto trajectory = straight_trajectory();
  trajectory.joint_trajectory.points.resize(2U);
  trajectory.joint_trajectory.points[1].positions = {
    0.001, 0, 0, 0, 0, 0};
  trajectory.joint_trajectory.points[1].time_from_start.sec = 1;
  trajectory.joint_trajectory.points[1].velocities.clear();
  trajectory.joint_trajectory.points[1].accelerations.clear();
  const auto result = massage_motion::make_native_ptp_command(
    trajectory, current_state(), 10.0);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.already_at_target);
  EXPECT_DOUBLE_EQ(result.speed, 0.0);
}

}  // namespace
