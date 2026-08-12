#include "gtest/gtest.h"

#include "massage_motion/trajectory_endpoint_error.hpp"

namespace massage_motion
{
namespace
{

moveit_msgs::msg::RobotTrajectory make_trajectory()
{
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2"};

    trajectory_msgs::msg::JointTrajectoryPoint endpoint;
    endpoint.positions = {1.0, -0.5};
    trajectory.joint_trajectory.points.push_back(endpoint);
    return trajectory;
}

TEST(TrajectoryEndpointErrorTest, MatchesByNameAndAllowsAdditionalActualJoints)
{
    const auto trajectory = make_trajectory();

    sensor_msgs::msg::JointState actual_state;
    actual_state.name = {"extra_joint", "joint_2", "joint_1"};
    actual_state.position = {2.0, -0.52, 1.03};

    const auto result =
        calculate_trajectory_endpoint_error(trajectory, actual_state);

    ASSERT_TRUE(result.valid) << result.message;
    ASSERT_EQ(result.joint_errors.size(), 2U);

    EXPECT_EQ(result.joint_errors[0].joint_name, "joint_1");
    EXPECT_NEAR(result.joint_errors[0].target_position, 1.0, 1e-12);
    EXPECT_NEAR(result.joint_errors[0].actual_position, 1.03, 1e-12);
    EXPECT_NEAR(result.joint_errors[0].absolute_error, 0.03, 1e-12);

    EXPECT_EQ(result.joint_errors[1].joint_name, "joint_2");
    EXPECT_NEAR(result.joint_errors[1].absolute_error, 0.02, 1e-12);
    EXPECT_NEAR(result.max_absolute_error, 0.03, 1e-12);
}

TEST(TrajectoryEndpointErrorTest, RejectsEmptyTrajectory)
{
    const moveit_msgs::msg::RobotTrajectory trajectory;
    const sensor_msgs::msg::JointState actual_state;

    const auto result =
        calculate_trajectory_endpoint_error(trajectory, actual_state);

    EXPECT_FALSE(result.valid);
}

TEST(TrajectoryEndpointErrorTest, RejectsMismatchedPlannedArrays)
{
    auto trajectory = make_trajectory();
    trajectory.joint_trajectory.points.back().positions.pop_back();

    sensor_msgs::msg::JointState actual_state;
    actual_state.name = {"joint_1", "joint_2"};
    actual_state.position = {1.0, -0.5};

    const auto result =
        calculate_trajectory_endpoint_error(trajectory, actual_state);

    EXPECT_FALSE(result.valid);
}

TEST(TrajectoryEndpointErrorTest, RejectsMismatchedActualArrays)
{
    const auto trajectory = make_trajectory();

    sensor_msgs::msg::JointState actual_state;
    actual_state.name = {"joint_1", "joint_2"};
    actual_state.position = {1.0};

    const auto result =
        calculate_trajectory_endpoint_error(trajectory, actual_state);

    EXPECT_FALSE(result.valid);
}

TEST(TrajectoryEndpointErrorTest, RejectsMissingPlannedJoint)
{
    const auto trajectory = make_trajectory();

    sensor_msgs::msg::JointState actual_state;
    actual_state.name = {"joint_1", "unrelated_joint"};
    actual_state.position = {1.0, -0.5};

    const auto result =
        calculate_trajectory_endpoint_error(trajectory, actual_state);

    EXPECT_FALSE(result.valid);
}

}  // namespace
}  // namespace massage_motion
