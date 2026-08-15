#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "massage_motion/relative_joint_target.hpp"

namespace
{

const std::vector<std::string> kExpectedNames{
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};

sensor_msgs::msg::JointState make_state()
{
    sensor_msgs::msg::JointState state;
    state.name = {
        "joint_3", "joint_1", "joint_6",
        "joint_4", "joint_2", "joint_5"};
    state.position = {0.3, 0.1, 0.6, 0.4, 0.2, 0.5};
    return state;
}

TEST(RelativeJointTargetTest, ReordersStateAndAppliesOneDelta)
{
    const auto result = massage_motion::make_relative_joint_target(
        make_state(), kExpectedNames, "joint_2", -0.02, 0.05);

    ASSERT_TRUE(result.validation.valid);
    ASSERT_EQ(result.target.positions.size(), 6U);
    EXPECT_DOUBLE_EQ(result.target.positions[0], 0.1);
    EXPECT_DOUBLE_EQ(result.target.positions[1], 0.18);
    EXPECT_DOUBLE_EQ(result.target.positions[2], 0.3);
    EXPECT_DOUBLE_EQ(result.target.positions[5], 0.6);
}

TEST(RelativeJointTargetTest, RejectsZeroAndOversizedDelta)
{
    EXPECT_FALSE(massage_motion::make_relative_joint_target(
        make_state(), kExpectedNames, "joint_1", 0.0, 0.05)
        .validation.valid);
    EXPECT_FALSE(massage_motion::make_relative_joint_target(
        make_state(), kExpectedNames, "joint_1", 0.06, 0.05)
        .validation.valid);
}

TEST(RelativeJointTargetTest, RejectsMissingAndDuplicateJoint)
{
    auto missing = make_state();
    missing.name.pop_back();
    missing.position.pop_back();
    EXPECT_FALSE(massage_motion::make_relative_joint_target(
        missing, kExpectedNames, "joint_1", 0.01, 0.05)
        .validation.valid);

    auto duplicate = make_state();
    duplicate.name[0] = duplicate.name[1];
    EXPECT_FALSE(massage_motion::make_relative_joint_target(
        duplicate, kExpectedNames, "joint_1", 0.01, 0.05)
        .validation.valid);
}

TEST(RelativeJointTargetTest, RejectsInvalidStateAndJointSelection)
{
    auto invalid = make_state();
    invalid.position[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(massage_motion::make_relative_joint_target(
        invalid, kExpectedNames, "joint_1", 0.01, 0.05)
        .validation.valid);
    EXPECT_FALSE(massage_motion::make_relative_joint_target(
        make_state(), kExpectedNames, "joint_7", 0.01, 0.05)
        .validation.valid);
}

TEST(RelativeJointTargetTest, RejectsOverflowedTarget)
{
    auto state = make_state();
    state.position[1] = std::numeric_limits<double>::max();
    EXPECT_FALSE(massage_motion::make_relative_joint_target(
        state,
        kExpectedNames,
        "joint_1",
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max())
        .validation.valid);
}

}  // namespace
