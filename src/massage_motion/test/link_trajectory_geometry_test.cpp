#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "srdfdom/model.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "urdf_parser/urdf_parser.h"

#include "massage_motion/link_trajectory_geometry.hpp"
#include "massage_motion/cartesian_path_verification.hpp"

namespace massage_motion
{

namespace
{

moveit::core::RobotModelPtr make_test_robot_model()
{
  static const std::string urdf_xml = R"(
<robot name="height_test_robot">
  <link name="world"/>
  <link name="Link_03"/>
  <joint name="joint_z" type="prismatic">
    <parent link="world"/>
    <child link="Link_03"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="1.0" velocity="1.0"/>
  </joint>
</robot>)";
  static const std::string srdf_xml = R"(
<robot name="height_test_robot">
  <group name="test_group">
    <joint name="joint_z"/>
  </group>
</robot>)";
  const auto urdf_model = urdf::parseURDF(urdf_xml);
  if (!urdf_model)
  {
    return nullptr;
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, srdf_xml))
  {
    return nullptr;
  }
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

moveit_msgs::msg::RobotTrajectory make_height_trajectory(
  const std::vector<double> & heights)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"joint_z"};
  for (std::size_t index = 0; index < heights.size(); ++index)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {heights[index]};
    point.time_from_start.sec = static_cast<std::int32_t>(index);
    trajectory.joint_trajectory.points.push_back(point);
  }
  return trajectory;
}

}  // namespace

TEST(LinkTrajectoryGeometryTest, CalculatesWholeTrajectoryHeightMetrics)
{
  const auto metrics = calculate_link_height_metrics(
    make_test_robot_model(), make_height_trajectory({0.12, 0.30, 0.08, 0.20}),
    "Link_03");

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_NEAR(metrics.start_z, 0.12, 1.0e-12);
  EXPECT_NEAR(metrics.end_z, 0.20, 1.0e-12);
  EXPECT_NEAR(metrics.minimum_z, 0.08, 1.0e-12);
  EXPECT_NEAR(metrics.maximum_z, 0.30, 1.0e-12);
  EXPECT_NEAR(metrics.maximum_drop_below_start, 0.04, 1.0e-12);
  EXPECT_EQ(metrics.minimum_point_index, 2U);
  EXPECT_DOUBLE_EQ(metrics.minimum_point_time, 2.0);
}

TEST(LinkTrajectoryGeometryTest, AcceptsHighLinkTrajectory)
{
  LinkHeightMetrics metrics;
  metrics.valid = true;
  metrics.start_z = 0.55;
  metrics.end_z = 0.48;
  metrics.minimum_z = 0.39;
  metrics.maximum_z = 0.55;
  metrics.maximum_drop_below_start = 0.16;

  const auto result = evaluate_link_height_gate(
    metrics, LinkHeightGateConfig{true, 0.0, 0.55, 0.20});

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.minimum_allowed_z, 0.35, 1.0e-12);
  EXPECT_NEAR(result.drop_below_reference, 0.16, 1.0e-12);
}

TEST(LinkTrajectoryGeometryTest, RejectsTrajectoryBelowWorldFloor)
{
  LinkHeightMetrics metrics;
  metrics.valid = true;
  metrics.start_z = 0.12;
  metrics.end_z = -0.15;
  metrics.minimum_z = -0.15;
  metrics.maximum_z = 0.12;
  metrics.maximum_drop_below_start = 0.27;

  const auto result = evaluate_link_height_gate(
    metrics, LinkHeightGateConfig{true, 0.0, 0.55, 0.80});

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.message.find("低于阈值"), std::string::npos);
}

TEST(LinkTrajectoryGeometryTest, RejectsIntermediateDropEvenWhenEndpointRecovers)
{
  const auto metrics = calculate_link_height_metrics(
    make_test_robot_model(), make_height_trajectory({0.20, 0.13, 0.25}),
    "Link_03");
  ASSERT_TRUE(metrics.valid) << metrics.message;

  const auto result = evaluate_link_height_gate(
    metrics, LinkHeightGateConfig{true, 0.0, 0.30, 0.15});

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.message.find("超过阈值"), std::string::npos);
}

TEST(LinkTrajectoryGeometryTest, UsesStandbyReferenceInsteadOfTrajectoryStart)
{
  LinkHeightMetrics metrics;
  metrics.valid = true;
  metrics.start_z = 0.80;
  metrics.end_z = 0.39;
  metrics.minimum_z = 0.39;
  metrics.maximum_z = 0.80;
  metrics.maximum_drop_below_start = 0.41;

  const auto result = evaluate_link_height_gate(
    metrics, LinkHeightGateConfig{true, 0.0, 0.55, 0.20});

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.accepted) << result.message;
  EXPECT_NEAR(result.drop_below_reference, 0.16, 1.0e-12);
  EXPECT_NEAR(result.minimum_allowed_z, 0.35, 1.0e-12);
}

TEST(LinkTrajectoryGeometryTest, RejectsDropBelowStandbyReferenceLimit)
{
  LinkHeightMetrics metrics;
  metrics.valid = true;
  metrics.start_z = 0.40;
  metrics.end_z = 0.34;
  metrics.minimum_z = 0.34;
  metrics.maximum_z = 0.40;
  metrics.maximum_drop_below_start = 0.06;

  const auto result = evaluate_link_height_gate(
    metrics, LinkHeightGateConfig{true, 0.0, 0.55, 0.20});

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.accepted);
  EXPECT_NEAR(result.drop_below_reference, 0.21, 1.0e-12);
  EXPECT_NE(result.message.find("drop_below_reference"), std::string::npos);
}

TEST(LinkTrajectoryGeometryTest, RejectsEmptyTrajectoryAndUnknownNames)
{
  const auto model = make_test_robot_model();
  ASSERT_TRUE(model);

  EXPECT_FALSE(calculate_link_height_metrics(
      model, moveit_msgs::msg::RobotTrajectory{}, "Link_03").valid);
  EXPECT_FALSE(calculate_link_height_metrics(
      model, make_height_trajectory({0.1}), "unknown_link").valid);

  auto unknown_joint = make_height_trajectory({0.1});
  unknown_joint.joint_trajectory.joint_names = {"unknown_joint"};
  const auto metrics = calculate_link_height_metrics(
    model, unknown_joint, "Link_03");
  EXPECT_FALSE(metrics.valid);
  EXPECT_NE(metrics.message.find("unknown_joint"), std::string::npos);
}

TEST(LinkTrajectoryGeometryTest, RejectsNonFiniteJointInput)
{
  const auto metrics = calculate_link_height_metrics(
    make_test_robot_model(),
    make_height_trajectory({std::numeric_limits<double>::quiet_NaN()}),
    "Link_03");

  EXPECT_FALSE(metrics.valid);
  EXPECT_NE(metrics.message.find("非有限"), std::string::npos);
}

TEST(LinkTrajectoryGeometryTest, DisabledGateKeepsExistingSelectionBehavior)
{
  LinkHeightMetrics invalid_metrics;
  invalid_metrics.message = "没有 FK 证据";

  const auto result = evaluate_link_height_gate(
    invalid_metrics, LinkHeightGateConfig{false, 0.0, 0.55, 0.20});

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.accepted);
  EXPECT_NE(result.message.find("已关闭"), std::string::npos);
}

TEST(LinkTrajectoryGeometryTest, RejectsInvalidGateConfiguration)
{
  LinkHeightMetrics metrics;
  metrics.valid = true;
  const auto negative_limit = evaluate_link_height_gate(
    metrics, LinkHeightGateConfig{true, 0.0, 0.55, -0.01});
  const auto missing_reference = evaluate_link_height_gate(
    metrics, LinkHeightGateConfig{});

  EXPECT_FALSE(negative_limit.valid);
  EXPECT_FALSE(negative_limit.accepted);
  EXPECT_FALSE(missing_reference.valid);
  EXPECT_FALSE(missing_reference.accepted);
}

TEST(LinkTrajectoryGeometryTest, CalculatesStandbyReferenceHeight)
{
  const auto result = calculate_link_height_at_joint_target(
    make_test_robot_model(), {"joint_z"}, {0.55}, "Link_03");

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_NEAR(result.z, 0.55, 1.0e-12);
}

TEST(LinkTrajectoryGeometryTest, RejectsMalformedStandbyReferenceTarget)
{
  const auto model = make_test_robot_model();
  EXPECT_FALSE(calculate_link_height_at_joint_target(
      model, {}, {}, "Link_03").valid);
  EXPECT_FALSE(calculate_link_height_at_joint_target(
      model, {"joint_z"}, {}, "Link_03").valid);
  EXPECT_FALSE(calculate_link_height_at_joint_target(
      model, {"other_joint"}, {0.5}, "Link_03").valid);
  EXPECT_FALSE(calculate_link_height_at_joint_target(
      model, {"joint_z"}, {std::numeric_limits<double>::quiet_NaN()},
      "Link_03").valid);
}

TEST(CartesianLinkPoseTest, CalculatesPoseFromCompleteJointState)
{
  sensor_msgs::msg::JointState state;
  state.name = {"joint_z"};
  state.position = {0.25};

  const auto result = calculate_link_pose(
    make_test_robot_model(), state, "Link_03");

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_NEAR(result.pose.position.z, 0.25, 1.0e-12);
}

TEST(CartesianLinkPoseTest, RejectsMissingModelVariable)
{
  sensor_msgs::msg::JointState state;
  state.name = {"unrelated_joint"};
  state.position = {0.25};

  const auto result = calculate_link_pose(
    make_test_robot_model(), state, "Link_03");

  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.message.find("joint_z"), std::string::npos);
}

}  // namespace massage_motion
