#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "srdfdom/model.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "urdf_parser/urdf_parser.h"

#include "massage_motion/elbow_posture_geometry.hpp"

namespace massage_motion
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

moveit::core::RobotModelPtr make_test_robot_model()
{
  static const std::string urdf_xml = R"(
<robot name="elbow_test_robot">
  <link name="world"/>
  <link name="Link_02"/>
  <link name="Link_03"/>
  <link name="Link_04"/>
  <joint name="joint_shoulder" type="revolute">
    <parent link="world"/>
    <child link="Link_02"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.2" upper="3.2" effort="1.0" velocity="1.0"/>
  </joint>
  <joint name="joint_elbow" type="revolute">
    <parent link="Link_02"/>
    <child link="Link_03"/>
    <origin xyz="1 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.2" upper="3.2" effort="1.0" velocity="1.0"/>
  </joint>
  <joint name="wrist_offset" type="fixed">
    <parent link="Link_03"/>
    <child link="Link_04"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>)";
  static const std::string srdf_xml = R"(
<robot name="elbow_test_robot">
  <group name="test_group">
    <joint name="joint_shoulder"/>
    <joint name="joint_elbow"/>
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

moveit_msgs::msg::RobotTrajectory make_trajectory(
  const std::vector<double> & elbow_positions,
  double shoulder_position = 0.0)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {
    "joint_shoulder", "joint_elbow"};
  for (std::size_t index = 0; index < elbow_positions.size(); ++index)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {shoulder_position, elbow_positions[index]};
    point.time_from_start.sec = static_cast<std::int32_t>(index);
    trajectory.joint_trajectory.points.push_back(point);
  }
  return trajectory;
}

}  // namespace

TEST(ElbowPostureGeometryTest, CalculatesSignedOffsetAndBendDistance)
{
  const auto metrics = calculate_elbow_posture_metrics(
    make_test_robot_model(), make_trajectory({kPi / 2.0}));

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_NEAR(metrics.start_signed_offset, 0.5, 1.0e-9);
  EXPECT_NEAR(metrics.end_signed_offset, 0.5, 1.0e-9);
  EXPECT_NEAR(metrics.minimum_signed_offset, 0.5, 1.0e-9);
  EXPECT_NEAR(metrics.maximum_signed_offset, 0.5, 1.0e-9);
  EXPECT_NEAR(metrics.minimum_bend_distance, std::sqrt(0.5), 1.0e-9);
  EXPECT_NEAR(metrics.maximum_bend_distance, std::sqrt(0.5), 1.0e-9);
  EXPECT_NEAR(
    metrics.minimum_direction_observability, std::sqrt(0.5), 1.0e-9);
  EXPECT_NEAR(
    metrics.minimum_shoulder_wrist_distance, std::sqrt(2.0), 1.0e-9);
  EXPECT_EQ(metrics.side_change_count, 0U);
  EXPECT_EQ(metrics.ambiguous_side_sample_count, 0U);
  EXPECT_EQ(metrics.direction_degenerate_sample_count, 0U);
  EXPECT_EQ(metrics.sample_count, 1U);
}

TEST(ElbowPostureGeometryTest, DistinguishesOppositeElbowSide)
{
  const auto positive = calculate_elbow_posture_metrics(
    make_test_robot_model(), make_trajectory({kPi / 2.0}));
  const auto negative = calculate_elbow_posture_metrics(
    make_test_robot_model(), make_trajectory({-kPi / 2.0}));

  ASSERT_TRUE(positive.valid);
  ASSERT_TRUE(negative.valid);
  EXPECT_GT(positive.minimum_signed_offset, 0.0);
  EXPECT_LT(negative.maximum_signed_offset, 0.0);
  EXPECT_NEAR(
    positive.minimum_bend_distance, negative.minimum_bend_distance, 1.0e-9);
}

TEST(ElbowPostureGeometryTest, DetectsSideChangeThroughStraightConfiguration)
{
  const auto metrics = calculate_elbow_posture_metrics(
    make_test_robot_model(),
    make_trajectory({kPi / 2.0, 0.0, -kPi / 2.0}));

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_EQ(metrics.side_change_count, 1U);
  EXPECT_EQ(metrics.ambiguous_side_sample_count, 1U);
  EXPECT_NEAR(metrics.minimum_bend_distance, 0.0, 1.0e-9);
  EXPECT_EQ(metrics.minimum_bend_point_index, 1U);
  EXPECT_DOUBLE_EQ(metrics.minimum_bend_point_time, 1.0);
}

TEST(ElbowPostureGeometryTest, DetectsDirectionDegeneracyForVerticalArm)
{
  const auto metrics = calculate_elbow_posture_metrics(
    make_test_robot_model(), make_trajectory({0.0}, -kPi / 2.0));

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_NEAR(metrics.minimum_direction_observability, 0.0, 1.0e-9);
  EXPECT_EQ(metrics.direction_degenerate_sample_count, 1U);
  EXPECT_EQ(metrics.minimum_observability_point_index, 0U);
}

TEST(ElbowPostureGeometryTest, NormalizesSurfaceNormal)
{
  ElbowPostureGeometryConfig config;
  config.surface_normal = {0.0, 0.0, 4.0};
  const auto metrics = calculate_elbow_posture_metrics(
    make_test_robot_model(), make_trajectory({kPi / 2.0}), config);

  ASSERT_TRUE(metrics.valid) << metrics.message;
  EXPECT_NEAR(metrics.minimum_signed_offset, 0.5, 1.0e-9);
}

TEST(ElbowPostureGeometryTest, RejectsInvalidModelLinksAndNormal)
{
  const auto trajectory = make_trajectory({kPi / 2.0});
  EXPECT_FALSE(calculate_elbow_posture_metrics(nullptr, trajectory).valid);

  ElbowPostureGeometryConfig missing_link;
  missing_link.elbow_link = "missing";
  EXPECT_FALSE(calculate_elbow_posture_metrics(
    make_test_robot_model(), trajectory, missing_link).valid);

  ElbowPostureGeometryConfig zero_normal;
  zero_normal.surface_normal = {0.0, 0.0, 0.0};
  EXPECT_FALSE(calculate_elbow_posture_metrics(
    make_test_robot_model(), trajectory, zero_normal).valid);

  ElbowPostureGeometryConfig nan_normal;
  nan_normal.surface_normal[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(calculate_elbow_posture_metrics(
    make_test_robot_model(), trajectory, nan_normal).valid);
}

TEST(ElbowPostureGeometryTest, RejectsIncompleteAndMalformedTrajectories)
{
  auto incomplete = make_trajectory({kPi / 2.0});
  incomplete.joint_trajectory.joint_names = {"joint_shoulder"};
  incomplete.joint_trajectory.points.front().positions = {0.0};
  EXPECT_FALSE(calculate_elbow_posture_metrics(
    make_test_robot_model(), incomplete).valid);

  auto non_finite = make_trajectory({kPi / 2.0});
  non_finite.joint_trajectory.points.front().positions[1] =
    std::numeric_limits<double>::infinity();
  EXPECT_FALSE(calculate_elbow_posture_metrics(
    make_test_robot_model(), non_finite).valid);

  auto decreasing_time = make_trajectory({0.5, 0.6});
  decreasing_time.joint_trajectory.points[1].time_from_start.sec = 0;
  decreasing_time.joint_trajectory.points[0].time_from_start.sec = 1;
  EXPECT_FALSE(calculate_elbow_posture_metrics(
    make_test_robot_model(), decreasing_time).valid);
}

}  // namespace massage_motion
