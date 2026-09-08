#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "srdfdom/model.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "urdf_parser/urdf_parser.h"

#include "massage_motion/tool_orientation.hpp"

namespace massage_motion
{
namespace
{

moveit::core::RobotModelPtr make_test_robot_model()
{
  static const std::string urdf_xml = R"(
<robot name="orientation_test_robot">
  <link name="world"/>
  <link name="tool"/>
  <joint name="joint_yaw" type="revolute">
    <parent link="world"/><child link="tool"/><axis xyz="0 1 0"/>
    <limit lower="-3.2" upper="3.2" effort="1.0" velocity="1.0"/>
  </joint>
</robot>)";
  static const std::string srdf_xml = R"(
<robot name="orientation_test_robot">
  <group name="test_group"><joint name="joint_yaw"/></group>
</robot>)";
  const auto urdf_model = urdf::parseURDF(urdf_xml);
  if (!urdf_model) return nullptr;
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, srdf_xml)) return nullptr;
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

moveit_msgs::msg::RobotTrajectory trajectory(const std::vector<double> & angles)
{
  moveit_msgs::msg::RobotTrajectory result;
  result.joint_trajectory.joint_names = {"joint_yaw"};
  for (std::size_t index = 0U; index < angles.size(); ++index)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {angles[index]};
    point.time_from_start.sec = static_cast<std::int32_t>(index);
    result.joint_trajectory.points.push_back(point);
  }
  return result;
}

}  // namespace

TEST(ToolOrientationTest, AlignsToolZDownAndToolXWithPositiveY)
{
  ToolOrientationRequest request;
  const auto result = make_surface_aligned_tool_orientation(request);

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_NEAR(result.tool_z_world[0], 0.0, 1.0e-12);
  EXPECT_NEAR(result.tool_z_world[1], 0.0, 1.0e-12);
  EXPECT_NEAR(result.tool_z_world[2], -1.0, 1.0e-12);
  EXPECT_NEAR(result.tool_x_world[0], 0.0, 1.0e-12);
  EXPECT_NEAR(result.tool_x_world[1], 1.0, 1.0e-12);
  EXPECT_NEAR(result.tool_x_world[2], 0.0, 1.0e-12);
  EXPECT_NEAR(result.determinant, 1.0, 1.0e-12);
  const double norm = std::sqrt(
    result.orientation.x * result.orientation.x +
    result.orientation.y * result.orientation.y +
    result.orientation.z * result.orientation.z +
    result.orientation.w * result.orientation.w);
  EXPECT_NEAR(norm, 1.0, 1.0e-12);
}

TEST(ToolOrientationTest, RejectsZeroNonFiniteAndParallelInputs)
{
  ToolOrientationRequest zero;
  zero.surface_normal_world = {0.0, 0.0, 0.0};
  EXPECT_FALSE(make_surface_aligned_tool_orientation(zero).valid);

  ToolOrientationRequest non_finite;
  non_finite.tangent_direction_world[0] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(make_surface_aligned_tool_orientation(non_finite).valid);

  ToolOrientationRequest parallel;
  parallel.tangent_direction_world = {0.0, 0.0, -1.0};
  EXPECT_FALSE(make_surface_aligned_tool_orientation(parallel).valid);
}

TEST(ToolOrientationTest, SignedAlignmentRejectsUpwardTool)
{
  ToolAlignmentGateConfig config;
  config.expected_orientation.w = 1.0;
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;

  const auto result = evaluate_tool_alignment(identity, config);

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_FALSE(result.accepted);
  EXPECT_NEAR(result.minimum_signed_alignment, -1.0, 1.0e-12);
  EXPECT_NEAR(result.maximum_axis_error, M_PI, 1.0e-12);
}

TEST(ToolOrientationTest, WholeTrajectoryGateFindsIntermediateAxisViolation)
{
  ToolAlignmentGateConfig config;
  config.desired_tool_z_world = {0.0, 0.0, 1.0};
  config.expected_orientation.w = 1.0;
  config.maximum_axis_error = 0.1;
  const auto result = evaluate_tool_alignment_trajectory(
    make_test_robot_model(), trajectory({0.0, 0.4, 0.0}), "tool", config);

  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.worst_point_index, 1U);
  EXPECT_NEAR(result.maximum_axis_error, 0.4, 1.0e-12);

  config.inspect_all_samples = false;
  const auto endpoint = evaluate_tool_alignment_trajectory(
    make_test_robot_model(), trajectory({0.0, 0.4, 0.0}), "tool", config);
  EXPECT_TRUE(endpoint.accepted) << endpoint.message;
  EXPECT_EQ(endpoint.sample_count, 1U);
}

TEST(ToolOrientationTest, RejectsMalformedTrajectoryAndUnknownTool)
{
  ToolAlignmentGateConfig config;
  config.expected_orientation.w = 1.0;
  EXPECT_FALSE(evaluate_tool_alignment_trajectory(
      nullptr, trajectory({0.0}), "tool", config).valid);
  EXPECT_FALSE(evaluate_tool_alignment_trajectory(
      make_test_robot_model(), trajectory({0.0}), "missing", config).valid);
  auto malformed = trajectory({0.0});
  malformed.joint_trajectory.points.front().positions.clear();
  EXPECT_FALSE(evaluate_tool_alignment_trajectory(
      make_test_robot_model(), malformed, "tool", config).valid);
}

}  // namespace massage_motion
