#ifndef MASSAGE_MOTION__POSE_IK_CANDIDATE_GENERATOR_HPP_
#define MASSAGE_MOTION__POSE_IK_CANDIDATE_GENERATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/robot_model/robot_model.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace massage_motion
{

struct IkTimeoutPolicy
{
  double base_timeout{0.02};
  double timeout_per_meter{0.25};
  double timeout_per_radian{0.05};
  double minimum_timeout{0.02};
  double maximum_timeout{0.50};
  double failure_backoff_factor{1.35};
};

struct PoseDistance
{
  double translation{0.0};
  double rotation{0.0};
};

struct JointVariableInfo
{
  std::string name;
  bool revolute{false};
  bool position_bounded{false};
  double minimum_position{0.0};
  double maximum_position{0.0};
};

struct OrderedJointState
{
  bool valid{false};
  std::string message;
  std::vector<double> positions;
};

struct PoseIkCandidateGeneratorConfig
{
  std::size_t maximum_attempts{24};
  std::size_t maximum_unique_candidates{8};
  double duplicate_tolerance{1.0e-4};
  std::uint32_t random_seed{684U};
  IkTimeoutPolicy timeout_policy;
};

struct PoseIkAttemptReport
{
  std::size_t attempt{0};
  double timeout{0.0};
  bool used_current_state_seed{false};
  bool ik_success{false};
  bool duplicate{false};
  std::string message;
};

struct PoseIkCandidate
{
  std::size_t source_attempt{0};
  double timeout{0.0};
  std::vector<double> positions;
};

struct PoseIkCandidateReport
{
  bool success{false};
  std::string message;
  std::string model_frame;
  std::string requested_tip_link;
  std::string solver_tip_link;
  bool fixed_tip_offset_applied{false};
  PoseDistance target_distance;
  std::vector<std::string> variable_names;
  std::vector<PoseIkAttemptReport> attempts;
  std::vector<PoseIkCandidate> candidates;
};

geometry_msgs::msg::Pose convert_business_tip_target_to_solver_tip(
  const geometry_msgs::msg::Pose & business_tip_target,
  const geometry_msgs::msg::Pose & solver_to_business_tip);

bool valid_ik_timeout_policy(const IkTimeoutPolicy & policy);

double calculate_ik_timeout(
  const PoseDistance & distance,
  std::size_t attempt_index,
  const IkTimeoutPolicy & policy);

std::uint32_t ik_seed_for_attempt(
  std::uint32_t base_seed, std::size_t attempt_index);

OrderedJointState order_joint_state_positions(
  const std::vector<std::string> & required_names,
  const sensor_msgs::msg::JointState & joint_state);

double closest_equivalent_revolute_position(
  double position, double reference, bool position_bounded,
  double minimum_position, double maximum_position);

std::vector<double> normalize_revolute_positions_near_reference(
  const std::vector<double> & positions,
  const std::vector<double> & reference,
  const std::vector<JointVariableInfo> & variables);

bool equivalent_joint_solution(
  const std::vector<double> & lhs,
  const std::vector<double> & rhs,
  const std::vector<JointVariableInfo> & variables,
  double tolerance);

class PoseIkCandidateGenerator
{
public:
  PoseIkCandidateGenerator(
    moveit::core::RobotModelConstPtr robot_model,
    std::string planning_group,
    std::string tip_link,
    PoseIkCandidateGeneratorConfig config = {});

  PoseIkCandidateReport generate(
    const sensor_msgs::msg::JointState & current_joint_state,
    const geometry_msgs::msg::PoseStamped & target_pose) const;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string planning_group_;
  std::string tip_link_;
  PoseIkCandidateGeneratorConfig config_;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__POSE_IK_CANDIDATE_GENERATOR_HPP_
