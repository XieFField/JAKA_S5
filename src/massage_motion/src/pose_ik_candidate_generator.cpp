#include "massage_motion/pose_ik_candidate_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "Eigen/Geometry"
#include "moveit/robot_model/joint_model.h"
#include "moveit/robot_model/link_model.h"
#include "moveit/robot_state/robot_state.h"
#include "random_numbers/random_numbers.h"

namespace massage_motion
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const double values[] = {
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w};
  if (!std::all_of(
      std::begin(values), std::end(values),
      [](double value) {return std::isfinite(value);} ))
  {
    return false;
  }
  const Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  return orientation.squaredNorm() > 1.0e-12;
}

bool valid_generator_config(const PoseIkCandidateGeneratorConfig & config)
{
  return config.maximum_attempts > 0U &&
         config.maximum_unique_candidates > 0U &&
         std::isfinite(config.duplicate_tolerance) &&
         config.duplicate_tolerance > 0.0 &&
         valid_ik_timeout_policy(config.timeout_policy);
}

Eigen::Quaterniond normalized_quaternion(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  Eigen::Quaterniond result(
    quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  if (!std::isfinite(result.squaredNorm()) || result.squaredNorm() <= 1.0e-12)
  {
    return Eigen::Quaterniond::Identity();
  }
  result.normalize();
  return result;
}

Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = normalized_quaternion(pose.orientation).toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond orientation(transform.rotation());
  orientation.normalize();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

PoseDistance calculate_pose_distance(
  const Eigen::Isometry3d & current,
  const geometry_msgs::msg::Pose & target)
{
  const Eigen::Vector3d target_translation(
    target.position.x, target.position.y, target.position.z);
  const Eigen::Quaterniond current_orientation(current.rotation());
  const Eigen::Quaterniond target_orientation =
    normalized_quaternion(target.orientation);
  PoseDistance result;
  result.translation = (target_translation - current.translation()).norm();
  result.rotation = Eigen::AngleAxisd(
    current_orientation.conjugate() * target_orientation).angle();
  return result;
}

std::vector<JointVariableInfo> make_variable_info(
  const moveit::core::RobotModel & robot_model,
  const std::vector<std::string> & variable_names)
{
  std::vector<JointVariableInfo> variables;
  variables.reserve(variable_names.size());
  for (const auto & name : variable_names)
  {
    const auto * joint = robot_model.getJointOfVariable(name);
    const auto & bounds = robot_model.getVariableBounds(name);
    JointVariableInfo variable;
    variable.name = name;
    variable.revolute = joint &&
      joint->getType() == moveit::core::JointModel::REVOLUTE;
    variable.position_bounded = bounds.position_bounded_;
    variable.minimum_position = bounds.min_position_;
    variable.maximum_position = bounds.max_position_;
    variables.push_back(std::move(variable));
  }
  return variables;
}

}  // namespace

geometry_msgs::msg::Pose convert_business_tip_target_to_solver_tip(
  const geometry_msgs::msg::Pose & business_tip_target,
  const geometry_msgs::msg::Pose & solver_to_business_tip)
{
  if (!finite_pose(business_tip_target) || !finite_pose(solver_to_business_tip))
  {
    geometry_msgs::msg::Pose invalid;
    invalid.position.x = std::numeric_limits<double>::quiet_NaN();
    return invalid;
  }
  return isometry_to_pose(
    pose_to_isometry(business_tip_target) *
    pose_to_isometry(solver_to_business_tip).inverse());
}

bool valid_ik_timeout_policy(const IkTimeoutPolicy & policy)
{
  return finite_nonnegative(policy.base_timeout) &&
         finite_nonnegative(policy.timeout_per_meter) &&
         finite_nonnegative(policy.timeout_per_radian) &&
         std::isfinite(policy.minimum_timeout) &&
         policy.minimum_timeout > 0.0 &&
         std::isfinite(policy.maximum_timeout) &&
         policy.maximum_timeout >= policy.minimum_timeout &&
         std::isfinite(policy.failure_backoff_factor) &&
         policy.failure_backoff_factor >= 1.0;
}

double calculate_ik_timeout(
  const PoseDistance & distance,
  std::size_t attempt_index,
  const IkTimeoutPolicy & policy)
{
  if (!valid_ik_timeout_policy(policy) ||
    !finite_nonnegative(distance.translation) ||
    !finite_nonnegative(distance.rotation))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double task_timeout = policy.base_timeout +
    distance.translation * policy.timeout_per_meter +
    distance.rotation * policy.timeout_per_radian;
  const double bounded_task_timeout = std::clamp(
    task_timeout, policy.minimum_timeout, policy.maximum_timeout);
  const double backoff = std::pow(
    policy.failure_backoff_factor, static_cast<double>(attempt_index));
  return std::clamp(
    bounded_task_timeout * backoff,
    policy.minimum_timeout, policy.maximum_timeout);
}

std::uint32_t ik_seed_for_attempt(
  std::uint32_t base_seed, std::size_t attempt_index)
{
  std::uint32_t value = base_seed +
    static_cast<std::uint32_t>(attempt_index) + 0x9e3779b9U;
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return value;
}

OrderedJointState order_joint_state_positions(
  const std::vector<std::string> & required_names,
  const sensor_msgs::msg::JointState & joint_state)
{
  OrderedJointState result;
  if (required_names.empty() || joint_state.name.empty() ||
    joint_state.name.size() != joint_state.position.size())
  {
    result.message = "关节名称为空或 /joint_states 名称与位置数量不一致";
    return result;
  }

  std::unordered_map<std::string, double> positions;
  positions.reserve(joint_state.name.size());
  for (std::size_t index = 0; index < joint_state.name.size(); ++index)
  {
    if (joint_state.name[index].empty() ||
      !std::isfinite(joint_state.position[index]) ||
      !positions.emplace(joint_state.name[index], joint_state.position[index]).second)
    {
      result.message = "/joint_states 包含空名称、重复名称或非有限位置";
      return result;
    }
  }

  std::unordered_set<std::string> required_unique;
  result.positions.reserve(required_names.size());
  for (const auto & name : required_names)
  {
    if (name.empty() || !required_unique.emplace(name).second)
    {
      result.message = "规划组包含空变量名或重复变量名";
      result.positions.clear();
      return result;
    }
    const auto iterator = positions.find(name);
    if (iterator == positions.end())
    {
      result.message = "/joint_states 缺少规划组变量: " + name;
      result.positions.clear();
      return result;
    }
    result.positions.push_back(iterator->second);
  }
  result.valid = true;
  result.message = "关节状态顺序转换成功";
  return result;
}

double closest_equivalent_revolute_position(
  double position, double reference, bool position_bounded,
  double minimum_position, double maximum_position)
{
  if (!std::isfinite(position) || !std::isfinite(reference))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const long center = std::lround((reference - position) / kTwoPi);
  double best = position;
  double best_distance = std::numeric_limits<double>::infinity();
  for (long offset = -3; offset <= 3; ++offset)
  {
    const double candidate = position +
      static_cast<double>(center + offset) * kTwoPi;
    const bool in_bounds = !position_bounded ||
      (candidate >= minimum_position && candidate <= maximum_position);
    if (in_bounds && std::abs(candidate - reference) < best_distance)
    {
      best = candidate;
      best_distance = std::abs(candidate - reference);
    }
  }
  return best_distance < std::numeric_limits<double>::infinity() ? best : position;
}

std::vector<double> normalize_revolute_positions_near_reference(
  const std::vector<double> & positions,
  const std::vector<double> & reference,
  const std::vector<JointVariableInfo> & variables)
{
  if (positions.size() != reference.size() || positions.size() != variables.size())
  {
    return {};
  }
  std::vector<double> normalized = positions;
  for (std::size_t index = 0; index < normalized.size(); ++index)
  {
    if (!std::isfinite(normalized[index]) || !std::isfinite(reference[index]))
    {
      return {};
    }
    if (variables[index].revolute)
    {
      normalized[index] = closest_equivalent_revolute_position(
        normalized[index], reference[index], variables[index].position_bounded,
        variables[index].minimum_position, variables[index].maximum_position);
    }
  }
  return normalized;
}

bool equivalent_joint_solution(
  const std::vector<double> & lhs,
  const std::vector<double> & rhs,
  const std::vector<JointVariableInfo> & variables,
  double tolerance)
{
  if (lhs.size() != rhs.size() || lhs.size() != variables.size() ||
    !std::isfinite(tolerance) || tolerance <= 0.0)
  {
    return false;
  }
  for (std::size_t index = 0; index < lhs.size(); ++index)
  {
    if (!std::isfinite(lhs[index]) || !std::isfinite(rhs[index]))
    {
      return false;
    }
    const double delta = variables[index].revolute ?
      std::remainder(lhs[index] - rhs[index], kTwoPi) :
      lhs[index] - rhs[index];
    if (std::abs(delta) > tolerance)
    {
      return false;
    }
  }
  return true;
}

PoseIkCandidateGenerator::PoseIkCandidateGenerator(
  moveit::core::RobotModelConstPtr robot_model,
  std::string planning_group,
  std::string tip_link,
  PoseIkCandidateGeneratorConfig config)
: robot_model_(std::move(robot_model)),
  planning_group_(std::move(planning_group)),
  tip_link_(std::move(tip_link)),
  config_(config)
{
}

PoseIkCandidateReport PoseIkCandidateGenerator::generate(
  const sensor_msgs::msg::JointState & current_joint_state,
  const geometry_msgs::msg::PoseStamped & target_pose) const
{
  PoseIkCandidateReport report;
  if (!robot_model_ || planning_group_.empty() || tip_link_.empty() ||
    !valid_generator_config(config_))
  {
    report.message = "IK 候选生成器模型、名称或配置无效";
    return report;
  }
  report.model_frame = robot_model_->getModelFrame();
  report.requested_tip_link = tip_link_;
  if (!finite_pose(target_pose.pose))
  {
    report.message = "IK 目标位姿包含非有限数值或零四元数";
    return report;
  }
  if (target_pose.header.frame_id != report.model_frame)
  {
    report.message = "IK 目标坐标系必须等于 RobotModel 坐标系: target=" +
      target_pose.header.frame_id + " model=" + report.model_frame;
    return report;
  }
  const auto * group = robot_model_->getJointModelGroup(planning_group_);
  if (!group)
  {
    report.message = "RobotModel 中不存在规划组: " + planning_group_;
    return report;
  }
  const auto * requested_tip_model = robot_model_->getLinkModel(tip_link_);
  if (!requested_tip_model)
  {
    report.message = "RobotModel 中不存在业务末端连杆: " + tip_link_;
    return report;
  }
  const auto solver = group->getSolverInstance();
  if (!solver || solver->getTipFrames().empty())
  {
    report.message = "规划组没有已初始化的 IK 求解器";
    return report;
  }
  report.solver_tip_link = solver->getTipFrame();
  if (!group->canSetStateFromIK(report.solver_tip_link))
  {
    report.message = "IK 求解器原生 tip 不受规划组支持: " +
      report.solver_tip_link;
    return report;
  }

  report.variable_names = group->getVariableNames();
  const auto ordered = order_joint_state_positions(
    report.variable_names, current_joint_state);
  if (!ordered.valid)
  {
    report.message = ordered.message;
    return report;
  }
  const auto variables = make_variable_info(*robot_model_, report.variable_names);

  moveit::core::RobotState current_state(robot_model_);
  current_state.setToDefaultValues();
  current_state.setJointGroupPositions(group, ordered.positions);
  current_state.update();
  report.target_distance = calculate_pose_distance(
    current_state.getGlobalLinkTransform(tip_link_), target_pose.pose);

  geometry_msgs::msg::Pose solver_target_pose = target_pose.pose;
  if (report.solver_tip_link != tip_link_)
  {
    const auto * rigid_parent =
      moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(
      requested_tip_model);
    if (!rigid_parent || rigid_parent->getName() != report.solver_tip_link)
    {
      report.message = "业务末端不是 IK 原生 tip 的固定子连杆: business=" +
        tip_link_ + " solver=" + report.solver_tip_link;
      return report;
    }
    const Eigen::Isometry3d solver_to_business =
      current_state.getGlobalLinkTransform(report.solver_tip_link).inverse() *
      current_state.getGlobalLinkTransform(tip_link_);
    solver_target_pose = convert_business_tip_target_to_solver_tip(
      target_pose.pose, isometry_to_pose(solver_to_business));
    if (!finite_pose(solver_target_pose))
    {
      report.message = "业务末端到 IK 原生 tip 的固定偏移换算失败";
      return report;
    }
    report.fixed_tip_offset_applied = true;
  }

  for (std::size_t attempt_index = 0;
    attempt_index < config_.maximum_attempts &&
    report.candidates.size() < config_.maximum_unique_candidates;
    ++attempt_index)
  {
    PoseIkAttemptReport attempt;
    attempt.attempt = attempt_index + 1U;
    attempt.timeout = calculate_ik_timeout(
      report.target_distance, attempt_index, config_.timeout_policy);
    attempt.used_current_state_seed = attempt_index == 0U;

    moveit::core::RobotState candidate_state(current_state);
    if (attempt_index > 0U)
    {
      random_numbers::RandomNumberGenerator rng(
        ik_seed_for_attempt(config_.random_seed, attempt_index));
      candidate_state.setToRandomPositions(group, rng);
    }
    attempt.ik_success = candidate_state.setFromIK(
      group, solver_target_pose, report.solver_tip_link, attempt.timeout);
    if (!attempt.ik_success)
    {
      attempt.message = "IK 未找到解";
      report.attempts.push_back(std::move(attempt));
      continue;
    }

    std::vector<double> positions;
    candidate_state.copyJointGroupPositions(group, positions);
    positions = normalize_revolute_positions_near_reference(
      positions, ordered.positions, variables);
    if (positions.empty())
    {
      attempt.message = "IK 解归一化失败";
      report.attempts.push_back(std::move(attempt));
      continue;
    }
    candidate_state.setJointGroupPositions(group, positions);
    if (!candidate_state.satisfiesBounds(group))
    {
      attempt.message = "IK 解超过关节位置限制";
      report.attempts.push_back(std::move(attempt));
      continue;
    }

    attempt.duplicate = std::any_of(
      report.candidates.begin(), report.candidates.end(),
      [&](const PoseIkCandidate & candidate)
      {
        return equivalent_joint_solution(
          candidate.positions, positions, variables,
          config_.duplicate_tolerance);
      });
    if (attempt.duplicate)
    {
      attempt.message = "IK 解与已有候选模 2pi 等价";
      report.attempts.push_back(std::move(attempt));
      continue;
    }

    PoseIkCandidate candidate;
    candidate.source_attempt = attempt.attempt;
    candidate.timeout = attempt.timeout;
    candidate.positions = std::move(positions);
    report.candidates.push_back(std::move(candidate));
    attempt.message = "新增唯一 IK 候选";
    report.attempts.push_back(std::move(attempt));
  }

  report.success = !report.candidates.empty();
  std::ostringstream message;
  message << (report.success ? "IK 候选生成完成" : "IK 候选生成失败")
          << ": attempts=" << report.attempts.size()
          << ", unique=" << report.candidates.size()
          << ", translation=" << report.target_distance.translation << " m"
          << ", rotation=" << report.target_distance.rotation << " rad";
  report.message = message.str();
  return report;
}

}  // namespace massage_motion
