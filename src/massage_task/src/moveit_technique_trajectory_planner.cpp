#include "massage_task/moveit_technique_trajectory_planner.hpp"

#include <cmath>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "tf2/exceptions.h"

namespace massage_task
{
namespace
{

double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

builtin_interfaces::msg::Duration duration_message(double value)
{
  builtin_interfaces::msg::Duration result;
  result.sec = static_cast<std::int32_t>(std::floor(value));
  result.nanosec = static_cast<std::uint32_t>(
    std::round((value - static_cast<double>(result.sec)) * 1.0e9));
  if (result.nanosec >= 1000000000U)
  {
    ++result.sec;
    result.nanosec -= 1000000000U;
  }
  return result;
}

geometry_msgs::msg::PoseStamped stamped(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  result.pose = pose;
  return result;
}

moveit_msgs::msg::RobotState terminal_state(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  moveit_msgs::msg::RobotState state;
  state.joint_state.name = trajectory.joint_trajectory.joint_names;
  state.joint_state.position = trajectory.joint_trajectory.points.back().positions;
  state.is_diff = true;
  return state;
}

double translation_distance(
  const geometry_msgs::msg::Pose & left,
  const geometry_msgs::msg::Pose & right)
{
  return std::sqrt(
    std::pow(left.position.x - right.position.x, 2) +
    std::pow(left.position.y - right.position.y, 2) +
    std::pow(left.position.z - right.position.z, 2));
}

massage_motion::PlanResult failed(const std::string & message)
{
  massage_motion::PlanResult result;
  result.error = massage_motion::MotionError::kPlanningFailed;
  result.message = message;
  return result;
}

}  // namespace

MoveItTechniqueTrajectoryPlanner::MoveItTechniqueTrajectoryPlanner(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<massage_motion::IMotionPlanner> planner,
  double velocity_scale, double acceleration_scale, double timeout,
  std::string tool_frame)
: logger_(node ? node->get_logger() : rclcpp::get_logger("technique_planner")),
  tf_buffer_(node ? std::make_unique<tf2_ros::Buffer>(node->get_clock()) : nullptr),
  tf_listener_(node ? std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_, node, false) : nullptr),
  planner_(std::move(planner)), velocity_scale_(velocity_scale),
  acceleration_scale_(acceleration_scale), timeout_(timeout),
  tool_frame_(std::move(tool_frame))
{
  if (!node || !planner_ || !std::isfinite(velocity_scale_) ||
    velocity_scale_ <= 0.0 || velocity_scale_ > 1.0 ||
    !std::isfinite(acceleration_scale_) || acceleration_scale_ <= 0.0 ||
    acceleration_scale_ > 1.0 || !std::isfinite(timeout_) || timeout_ <= 0.0 ||
    tool_frame_.empty())
  {
    throw std::invalid_argument("MoveIt 手法轨迹规划器依赖或配置无效");
  }
}

massage_motion::PlanResult MoveItTechniqueTrajectoryPlanner::plan(
  const massage_motion::TechniquePath & path,
  const moveit_msgs::msg::RobotState & start_state,
  bool align_to_current_tcp)
{
  if (path.points.size() < 2U || path.reference_frame.empty())
  {
    return failed("手法路径为空或缺少参考系");
  }

  auto aligned_path = path;
  if (align_to_current_tcp)
  {
    try
    {
      const auto current = tf_buffer_->lookupTransform(
        path.reference_frame, tool_frame_, tf2::TimePointZero);
      const auto & nominal_start = path.points.front().pose.position;
      const double dx = current.transform.translation.x - nominal_start.x;
      const double dy = current.transform.translation.y - nominal_start.y;
      const double dz = current.transform.translation.z - nominal_start.z;
      for (auto & point : aligned_path.points)
      {
        point.pose.position.x += dx;
        point.pose.position.y += dy;
        point.pose.position.z += dz;
      }
      RCLCPP_INFO(
        logger_, "手法路径对齐实际接触 TCP: offset=[%.6f %.6f %.6f] m",
        dx, dy, dz);
    }
    catch (const tf2::TransformException & exception)
    {
      return failed(std::string("读取实际接触 TCP 失败: ") + exception.what());
    }
  }

  const auto & first = aligned_path.points.front().pose;
  std::vector<std::size_t> boundaries{0U};
  for (std::size_t index = 1U; index < aligned_path.points.size(); ++index)
  {
    if (translation_distance(first, aligned_path.points[index].pose) < 1.0e-7 &&
      translation_distance(first, aligned_path.points[index - 1U].pose) >= 1.0e-7)
    {
      boundaries.push_back(index);
    }
  }
  if (boundaries.back() != aligned_path.points.size() - 1U)
  {
    boundaries.push_back(aligned_path.points.size() - 1U);
  }

  struct SegmentTarget
  {
    massage_motion::MotionType type;
    massage_motion::MotionTarget target;
  };
  std::vector<SegmentTarget> targets;
  if (aligned_path.type == massage_motion::TechniquePathType::kPush)
  {
    targets.push_back({
      massage_motion::MotionType::kLin,
      massage_motion::PoseTarget{
        stamped(aligned_path.points.back().pose, aligned_path.reference_frame)}});
  }
  else if (aligned_path.type == massage_motion::TechniquePathType::kPress)
  {
    for (std::size_t cycle = 1U; cycle < boundaries.size(); ++cycle)
    {
      const std::size_t begin = boundaries[cycle - 1U];
      const std::size_t end = boundaries[cycle];
      std::size_t deepest = begin;
      double largest_distance = -1.0;
      for (std::size_t index = begin; index <= end; ++index)
      {
        const double distance = translation_distance(
          first, aligned_path.points[index].pose);
        if (distance > largest_distance)
        {
          largest_distance = distance;
          deepest = index;
        }
      }
      targets.push_back({
        massage_motion::MotionType::kLin,
        massage_motion::PoseTarget{
          stamped(aligned_path.points[deepest].pose, aligned_path.reference_frame)}});
      targets.push_back({
        massage_motion::MotionType::kLin,
        massage_motion::PoseTarget{
          stamped(aligned_path.points[end].pose, aligned_path.reference_frame)}});
    }
  }
  else
  {
    for (std::size_t cycle = 1U; cycle < boundaries.size(); ++cycle)
    {
      const std::size_t begin = boundaries[cycle - 1U];
      const std::size_t end = boundaries[cycle];
      const std::size_t span = end - begin;
      const std::size_t quarter = begin + span / 4U;
      const std::size_t half = begin + span / 2U;
      const std::size_t three_quarter = begin + (3U * span) / 4U;
      targets.push_back({
        massage_motion::MotionType::kCirc,
        massage_motion::CircularTarget{
          stamped(aligned_path.points[quarter].pose, aligned_path.reference_frame),
          stamped(aligned_path.points[half].pose, aligned_path.reference_frame)}});
      targets.push_back({
        massage_motion::MotionType::kCirc,
        massage_motion::CircularTarget{
          stamped(
            aligned_path.points[three_quarter].pose,
            aligned_path.reference_frame),
          stamped(aligned_path.points[end].pose, aligned_path.reference_frame)}});
    }
  }
  if (targets.empty())
  {
    return failed("无法从手法路径提取连续规划段");
  }

  massage_motion::PlanResult combined;
  moveit_msgs::msg::RobotState next_start = start_state;
  double time_offset = 0.0;
  for (std::size_t index = 0U; index < targets.size(); ++index)
  {
    massage_motion::MotionRequest request;
    request.request_id = "technique_segment_" + std::to_string(index + 1U);
    request.motion_type = targets[index].type;
    request.target = targets[index].target;
    request.velocity_scale = velocity_scale_;
    request.acceleration_scale = acceleration_scale_;
    request.planning_timeout = timeout_;
    request.start_state = next_start;
    massage_motion::PlanResult segment;
    constexpr std::array<double, 3> circ_backoff{{1.0, 0.5, 0.25}};
    const std::size_t attempt_count =
      request.motion_type == massage_motion::MotionType::kCirc ?
      circ_backoff.size() : 1U;
    for (std::size_t attempt = 0U; attempt < attempt_count; ++attempt)
    {
      const double factor = circ_backoff[attempt];
      request.velocity_scale = velocity_scale_ * factor;
      request.acceleration_scale = acceleration_scale_ * factor;
      segment = planner_->plan(request);
      if (segment.success) break;
      RCLCPP_WARN(
        logger_,
        "CIRC 规划比例退让: segment=%zu attempt=%zu/%zu velocity_scale=%.6f acceleration_scale=%.6f result=%s",
        index + 1U, attempt + 1U, attempt_count,
        request.velocity_scale, request.acceleration_scale,
        segment.message.c_str());
    }
    if (!segment.success || segment.trajectory.joint_trajectory.points.empty())
    {
      return failed(request.request_id + " 失败: " + segment.message);
    }
    if (combined.trajectory.joint_trajectory.joint_names.empty())
    {
      combined.trajectory.joint_trajectory.joint_names =
        segment.trajectory.joint_trajectory.joint_names;
    }
    else if (combined.trajectory.joint_trajectory.joint_names !=
      segment.trajectory.joint_trajectory.joint_names)
    {
      return failed("手法分段轨迹关节顺序不一致");
    }
    combined.execution_segments.push_back({
      request.request_id,
      request.motion_type,
      request.target,
      segment.trajectory,
      request.velocity_scale,
      request.acceleration_scale,
      segment.planner_id,
      aligned_path.nominal_speed});
    for (auto point : segment.trajectory.joint_trajectory.points)
    {
      point.time_from_start = duration_message(
        time_offset + seconds(point.time_from_start));
      combined.trajectory.joint_trajectory.points.push_back(std::move(point));
    }
    time_offset = seconds(
      combined.trajectory.joint_trajectory.points.back().time_from_start);
    combined.planning_time += segment.planning_time;
    next_start = terminal_state(segment.trajectory);
  }
  combined.success = true;
  combined.error = massage_motion::MotionError::kNone;
  combined.message = "手法分段轨迹规划并拼接成功";
  combined.planner_id = "semantic_cartesian_segments";
  return combined;
}

}  // namespace massage_task
