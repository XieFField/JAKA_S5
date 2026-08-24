#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

#include "massage_motion/motion_types.hpp"
#include "massage_motion/ptp_planner.hpp"
#include "massage_motion/technique_path_generator.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr char kMarkerName[] = "push_start_pose";

enum class PreviewStatus : std::uint8_t
{
  kPending = 0,
  kPlanning,
  kValid,
  kInvalid,
};

double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool normalize_orientation(geometry_msgs::msg::Pose & pose)
{
  if (!finite_pose(pose))
  {
    return false;
  }
  tf2::Quaternion orientation;
  tf2::fromMsg(pose.orientation, orientation);
  const double norm_squared = orientation.length2();
  if (!std::isfinite(norm_squared) || norm_squared <= 1.0e-12)
  {
    return false;
  }
  orientation.normalize();
  pose.orientation = tf2::toMsg(orientation);
  return true;
}

std_msgs::msg::ColorRGBA status_color(PreviewStatus status)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.9F;
  switch (status)
  {
    case PreviewStatus::kValid:
      color.g = 0.85F;
      color.b = 0.15F;
      break;
    case PreviewStatus::kInvalid:
      color.r = 0.9F;
      color.g = 0.1F;
      break;
    case PreviewStatus::kPlanning:
      color.r = 0.95F;
      color.g = 0.55F;
      break;
    case PreviewStatus::kPending:
    default:
      color.r = 0.95F;
      color.g = 0.85F;
      color.b = 0.1F;
      break;
  }
  return color;
}

class PushPoseTuner : public rclcpp::Node
{
public:
  explicit PushPoseTuner(const rclcpp::NodeOptions & options)
  : Node("push_pose_tuner", options)
  {
  }

  ~PushPoseTuner() override
  {
    stop();
  }

  bool initialize()
  {
    get_parameter_or("reference_frame", reference_frame_, std::string{"world"});
    get_parameter_or(
      "end_effector_link", end_effector_link_,
      std::string{"massage_tool_tip"});
    get_parameter_or("direction_x", direction_x_, 1.0);
    get_parameter_or("direction_y", direction_y_, 0.0);
    get_parameter_or("push_length", push_length_, 0.05);
    get_parameter_or("push_speed", push_speed_, 0.01);
    get_parameter_or("sample_period", sample_period_, 0.05);
    get_parameter_or("maximum_speed", maximum_speed_, 0.02);
    get_parameter_or("planning_timeout", planning_timeout_, 5.0);
    get_parameter_or("velocity_scale", velocity_scale_, 0.05);
    get_parameter_or("acceleration_scale", acceleration_scale_, 0.05);
    get_parameter_or("maximum_translation_nudge", maximum_translation_nudge_, 0.02);
    get_parameter_or(
      "maximum_rotation_nudge_deg", maximum_rotation_nudge_deg_, 10.0);
    get_parameter_or("marker_scale", marker_scale_, 0.15);
    get_parameter_or(
      "output_yaml", output_yaml_,
      std::string{"/tmp/massage_push_pose.yaml"});

    if (!valid_parameters())
    {
      RCLCPP_ERROR(get_logger(), "姿态调节器参数无效，节点不会启动");
      return false;
    }

    const auto shared_node =
      std::static_pointer_cast<rclcpp::Node>(shared_from_this());
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_, shared_node, false);

    interactive_server_ =
      std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "push_pose_tuner", shared_node);

    auto preview_qos = rclcpp::QoS(1).reliable().transient_local();
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/push_pose_tuner/path_markers", preview_qos);
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/push_pose_tuner/candidate_pose", preview_qos);
    nudge_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/push_pose_tuner/nudge", 10,
      std::bind(&PushPoseTuner::handle_nudge, this, std::placeholders::_1));

    confirm_service_ = create_service<std_srvs::srv::Trigger>(
      "~/confirm",
      std::bind(
        &PushPoseTuner::confirm_pose, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      std::bind(
        &PushPoseTuner::reset_pose, this,
        std::placeholders::_1, std::placeholders::_2));
    replan_service_ = create_service<std_srvs::srv::Trigger>(
      "~/replan",
      std::bind(
        &PushPoseTuner::replan_pose, this,
        std::placeholders::_1, std::placeholders::_2));

    massage_motion::PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = end_effector_link_;
    planner_config.reference_frame = reference_frame_;
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";
    planner_ = std::make_unique<massage_motion::PtpPlanner>(
      shared_node, planner_config);

    planning_thread_ = std::thread(&PushPoseTuner::planning_loop, this);
    initialization_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&PushPoseTuner::initialize_from_tf, this));

    RCLCPP_INFO(
      get_logger(),
      "推法姿态调节器已启动: preview_only=true, frame=%s, tool=%s, "
      "push=[direction=(%.3f, %.3f), length=%.3f m, speed=%.3f m/s], "
      "candidate_position_limit=none, orientation_preset=none, output=%s",
      reference_frame_.c_str(), end_effector_link_.c_str(),
      direction_x_, direction_y_, push_length_, push_speed_,
      output_yaml_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "本节点没有轨迹执行器；绿色只表示候选起点的 PTP 只规划通过，"
      "不代表青色推路径已经执行验证");
    return true;
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_)
      {
        return;
      }
      stopping_ = true;
    }
    planning_condition_.notify_all();
    if (planning_thread_.joinable())
    {
      planning_thread_.join();
    }
    planner_.reset();
  }

private:
  bool valid_parameters() const
  {
    const double direction_norm = std::hypot(direction_x_, direction_y_);
    return !reference_frame_.empty() && !end_effector_link_.empty() &&
           std::isfinite(direction_norm) && direction_norm > 1.0e-12 &&
           std::isfinite(push_length_) && push_length_ > 0.0 &&
           std::isfinite(push_speed_) && push_speed_ > 0.0 &&
           std::isfinite(sample_period_) && sample_period_ > 0.0 &&
           std::isfinite(maximum_speed_) && maximum_speed_ >= push_speed_ &&
           std::isfinite(planning_timeout_) && planning_timeout_ > 0.0 &&
           std::isfinite(velocity_scale_) && velocity_scale_ > 0.0 &&
           velocity_scale_ <= 1.0 &&
           std::isfinite(acceleration_scale_) && acceleration_scale_ > 0.0 &&
           acceleration_scale_ <= 1.0 &&
           std::isfinite(maximum_translation_nudge_) &&
           maximum_translation_nudge_ > 0.0 &&
           std::isfinite(maximum_rotation_nudge_deg_) &&
           maximum_rotation_nudge_deg_ > 0.0 &&
           std::isfinite(marker_scale_) && marker_scale_ > 0.0 &&
           !output_yaml_.empty();
  }

  void initialize_from_tf()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pose_initialized_)
      {
        initialization_timer_->cancel();
        return;
      }
    }

    try
    {
      const auto transform = tf_buffer_->lookupTransform(
        reference_frame_, end_effector_link_, tf2::TimePointZero);
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;
      if (!normalize_orientation(pose))
      {
        throw std::runtime_error("TF 返回了无效工具位姿");
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        candidate_pose_ = pose;
        pose_initialized_ = true;
        ++candidate_revision_;
        preview_status_ = PreviewStatus::kPending;
      }
      create_interactive_marker(pose);
      publish_preview();
      log_pose("初始工具姿态", pose, candidate_revision_);
      initialization_timer_->cancel();
    }
    catch (const std::exception & exception)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "等待 %s -> %s TF: %s",
        reference_frame_.c_str(), end_effector_link_.c_str(), exception.what());
    }
  }

  void create_interactive_marker(const geometry_msgs::msg::Pose & pose)
  {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = reference_frame_;
    marker.name = kMarkerName;
    marker.description = "push_start_pose";
    marker.pose = pose;
    marker.scale = marker_scale_;

    visualization_msgs::msg::InteractiveMarkerControl body_control;
    body_control.name = "candidate";
    body_control.always_visible = true;
    visualization_msgs::msg::Marker body;
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.scale.x = marker_scale_ * 0.18;
    body.scale.y = marker_scale_ * 0.18;
    body.scale.z = marker_scale_ * 0.18;
    body.color.r = 0.75F;
    body.color.g = 0.75F;
    body.color.b = 0.75F;
    body.color.a = 0.65F;
    body_control.markers.push_back(body);
    marker.controls.push_back(body_control);

    add_axis_controls(marker, "x", 1.0, 0.0, 0.0);
    add_axis_controls(marker, "y", 0.0, 0.0, 1.0);
    add_axis_controls(marker, "z", 0.0, -1.0, 0.0);

    interactive_server_->insert(
      marker,
      std::bind(
        &PushPoseTuner::handle_marker_feedback, this,
        std::placeholders::_1));
    interactive_server_->applyChanges();
  }

  void add_axis_controls(
    visualization_msgs::msg::InteractiveMarker & marker,
    const std::string & axis_name,
    double orientation_x,
    double orientation_y,
    double orientation_z)
  {
    constexpr double component = 0.7071067811865476;
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = component;
    control.orientation.x = component * orientation_x;
    control.orientation.y = component * orientation_y;
    control.orientation.z = component * orientation_z;
    control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.name = "rotate_" + axis_name;
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_" + axis_name;
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
  }

  void handle_marker_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (feedback->marker_name != kMarkerName ||
      (feedback->event_type !=
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE &&
      feedback->event_type !=
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP))
    {
      return;
    }

    auto pose = feedback->pose;
    if (!normalize_orientation(pose))
    {
      geometry_msgs::msg::Pose current_pose;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose = candidate_pose_;
      }
      interactive_server_->setPose(kMarkerName, current_pose);
      interactive_server_->applyChanges();
      RCLCPP_WARN(
        get_logger(), "拒绝交互标记姿态：位置或姿态包含无效数值");
      return;
    }

    std::uint64_t revision = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      candidate_pose_ = pose;
      revision = ++candidate_revision_;
      validated_revision_ = 0U;
      preview_status_ = PreviewStatus::kPending;
    }
    publish_preview();

    if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
    {
      log_pose("候选工具姿态", pose, revision);
      request_planning();
    }
  }

  void handle_nudge(const geometry_msgs::msg::Twist::SharedPtr nudge)
  {
    const double translation_norm = std::sqrt(
      nudge->linear.x * nudge->linear.x +
      nudge->linear.y * nudge->linear.y +
      nudge->linear.z * nudge->linear.z);
    const double rotation_norm = std::sqrt(
      nudge->angular.x * nudge->angular.x +
      nudge->angular.y * nudge->angular.y +
      nudge->angular.z * nudge->angular.z);
    const double maximum_rotation_nudge =
      maximum_rotation_nudge_deg_ * kPi / 180.0;
    if (!std::isfinite(translation_norm) ||
      !std::isfinite(rotation_norm) ||
      translation_norm > maximum_translation_nudge_ ||
      rotation_norm > maximum_rotation_nudge)
    {
      RCLCPP_WARN(
        get_logger(),
        "拒绝步进命令: translation=%.6f/%.6f m, rotation=%.3f/%.3f deg",
        translation_norm, maximum_translation_nudge_,
        radians_to_degrees(rotation_norm), maximum_rotation_nudge_deg_);
      return;
    }

    geometry_msgs::msg::Pose pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pose_initialized_)
      {
        RCLCPP_WARN(get_logger(), "工具 TF 尚未就绪，忽略步进命令");
        return;
      }
      pose = candidate_pose_;
    }
    pose.position.x += nudge->linear.x;
    pose.position.y += nudge->linear.y;
    pose.position.z += nudge->linear.z;

    tf2::Quaternion current_orientation;
    tf2::fromMsg(pose.orientation, current_orientation);
    tf2::Quaternion increment;
    increment.setRPY(
      nudge->angular.x, nudge->angular.y, nudge->angular.z);
    tf2::Quaternion updated_orientation = increment * current_orientation;
    updated_orientation.normalize();
    pose.orientation = tf2::toMsg(updated_orientation);

    if (!normalize_orientation(pose))
    {
      RCLCPP_WARN(get_logger(), "拒绝步进结果：位置或姿态包含无效数值");
      return;
    }

    std::uint64_t revision = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      candidate_pose_ = pose;
      revision = ++candidate_revision_;
      validated_revision_ = 0U;
      preview_status_ = PreviewStatus::kPending;
    }
    interactive_server_->setPose(kMarkerName, pose);
    interactive_server_->applyChanges();
    publish_preview();
    log_pose("步进后工具姿态", pose, revision);
    request_planning();
  }

  void request_planning()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pose_initialized_ || stopping_)
      {
        return;
      }
      planning_requested_ = true;
    }
    planning_condition_.notify_one();
  }

  void planning_loop()
  {
    while (true)
    {
      geometry_msgs::msg::Pose pose;
      std::uint64_t revision = 0U;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        planning_condition_.wait(lock, [this]() {
          return planning_requested_ || stopping_;
        });
        if (stopping_)
        {
          return;
        }
        planning_requested_ = false;
        pose = candidate_pose_;
        revision = candidate_revision_;
        preview_status_ = PreviewStatus::kPlanning;
      }
      publish_preview();

      massage_motion::MotionRequest request;
      request.request_id = "push_pose_tuner_approach";
      request.motion_type = massage_motion::MotionType::kPtp;
      massage_motion::PoseTarget target;
      target.pose.header.frame_id = reference_frame_;
      target.pose.header.stamp = now();
      target.pose.pose = pose;
      request.target = target;
      request.velocity_scale = velocity_scale_;
      request.acceleration_scale = acceleration_scale_;
      request.planning_timeout = planning_timeout_;
      massage_motion::PlanResult result;
      try
      {
        result = planner_->plan(request);
      }
      catch (const std::exception & exception)
      {
        result.success = false;
        result.error = massage_motion::MotionError::kPlanningFailed;
        result.message = std::string{"规划器异常: "} + exception.what();
      }

      bool publish_result = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (candidate_revision_ == revision)
        {
          preview_status_ = result.success ?
            PreviewStatus::kValid : PreviewStatus::kInvalid;
          validated_revision_ = result.success ? revision : 0U;
          publish_result = true;
        }
      }
      if (!publish_result)
      {
        continue;
      }
      publish_preview();
      if (result.success)
      {
        RCLCPP_INFO(
          get_logger(),
          "POSE TUNER PLAN-ONLY: PASS: revision=%llu, planning_time=%.6f s, "
          "points=%zu；未发送运动命令",
          static_cast<unsigned long long>(revision), result.planning_time,
          result.trajectory.joint_trajectory.points.size());
      }
      else
      {
        RCLCPP_WARN(
          get_logger(),
          "POSE TUNER PLAN-ONLY: FAIL: revision=%llu, moveit_error=%d, message=%s",
          static_cast<unsigned long long>(revision), result.moveit_error_code,
          result.message.c_str());
      }
    }
  }

  void publish_preview()
  {
    geometry_msgs::msg::Pose pose;
    PreviewStatus status = PreviewStatus::kPending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pose_initialized_)
      {
        return;
      }
      pose = candidate_pose_;
      status = preview_status_;
    }

    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = reference_frame_;
    stamped_pose.header.stamp = now();
    stamped_pose.pose = pose;
    pose_publisher_->publish(stamped_pose);

    massage_motion::PushPathRequest request;
    request.start_pose = stamped_pose;
    request.direction_x = direction_x_;
    request.direction_y = direction_y_;
    request.length = push_length_;
    request.speed = push_speed_;
    request.sample_period = sample_period_;
    request.maximum_speed = maximum_speed_;
    const auto path_result =
      massage_motion::TechniquePathGenerator::generate_push(request);
    if (!path_result.success)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "推路径预览生成失败: %s", path_result.message.c_str());
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear);

    visualization_msgs::msg::Marker path_marker;
    path_marker.header = stamped_pose.header;
    path_marker.ns = "push_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.006;
    path_marker.color.r = 0.0F;
    path_marker.color.g = 0.8F;
    path_marker.color.b = 0.95F;
    path_marker.color.a = 0.95F;
    for (const auto & point : path_result.path.points)
    {
      path_marker.points.push_back(point.pose.position);
    }
    marker_array.markers.push_back(path_marker);

    visualization_msgs::msg::Marker status_marker;
    status_marker.header = stamped_pose.header;
    status_marker.ns = "candidate_status";
    status_marker.id = 1;
    status_marker.type = visualization_msgs::msg::Marker::SPHERE;
    status_marker.action = visualization_msgs::msg::Marker::ADD;
    status_marker.pose = pose;
    status_marker.scale.x = 0.04;
    status_marker.scale.y = 0.04;
    status_marker.scale.z = 0.04;
    status_marker.color = status_color(status);
    marker_array.markers.push_back(status_marker);

    visualization_msgs::msg::Marker endpoint_marker;
    endpoint_marker.header = stamped_pose.header;
    endpoint_marker.ns = "push_endpoint";
    endpoint_marker.id = 2;
    endpoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
    endpoint_marker.action = visualization_msgs::msg::Marker::ADD;
    endpoint_marker.pose = path_result.path.points.back().pose;
    endpoint_marker.scale.x = 0.025;
    endpoint_marker.scale.y = 0.025;
    endpoint_marker.scale.z = 0.025;
    endpoint_marker.color.r = 0.0F;
    endpoint_marker.color.g = 0.8F;
    endpoint_marker.color.b = 0.95F;
    endpoint_marker.color.a = 0.95F;
    marker_array.markers.push_back(endpoint_marker);
    marker_publisher_->publish(marker_array);
  }

  void confirm_pose(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    geometry_msgs::msg::Pose pose;
    std::uint64_t revision = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pose_initialized_)
      {
        response->message = "工具 TF 尚未就绪";
        return;
      }
      if (preview_status_ != PreviewStatus::kValid ||
        validated_revision_ != candidate_revision_)
      {
        response->message = "当前候选姿态尚未通过对应版本的 PTP 只规划检查";
        return;
      }
      pose = candidate_pose_;
      revision = candidate_revision_;
    }

    std::string error;
    try
    {
      if (!write_yaml(pose, revision, error))
      {
        response->message = error;
        return;
      }
    }
    catch (const std::exception & exception)
    {
      response->message = std::string{"保存 YAML 异常: "} + exception.what();
      return;
    }
    response->success = true;
    response->message = "候选姿态已保存到 " + output_yaml_;
    RCLCPP_INFO(
      get_logger(),
      "PUSH POSE CONFIRMED: revision=%llu, yaml=%s；未发送运动命令",
      static_cast<unsigned long long>(revision), output_yaml_.c_str());
  }

  void reset_pose(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pose_initialized_)
      {
        response->message = "工具 TF 和交互标记尚未就绪";
        return;
      }
    }

    geometry_msgs::msg::Pose pose;
    try
    {
      const auto transform = tf_buffer_->lookupTransform(
        reference_frame_, end_effector_link_, tf2::TimePointZero);
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;
      if (!normalize_orientation(pose))
      {
        throw std::runtime_error("TF 返回了无效工具位姿");
      }
    }
    catch (const std::exception & exception)
    {
      response->message = std::string{"复位读取 TF 失败: "} + exception.what();
      return;
    }

    std::uint64_t revision = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      candidate_pose_ = pose;
      pose_initialized_ = true;
      revision = ++candidate_revision_;
      validated_revision_ = 0U;
      preview_status_ = PreviewStatus::kPending;
    }
    interactive_server_->setPose(kMarkerName, pose);
    interactive_server_->applyChanges();
    publish_preview();
    log_pose("复位后工具姿态", pose, revision);
    response->success = true;
    response->message = "候选姿态已复位到当前工具 TF，等待重新规划";
    request_planning();
  }

  void replan_pose(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pose_initialized_)
      {
        response->message = "工具 TF 尚未就绪";
        return;
      }
      validated_revision_ = 0U;
      preview_status_ = PreviewStatus::kPending;
    }
    publish_preview();
    request_planning();
    response->success = true;
    response->message = "已提交当前候选姿态的只规划检查";
  }

  bool write_yaml(
    const geometry_msgs::msg::Pose & pose,
    std::uint64_t revision,
    std::string & error) const
  {
    const std::filesystem::path output_path{output_yaml_};
    const auto parent = output_path.parent_path();
    std::error_code filesystem_error;
    if (!parent.empty() &&
      !std::filesystem::is_directory(parent, filesystem_error))
    {
      error = filesystem_error ?
        "检查 YAML 输出目录失败: " + filesystem_error.message() :
        "YAML 输出目录不存在: " + parent.string();
      return false;
    }

    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    YAML::Emitter yaml;
    yaml.SetDoublePrecision(15);
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "schema_version" << YAML::Value << 1;
    yaml << YAML::Key << "validation" << YAML::Value << YAML::BeginMap;
    yaml << YAML::Key << "preview_only" << YAML::Value << true;
    yaml << YAML::Key << "approach_ptp_plan_valid" << YAML::Value << true;
    yaml << YAML::Key << "push_path_execution_validated" << YAML::Value << false;
    yaml << YAML::Key << "candidate_revision" << YAML::Value << revision;
    yaml << YAML::EndMap;
    yaml << YAML::Key << "start_pose" << YAML::Value << YAML::BeginMap;
    yaml << YAML::Key << "reference_frame" << YAML::Value << reference_frame_;
    yaml << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
    yaml << YAML::Key << "x" << YAML::Value << pose.position.x;
    yaml << YAML::Key << "y" << YAML::Value << pose.position.y;
    yaml << YAML::Key << "z" << YAML::Value << pose.position.z;
    yaml << YAML::EndMap;
    yaml << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
    yaml << YAML::Key << "x" << YAML::Value << pose.orientation.x;
    yaml << YAML::Key << "y" << YAML::Value << pose.orientation.y;
    yaml << YAML::Key << "z" << YAML::Value << pose.orientation.z;
    yaml << YAML::Key << "w" << YAML::Value << pose.orientation.w;
    yaml << YAML::EndMap;
    yaml << YAML::Key << "rpy_degrees" << YAML::Value << YAML::BeginMap;
    yaml << YAML::Key << "roll" << YAML::Value << radians_to_degrees(roll);
    yaml << YAML::Key << "pitch" << YAML::Value << radians_to_degrees(pitch);
    yaml << YAML::Key << "yaw" << YAML::Value << radians_to_degrees(yaw);
    yaml << YAML::EndMap;
    yaml << YAML::EndMap;
    yaml << YAML::Key << "push_path" << YAML::Value << YAML::BeginMap;
    yaml << YAML::Key << "direction_x" << YAML::Value << direction_x_;
    yaml << YAML::Key << "direction_y" << YAML::Value << direction_y_;
    yaml << YAML::Key << "length" << YAML::Value << push_length_;
    yaml << YAML::Key << "speed" << YAML::Value << push_speed_;
    yaml << YAML::Key << "sample_period" << YAML::Value << sample_period_;
    yaml << YAML::Key << "maximum_speed" << YAML::Value << maximum_speed_;
    yaml << YAML::EndMap;
    yaml << YAML::EndMap;
    if (!yaml.good())
    {
      error = "YAML 序列化失败: " + yaml.GetLastError();
      return false;
    }

    std::ofstream stream(output_yaml_, std::ios::out | std::ios::trunc);
    if (!stream.is_open())
    {
      error = "无法打开 YAML 输出文件: " + output_yaml_;
      return false;
    }
    stream << yaml.c_str() << '\n';
    stream.close();
    if (!stream)
    {
      error = "写入 YAML 输出文件失败: " + output_yaml_;
      return false;
    }
    return true;
  }

  void log_pose(
    const std::string & prefix,
    const geometry_msgs::msg::Pose & pose,
    std::uint64_t revision) const
  {
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
      get_logger(),
      "%s: revision=%llu, xyz=[%.6f %.6f %.6f] m, "
      "rpy=[%.3f %.3f %.3f] deg, quaternion=[%.9f %.9f %.9f %.9f]",
      prefix.c_str(), static_cast<unsigned long long>(revision),
      pose.position.x, pose.position.y, pose.position.z,
      radians_to_degrees(roll), radians_to_degrees(pitch),
      radians_to_degrees(yaw), pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
  }

  std::string reference_frame_;
  std::string end_effector_link_;
  std::string output_yaml_;
  double direction_x_{1.0};
  double direction_y_{0.0};
  double push_length_{0.05};
  double push_speed_{0.01};
  double sample_period_{0.05};
  double maximum_speed_{0.02};
  double planning_timeout_{5.0};
  double velocity_scale_{0.05};
  double acceleration_scale_{0.05};
  double maximum_translation_nudge_{0.02};
  double maximum_rotation_nudge_deg_{10.0};
  double marker_scale_{0.15};

  mutable std::mutex mutex_;
  std::condition_variable planning_condition_;
  bool stopping_{false};
  bool planning_requested_{false};
  bool pose_initialized_{false};
  std::uint64_t candidate_revision_{0U};
  std::uint64_t validated_revision_{0U};
  geometry_msgs::msg::Pose candidate_pose_;
  PreviewStatus preview_status_{PreviewStatus::kPending};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer>
    interactive_server_;
  std::unique_ptr<massage_motion::PtpPlanner> planner_;
  std::thread planning_thread_;
  rclcpp::TimerBase::SharedPtr initialization_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    nudge_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PushPoseTuner>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  int exit_code = 0;
  try
  {
    if (!node->initialize())
    {
      exit_code = 1;
      rclcpp::shutdown();
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(node->get_logger(), "姿态调节器初始化异常: %s", exception.what());
    exit_code = 2;
    rclcpp::shutdown();
  }

  if (executor_thread.joinable())
  {
    executor_thread.join();
  }
  node->stop();
  executor.remove_node(node);
  node.reset();
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
  return exit_code;
}
