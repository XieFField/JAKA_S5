#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/moveit_trajectory_executor.hpp"
#include "massage_motion/ros2_control_compliance_controller.hpp"
#include "massage_task/press_task_state_machine.hpp"

namespace
{

class MoveItContactSceneManager final :
    public massage_task::IContactSceneManager
{
public:
    explicit MoveItContactSceneManager(rclcpp::Node::SharedPtr node)
        : node_(std::move(node)),
          get_scene_client_(
              node_->create_client<moveit_msgs::srv::GetPlanningScene>(
                  "/get_planning_scene")),
          apply_scene_client_(
              node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
                  "/apply_planning_scene"))
    {
    }

    bool prepare() override
    {
        if (!read_allowed_collision_matrix(original_matrix_))
        {
            return false;
        }

        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = "world";
        object.id = object_id_;

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {0.20, 0.10, 0.10};
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.75;
        pose.position.y = -0.15;
        pose.position.z = 0.106382;
        pose.orientation.w = 1.0;
        object.primitives.push_back(box);
        object.primitive_poses.push_back(pose);
        object.operation = moveit_msgs::msg::CollisionObject::ADD;
        prepared_ = planning_scene_interface_.applyCollisionObject(object);
        return prepared_;
    }

    bool allow_tool_contact(bool allowed) override
    {
        if (!prepared_)
        {
            return false;
        }

        moveit_msgs::msg::AllowedCollisionMatrix matrix;
        if (!read_allowed_collision_matrix(matrix))
        {
            return false;
        }
        ensure_entry(matrix, tool_link_);
        ensure_entry(matrix, object_id_);
        const auto tool_index = index_of(matrix, tool_link_);
        const auto object_index = index_of(matrix, object_id_);
        matrix.entry_values[tool_index].enabled[object_index] = allowed;
        matrix.entry_values[object_index].enabled[tool_index] = allowed;
        return apply_allowed_collision_matrix(matrix);
    }

    bool restore() override
    {
        if (!prepared_)
        {
            return true;
        }
        const bool matrix_restored =
            apply_allowed_collision_matrix(original_matrix_);
        moveit_msgs::msg::CollisionObject remove_object;
        remove_object.header.frame_id = "world";
        remove_object.id = object_id_;
        remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        const bool object_removed =
            planning_scene_interface_.applyCollisionObject(remove_object);
        prepared_ = false;
        return matrix_restored && object_removed;
    }

private:
    static std::size_t index_of(
        const moveit_msgs::msg::AllowedCollisionMatrix & matrix,
        const std::string & name)
    {
        return static_cast<std::size_t>(std::distance(
            matrix.entry_names.begin(),
            std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name)));
    }

    static void ensure_entry(
        moveit_msgs::msg::AllowedCollisionMatrix & matrix,
        const std::string & name)
    {
        if (std::find(
                matrix.entry_names.begin(), matrix.entry_names.end(), name) !=
            matrix.entry_names.end())
        {
            return;
        }

        const std::size_t new_size = matrix.entry_names.size() + 1;
        matrix.entry_names.push_back(name);
        for (auto & entry : matrix.entry_values)
        {
            entry.enabled.resize(new_size, false);
        }
        moveit_msgs::msg::AllowedCollisionEntry entry;
        entry.enabled.resize(new_size, false);
        matrix.entry_values.push_back(entry);
    }

    bool read_allowed_collision_matrix(
        moveit_msgs::msg::AllowedCollisionMatrix & matrix)
    {
        const auto timeout = std::chrono::seconds(3);
        if (!get_scene_client_->wait_for_service(timeout))
        {
            RCLCPP_ERROR(node_->get_logger(), "等待 /get_planning_scene 超时");
            return false;
        }
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components =
            moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        auto future = get_scene_client_->async_send_request(request);
        if (future.wait_for(timeout) != std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "读取 Allowed Collision Matrix 超时");
            return false;
        }
        matrix = future.get()->scene.allowed_collision_matrix;
        return true;
    }

    bool apply_allowed_collision_matrix(
        const moveit_msgs::msg::AllowedCollisionMatrix & matrix)
    {
        const auto timeout = std::chrono::seconds(3);
        if (!apply_scene_client_->wait_for_service(timeout))
        {
            RCLCPP_ERROR(node_->get_logger(), "等待 /apply_planning_scene 超时");
            return false;
        }
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene.is_diff = true;
        request->scene.allowed_collision_matrix = matrix;
        auto future = apply_scene_client_->async_send_request(request);
        return future.wait_for(timeout) == std::future_status::ready &&
            future.get()->success;
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr
        get_scene_client_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr
        apply_scene_client_;
    moveit_msgs::msg::AllowedCollisionMatrix original_matrix_;
    const std::string object_id_{"contact_pad"};
    const std::string tool_link_{"massage_head_link"};
    bool prepared_{false};
};

massage_motion::PoseTarget default_contact_target()
{
    massage_motion::PoseTarget target;
    target.pose.header.frame_id = "world";
    target.pose.pose.position.x = 0.75;
    // contact_pad 的 +Y 表面为 -0.10 m；目标描述的是按摩头端面中心。
    target.pose.pose.position.y = -0.1005;
    target.pose.pose.position.z = 0.106382;
    target.pose.pose.orientation.x = 0.707108079859;
    target.pose.pose.orientation.y = -0.000000008619;
    target.pose.pose.orientation.z = 0.000000008627;
    target.pose.pose.orientation.w = 0.707105482511;
    return target;
}

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "compliant_press_task_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::MultiThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    std::thread spin_thread([&ros_executor]() {ros_executor.spin();});

    int exit_code = 1;
    try
    {
        double approach_distance = 0.04;
        double precontact_distance = 0.0055;
        double force_limit = 1.0;
        double contact_threshold = 0.1;
        double hold_duration = 2.0;
        bool expect_limit_exceeded = false;
        node->get_parameter_or("approach_distance", approach_distance, 0.04);
        node->get_parameter_or("precontact_distance", precontact_distance, 0.0055);
        node->get_parameter_or("force_limit", force_limit, 1.0);
        node->get_parameter_or("contact_threshold", contact_threshold, 0.1);
        node->get_parameter_or("hold_duration", hold_duration, 2.0);
        node->get_parameter_or(
            "expect_limit_exceeded", expect_limit_exceeded, false);

        massage_motion::PlannerConfig planner_config;
        planner_config.planning_group = "jaka_s5";
        planner_config.end_effector_link = "massage_tool_tip";
        planner_config.reference_frame = "world";
        planner_config.planning_pipeline = "pilz_industrial_motion_planner";

        auto planner = std::make_shared<massage_motion::MotionPlanningSdk>(
            node, planner_config);
        auto trajectory_executor =
            std::make_shared<massage_motion::MoveItTrajectoryExecutor>(node);

        massage_motion::Ros2ControlComplianceConfig compliance_config;
        compliance_config.joint_names = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"};
        auto compliance =
            std::make_shared<massage_motion::Ros2ControlComplianceController>(
                node, compliance_config);
        auto scene_manager = std::make_shared<MoveItContactSceneManager>(node);

        massage_task::PressTaskRequest request;
        request.task_id = "simulated_press_001";
        request.contact_target = default_contact_target();
        request.surface_normal.y = 1.0;
        request.approach_distance = approach_distance;
        request.precontact_distance = precontact_distance;
        request.contact_threshold = contact_threshold;
        request.maximum_contact_wrench = force_limit;
        request.hold_duration = hold_duration;
        request.compliance_request.request_id = "simulated_press_compliance";
        request.compliance_request.enabled_axes[2] = true;
        request.compliance_request.max_absolute_wrench[2] = force_limit;
        request.compliance_request.max_joint_displacement = 0.20;
        request.compliance_request.max_linear_displacement = 0.03;
        request.compliance_request.timeout = 20.0;

        massage_task::PressTaskStateMachine state_machine(
            planner, trajectory_executor, compliance, scene_manager);

        RCLCPP_INFO(
            node->get_logger(),
            "开始按压状态机: approach=%.3f m, precontact=%.4f m, "
            "contact=%.3f N, limit=%.3f N",
            approach_distance,
            precontact_distance,
            contact_threshold,
            force_limit);
        const auto result = state_machine.run(request);
        const bool expected_limit_result =
            expect_limit_exceeded &&
            !result.success &&
            result.final_state == massage_task::PressTaskState::kFault &&
            result.primary_error ==
                massage_task::PressTaskError::kComplianceFailed &&
            result.recovery_succeeded;
        if (!result.success && !expected_limit_result)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "按压任务失败: error=%d, primary=%d, message=%s, recovery=%s",
                static_cast<int>(result.error),
                static_cast<int>(result.primary_error),
                result.message.c_str(),
                result.recovery_message.c_str());
        }
        else if (result.success)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "按压任务完成: contact=%s, peak=%.6f N, recovery=%s",
                result.contact_detected ? "true" : "false",
                result.peak_absolute_wrench,
                result.recovery_message.c_str());
            exit_code = 0;
        }
        else
        {
            RCLCPP_INFO(
                node->get_logger(),
                "预期超限验收通过: peak=%.6f N, compliance_error=%d, recovery=%s",
                result.peak_absolute_wrench,
                static_cast<int>(result.compliance_result.error),
                result.recovery_message.c_str());
            exit_code = 0;
        }
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(node->get_logger(), "按压 demo 异常: %s", exception.what());
    }

    ros_executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}
