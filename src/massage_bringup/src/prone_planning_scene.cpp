#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

using Vec3 = std::array<double, 3>;
using namespace std::chrono_literals;

Vec3 read_vec3(const YAML::Node & node, const std::string & name)
{
    if (!node || !node.IsSequence() || node.size() != 3)
    {
        throw std::runtime_error(name + " must contain three values");
    }
    return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

Vec3 add(const Vec3 & lhs, const Vec3 & rhs)
{
    return {lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]};
}

geometry_msgs::msg::Pose pose_at(const Vec3 & position)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.w = 1.0;
    return pose;
}

moveit_msgs::msg::CollisionObject make_box(
    const std::string & id,
    const Vec3 & center,
    const Vec3 & half_size)
{
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "world";
    object.id = id;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {
        2.0 * half_size[0], 2.0 * half_size[1], 2.0 * half_size[2]};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose_at(center));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
}

geometry_msgs::msg::Pose cylinder_pose(const Vec3 & start, const Vec3 & end)
{
    const Vec3 direction{
        end[0] - start[0], end[1] - start[1], end[2] - start[2]};
    const double length = std::sqrt(
        direction[0] * direction[0] + direction[1] * direction[1] +
        direction[2] * direction[2]);
    if (length <= 0.0)
    {
        throw std::runtime_error("capsule endpoints must differ");
    }

    geometry_msgs::msg::Pose pose = pose_at({
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    });
    const Vec3 unit{
        direction[0] / length, direction[1] / length, direction[2] / length};
    if (unit[2] < -0.999999)
    {
        pose.orientation.x = 1.0;
        pose.orientation.w = 0.0;
        return pose;
    }
    pose.orientation.x = -unit[1];
    pose.orientation.y = unit[0];
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0 + unit[2];
    const double norm = std::sqrt(
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.w * pose.orientation.w);
    pose.orientation.x /= norm;
    pose.orientation.y /= norm;
    pose.orientation.w /= norm;
    return pose;
}

moveit_msgs::msg::CollisionObject make_capsule(
    const std::string & id,
    const Vec3 & start,
    const Vec3 & end,
    double radius)
{
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "world";
    object.id = id;

    const double dx = end[0] - start[0];
    const double dy = end[1] - start[1];
    const double dz = end[2] - start[2];
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
        std::sqrt(dx * dx + dy * dy + dz * dz);
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;
    object.primitives.push_back(cylinder);
    object.primitive_poses.push_back(cylinder_pose(start, end));

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions = {radius};
    object.primitives.push_back(sphere);
    object.primitive_poses.push_back(pose_at(start));
    object.primitives.push_back(sphere);
    object.primitive_poses.push_back(pose_at(end));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
}

std::size_t ensure_acm_entry(
    moveit_msgs::msg::AllowedCollisionMatrix & matrix,
    const std::string & name)
{
    const auto existing = std::find(
        matrix.entry_names.begin(), matrix.entry_names.end(), name);
    if (existing != matrix.entry_names.end())
    {
        return static_cast<std::size_t>(
            std::distance(matrix.entry_names.begin(), existing));
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
    return new_size - 1;
}

class PronePlanningScene
{
public:
    explicit PronePlanningScene(rclcpp::Node::SharedPtr node)
        : node_(std::move(node)),
          get_scene_(node_->create_client<moveit_msgs::srv::GetPlanningScene>(
              "/get_planning_scene")),
          apply_scene_(node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
              "/apply_planning_scene"))
    {
    }

    bool apply()
    {
        std::string config_path = node_->declare_parameter<std::string>(
            "config_path", "");
        if (config_path.empty())
        {
            config_path = ament_index_cpp::get_package_share_directory(
                "massage_mujoco") + "/config/mujoco.yaml";
        }
        const YAML::Node mannequin =
            YAML::LoadFile(config_path)["prone_mannequin"];
        const YAML::Node layout = mannequin["layout"];
        const Vec3 center = read_vec3(mannequin["back_center"], "back_center");
        const Vec3 back_size = read_vec3(mannequin["back_size"], "back_size");
        const double back_radius = mannequin["back_radius"].as<double>();

        std::vector<moveit_msgs::msg::CollisionObject> objects;
        objects.push_back(make_box(
            "prone_back_soft_tissue",
            center,
            {0.5 * back_size[0] + back_radius,
             0.5 * back_size[1] + back_radius,
             0.5 * back_size[2] + back_radius}));
        add_layout_box(objects, layout, center, "massage_table", "massage_table");
        add_layout_box(objects, layout, center, "torso_core", "prone_torso_core");
        add_layout_box(objects, layout, center, "pelvis", "prone_pelvis");
        add_layout_box(objects, layout, center, "head", "prone_head");
        add_limbs(objects, layout["arms"], center, "arm");
        add_limbs(objects, layout["legs"], center, "leg");

        moveit_msgs::msg::AllowedCollisionMatrix matrix;
        if (!read_scene(
                moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX,
                nullptr,
                &matrix))
        {
            return false;
        }
        const std::size_t tool = ensure_acm_entry(matrix, "massage_head_link");
        const std::size_t back = ensure_acm_entry(matrix, "prone_back_soft_tissue");
        matrix.entry_values[tool].enabled[back] = true;
        matrix.entry_values[back].enabled[tool] = true;

        moveit_msgs::msg::PlanningScene scene;
        scene.is_diff = true;
        scene.world.collision_objects = objects;
        scene.allowed_collision_matrix = matrix;
        if (!apply_scene(scene))
        {
            return false;
        }
        return verify(objects);
    }

private:
    static void add_layout_box(
        std::vector<moveit_msgs::msg::CollisionObject> & objects,
        const YAML::Node & layout,
        const Vec3 & center,
        const std::string & config_name,
        const std::string & object_name)
    {
        const YAML::Node item = layout[config_name];
        objects.push_back(make_box(
            object_name,
            add(center, read_vec3(item["center_offset"], config_name + ".center_offset")),
            read_vec3(item["half_size"], config_name + ".half_size")));
    }

    static void add_limbs(
        std::vector<moveit_msgs::msg::CollisionObject> & objects,
        const YAML::Node & limb,
        const Vec3 & center,
        const std::string & limb_name)
    {
        const YAML::Node offsets = limb["x_offsets"];
        if (!offsets || !offsets.IsSequence() || offsets.size() != 2)
        {
            throw std::runtime_error(limb_name + ".x_offsets must contain two values");
        }
        const Vec3 start_offset = read_vec3(
            limb["start_offset"], limb_name + ".start_offset");
        const Vec3 end_offset = read_vec3(
            limb["end_offset"], limb_name + ".end_offset");
        const double radius = limb["radius"].as<double>();
        for (std::size_t index = 0; index < 2; ++index)
        {
            Vec3 start = add(center, start_offset);
            Vec3 end = add(center, end_offset);
            start[0] += offsets[index].as<double>();
            end[0] += offsets[index].as<double>();
            const std::string side = index == 0 ? "left" : "right";
            objects.push_back(make_capsule(
                "prone_" + side + "_" + limb_name, start, end, radius));
        }
    }

    bool read_scene(
        std::uint32_t components,
        std::vector<moveit_msgs::msg::CollisionObject> * objects,
        moveit_msgs::msg::AllowedCollisionMatrix * matrix)
    {
        if (!get_scene_->wait_for_service(30s))
        {
            RCLCPP_ERROR(node_->get_logger(), "等待 /get_planning_scene 超时");
            return false;
        }
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = components;
        auto future = get_scene_->async_send_request(request);
        if (future.wait_for(10s) != std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "读取 MoveIt PlanningScene 超时");
            return false;
        }
        const auto response = future.get();
        if (objects != nullptr)
        {
            *objects = response->scene.world.collision_objects;
        }
        if (matrix != nullptr)
        {
            *matrix = response->scene.allowed_collision_matrix;
        }
        return true;
    }

    bool apply_scene(const moveit_msgs::msg::PlanningScene & scene)
    {
        if (!apply_scene_->wait_for_service(30s))
        {
            RCLCPP_ERROR(node_->get_logger(), "等待 /apply_planning_scene 超时");
            return false;
        }
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene = scene;
        auto future = apply_scene_->async_send_request(request);
        if (future.wait_for(10s) != std::future_status::ready ||
            !future.get()->success)
        {
            RCLCPP_ERROR(node_->get_logger(), "写入 MoveIt PlanningScene 失败");
            return false;
        }
        return true;
    }

    bool verify(const std::vector<moveit_msgs::msg::CollisionObject> & expected)
    {
        std::vector<moveit_msgs::msg::CollisionObject> objects;
        moveit_msgs::msg::AllowedCollisionMatrix matrix;
        if (!read_scene(
                moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX,
                &objects,
                &matrix))
        {
            return false;
        }
        std::set<std::string> ids;
        for (const auto & object : objects)
        {
            ids.insert(object.id);
        }
        for (const auto & object : expected)
        {
            if (ids.count(object.id) == 0)
            {
                RCLCPP_ERROR(
                    node_->get_logger(), "PlanningScene 缺少碰撞体 %s",
                    object.id.c_str());
                return false;
            }
        }
        const auto tool = std::find(
            matrix.entry_names.begin(), matrix.entry_names.end(), "massage_head_link");
        const auto back = std::find(
            matrix.entry_names.begin(), matrix.entry_names.end(), "prone_back_soft_tissue");
        if (tool == matrix.entry_names.end() || back == matrix.entry_names.end())
        {
            RCLCPP_ERROR(node_->get_logger(), "PlanningScene 接触白名单缺少按摩头或背部");
            return false;
        }
        const auto tool_index = static_cast<std::size_t>(
            std::distance(matrix.entry_names.begin(), tool));
        const auto back_index = static_cast<std::size_t>(
            std::distance(matrix.entry_names.begin(), back));
        if (!matrix.entry_values[tool_index].enabled[back_index] ||
            !matrix.entry_values[back_index].enabled[tool_index])
        {
            RCLCPP_ERROR(node_->get_logger(), "按摩头与背部接触白名单未生效");
            return false;
        }
        RCLCPP_INFO(
            node_->get_logger(),
            "已向 MoveIt 注入 %zu 个俯卧假人/按摩床碰撞体，仅允许按摩头接触背部",
            expected.size());
        return true;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
};

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("prone_planning_scene");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });
    int exit_code = 1;
    try
    {
        exit_code = PronePlanningScene(node).apply() ? 0 : 1;
    }
    catch (const std::exception & error)
    {
        RCLCPP_ERROR(node->get_logger(), "创建俯卧假人 PlanningScene 失败: %s", error.what());
    }
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return exit_code;
}
