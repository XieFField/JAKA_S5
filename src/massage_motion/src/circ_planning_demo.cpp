#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "massage_motion/motion_types.hpp"
#include "massage_motion/circ_planner.hpp"

using namespace massage_motion;
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "circ_planning_demo",
        rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // 在单独的线程中运行执行器
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    int exit_code = 0;
{
    PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "Link_06";
    planner_config.reference_frame = "world";
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";

    CircPlanner circ_planner(node, planner_config);

    MotionRequest request;
    request.motion_type = MotionType::kCirc;
    request.request_id = "circ_demo_001";

    request.velocity_scale = 0.05;
    request.acceleration_scale = 0.05;

    CircularTarget circular_target;

    circular_target.interim_pose.header.frame_id = "world";
    circular_target.interim_pose.pose.position.x = 0.748761369855;
    circular_target.interim_pose.pose.position.y = -0.003724931204;
    circular_target.interim_pose.pose.position.z = 0.056382015840;
    circular_target.interim_pose.pose.orientation.w = 1.0;

    circular_target.goal_pose.header.frame_id = "world";
    circular_target.goal_pose.pose.position.x = 0.698761369855;
    circular_target.goal_pose.pose.position.y = -0.003724931204;
    circular_target.goal_pose.pose.position.z = 0.006382015840;

    circular_target.goal_pose.pose.orientation.x = 0.707108079859;
    circular_target.goal_pose.pose.orientation.y = -0.000000008619;
    circular_target.goal_pose.pose.orientation.z = 0.000000008627;
    circular_target.goal_pose.pose.orientation.w = 0.707105482511;

    request.target = circular_target;

    auto result = circ_planner.plan(request);

    if(!result.success)
    {
        RCLCPP_ERROR(
            node->get_logger(), 
            "CIRC规划失败，错误码: %d, moveit错误码: %d, 消息: %s", 
            static_cast<int>(result.error),
            result.moveit_error_code,
            result.message.c_str()
        );
        exit_code = 1;
    }
    else
    {
        RCLCPP_INFO(
            node->get_logger(), 
            "CIRC规划成功,规划时间: %.3f 秒，轨迹点数: %zu",
            result.planning_time,
            result.trajectory.joint_trajectory.points.size()
        );
    }
}

    executor.cancel();
    if(executor_thread.joinable())
    {
        executor_thread.join();
    }
    rclcpp::shutdown();
    return exit_code;
}