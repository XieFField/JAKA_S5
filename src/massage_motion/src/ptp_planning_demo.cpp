#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "massage_motion/motion_types.hpp"
#include "massage_motion/ptp_planner.hpp"

using namespace massage_motion;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "ptp_planning_demo",
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

        PtpPlanner ptp_planner(node, planner_config);

        MotionRequest request;

        // 设置关节目标
        request.motion_type = MotionType::kPtp;
        request.request_id = "ptp_demo_001";
        // JointTarget joint_target;

        // joint_target.positions = {
        //     0.0,
        //     1.407,
        //     -1.5707,
        //     1.5707,
        //     1.5707,
        //     0.0
        // };

        // request.target = joint_target;

        PoseTarget pose_target;
        pose_target.pose.header.frame_id = "world";
        pose_target.pose.pose.position.x = 0.798761369855;
        pose_target.pose.pose.position.y = -0.003724931204;
        pose_target.pose.pose.position.z = 0.106382015840;

        pose_target.pose.pose.orientation.x = 0.707108079859;
        pose_target.pose.pose.orientation.y = -0.000000008619;
        pose_target.pose.pose.orientation.z = 0.000000008627;
        pose_target.pose.pose.orientation.w = 0.707105482511;

        request.target = pose_target;
        auto plan_result = ptp_planner.plan(request);

        if(plan_result.success)
        {
            RCLCPP_INFO(
                node->get_logger(),
                "PTP规划成功，规划时间: %.3f 秒，轨迹点数: %zu",
                plan_result.planning_time,
                plan_result.trajectory.joint_trajectory.points.size()
            );
        }
        else
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "PTP规划失败，错误码: %d, moveit错误码: %d, 消息: %s",
                static_cast<int>(plan_result.error),
                plan_result.moveit_error_code,
                plan_result.message.c_str()
            );
            exit_code = 1;
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
