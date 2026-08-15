#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "massage_motion/motion_planning_sdk.hpp"
#include "massage_motion/motion_types.hpp"
using namespace massage_motion;

PlannerConfig make_planner_config();

MotionRequest make_ptp_request();
MotionRequest make_lin_request();
MotionRequest make_circ_request();
MotionRequest make_ptp_pose_request();

bool report_result(
    const rclcpp::Logger & logger,
    const std::string & stage_name,
    const PlanResult & result);

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "motion_planning_sdk_demo",
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

    auto planner_config = make_planner_config();

    MotionPlanningSdk sdk(node, planner_config);

    MotionRequest ptp_request = make_ptp_request();
    MotionRequest lin_request = make_lin_request();
    MotionRequest circ_request = make_circ_request();
    MotionRequest ptp_pose_request = make_ptp_pose_request();
    MotionRequest circ_request_2 = make_circ_request();
    circ_request_2.request_id = "circ_demo_002";

    auto ptp_plan_result = sdk.plan(ptp_request);
    auto ptp_pose_plan_result = sdk.plan(ptp_pose_request);
    if(!report_result(node->get_logger(), "PTP", ptp_plan_result))
    {
        exit_code = 1;
    }

    if(!report_result(node->get_logger(), "PTP_POSE", ptp_pose_plan_result))
    {
        exit_code = 1;
    }

    auto lin_plan_result = sdk.plan(lin_request);
    if(!report_result(node->get_logger(), "LIN", lin_plan_result))
    {
        exit_code = 1;
    }

    auto circ_plan_result = sdk.plan(circ_request);
    auto circ_plan_result_2 = sdk.plan(circ_request_2);
    if(!report_result(node->get_logger(), "CIRC", circ_plan_result))
    {
        exit_code = 1;
    }

    if(!report_result(node->get_logger(), "CIRC_2", circ_plan_result_2))
    {
        exit_code = 1;
    }

    executor.cancel();
    if(executor_thread.joinable())
    {
        executor_thread.join();
    }
    rclcpp::shutdown();

    return exit_code;
}

PlannerConfig make_planner_config()
{
    PlannerConfig planner_config;
    planner_config.planning_group = "jaka_s5";
    planner_config.end_effector_link = "Link_06";
    planner_config.reference_frame = "world";
    planner_config.planning_pipeline = "pilz_industrial_motion_planner";

    return planner_config;
}

MotionRequest make_ptp_request()
{
    MotionRequest request;
    request.motion_type = MotionType::kPtp;
    request.request_id = "ptp_demo_001";

    JointTarget joint_target;
    joint_target.positions = {
        0.0,
        1.407,
        -1.5707,
        1.5707,
        1.5707,
        0.0
    };

    request.target = joint_target;

    return request;
}

MotionRequest make_lin_request()
{
    MotionRequest request;
    request.motion_type = MotionType::kLin;
    request.request_id = "lin_demo_001";

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

    return request;
}

MotionRequest make_circ_request()
{
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

    return request;
}

MotionRequest make_ptp_pose_request()
{
    MotionRequest request;
    request.motion_type = MotionType::kPtp;
    request.request_id = "ptp_pose_demo_001";

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

    return request;
}

bool report_result(
    const rclcpp::Logger & logger,
    const std::string & stage_name,
    const PlanResult & result)
{
    if(!result.success)
    {
        RCLCPP_ERROR(
            logger, 
            "%s规划失败，错误码: %d, moveit错误码: %d, 消息: %s", 
            stage_name.c_str(),
            static_cast<int>(result.error),
            result.moveit_error_code,
            result.message.c_str()
        );
        return false;
    }
    else
    {
        RCLCPP_INFO(
            logger, 
            "%s规划成功,规划时间: %.3f 秒，轨迹点数: %zu",
            stage_name.c_str(),
            result.planning_time,
            result.trajectory.joint_trajectory.points.size()
        );
        return true;
    }
}
