#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "gtest/gtest.h"
#include "jaka_msgs/srv/set_admittance_config.hpp"
#include "jaka_msgs/srv/set_torque_sensor_soft_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "massage_jaka/jaka_compliance_controller.hpp"

namespace
{

using namespace std::chrono_literals;

class JakaComplianceControllerTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }
};

massage_motion::ComplianceRequest make_request()
{
    massage_motion::ComplianceRequest request;
    request.request_id = "fake_jaka_compliance";
    request.enabled_axes[2] = true;
    request.max_absolute_wrench = {5.0, 5.0, 5.0, 1.0, 1.0, 1.0};
    request.max_joint_displacement = 0.1;
    request.max_linear_displacement = 0.1;
    request.timeout = 1.0;
    return request;
}

TEST_F(JakaComplianceControllerTest, ConfiguresStartsMonitorsStopsAndResets)
{
    auto adapter_node = std::make_shared<rclcpp::Node>(
        "jaka_compliance_adapter_test");
    auto backend_node = std::make_shared<rclcpp::Node>(
        "jaka_compliance_backend_test");

    std::atomic<int> soft_limit_calls{0};
    std::atomic<int> config_calls{0};
    std::mutex enable_mutex;
    std::vector<bool> enable_requests;
    auto soft_limit_service = backend_node->create_service<
        jaka_msgs::srv::SetTorqueSensorSoftLimit>(
        "/test_jaka/set_ft_soft_limit",
        [&soft_limit_calls](
            const std::shared_ptr<
                jaka_msgs::srv::SetTorqueSensorSoftLimit::Request>,
            std::shared_ptr<
                jaka_msgs::srv::SetTorqueSensorSoftLimit::Response> response)
        {
            ++soft_limit_calls;
            response->success = true;
        });
    auto config_service = backend_node->create_service<
        jaka_msgs::srv::SetAdmittanceConfig>(
        "/test_jaka/set_admittance_config",
        [&config_calls](
            const std::shared_ptr<jaka_msgs::srv::SetAdmittanceConfig::Request>,
            std::shared_ptr<jaka_msgs::srv::SetAdmittanceConfig::Response> response)
        {
            ++config_calls;
            response->success = true;
        });
    auto enable_service = backend_node->create_service<std_srvs::srv::SetBool>(
        "/test_jaka/enable_admittance",
        [&enable_mutex, &enable_requests](
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            {
                std::lock_guard<std::mutex> lock(enable_mutex);
                enable_requests.push_back(request->data);
            }
            response->success = true;
            response->message = request->data ? "fake enabled" : "fake disabled";
        });

    auto wrench_publisher = backend_node->create_publisher<
        geometry_msgs::msg::WrenchStamped>(
        "/test_jaka/wrench", rclcpp::SensorDataQoS());
    auto joint_publisher = backend_node->create_publisher<sensor_msgs::msg::JointState>(
        "/test_jaka/joint_states", rclcpp::SensorDataQoS());
    tf2_ros::StaticTransformBroadcaster static_broadcaster(backend_node);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = backend_node->now();
    transform.header.frame_id = "test_world";
    transform.child_frame_id = "test_tool";
    transform.transform.rotation.w = 1.0;
    static_broadcaster.sendTransform(transform);

    massage_jaka::JakaComplianceConfig config;
    config.wrench_topic = "/test_jaka/wrench";
    config.soft_limit_service = "/test_jaka/set_ft_soft_limit";
    config.config_service = "/test_jaka/set_admittance_config";
    config.enable_service = "/test_jaka/enable_admittance";
    config.joint_state_topic = "/test_jaka/joint_states";
    config.wrench_frame = "test_ft";
    config.base_frame = "test_world";
    config.tool_frame = "test_tool";
    config.service_timeout = 0.5;
    config.feedback_timeout = 0.2;
    config.state_timeout = 0.2;
    config.monitor_period = 0.005;
    auto controller = std::make_shared<massage_jaka::JakaComplianceController>(
        adapter_node, config);

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);
    executor.add_node(adapter_node);
    executor.add_node(backend_node);
    std::thread spin_thread([&executor]() {executor.spin();});
    const auto shutdown = [&]()
        {
            if (controller->status() ==
                massage_motion::ComplianceStatus::kActive)
            {
                controller->stop();
            }
            if (controller->status() != massage_motion::ComplianceStatus::kIdle)
            {
                controller->reset();
            }
            controller.reset();
            executor.cancel();
            if (spin_thread.joinable())
            {
                spin_thread.join();
            }
        };

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = backend_node->now();
    joint_state.name.assign(config.joint_names.begin(), config.joint_names.end());
    joint_state.position.assign(config.joint_names.size(), 0.0);
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = backend_node->now();
    wrench.header.frame_id = config.wrench_frame;
    wrench.wrench.force.z = 0.2;

    bool feedback_ready = false;
    const auto feedback_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < feedback_deadline)
    {
        joint_publisher->publish(joint_state);
        wrench_publisher->publish(wrench);
        if (!controller->feedback().stale)
        {
            feedback_ready = true;
            break;
        }
        std::this_thread::sleep_for(10ms);
    }
    if (!feedback_ready)
    {
        shutdown();
        FAIL() << "未收到新鲜的假关节和 FT 反馈";
        return;
    }

    wrench.wrench.force.z = std::numeric_limits<double>::quiet_NaN();
    const auto invalid_feedback_deadline =
        std::chrono::steady_clock::now() + 250ms;
    while (std::chrono::steady_clock::now() < invalid_feedback_deadline)
    {
        joint_publisher->publish(joint_state);
        wrench_publisher->publish(wrench);
        std::this_thread::sleep_for(10ms);
    }
    EXPECT_TRUE(controller->feedback().stale);

    wrench.wrench.force.z = 0.2;
    feedback_ready = false;
    const auto recovery_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < recovery_deadline)
    {
        joint_publisher->publish(joint_state);
        wrench_publisher->publish(wrench);
        if (!controller->feedback().stale)
        {
            feedback_ready = true;
            break;
        }
        std::this_thread::sleep_for(10ms);
    }
    if (!feedback_ready)
    {
        shutdown();
        FAIL() << "有效 FT 反馈恢复后适配器仍报告过期";
        return;
    }
    // 为控制器内部 TF listener 留出接收静态变换的时间。
    std::this_thread::sleep_for(50ms);

    const auto start_result = controller->start(make_request());
    if (!start_result.success)
    {
        shutdown();
        FAIL() << start_result.message;
        return;
    }
    EXPECT_EQ(
        controller->status(),
        massage_motion::ComplianceStatus::kActive);
    massage_motion::ComplianceReference invalid_reference;
    invalid_reference.joint_names = {"joint_1"};
    invalid_reference.positions = {0.0};
    EXPECT_FALSE(controller->update_reference(invalid_reference));

    massage_motion::ComplianceReference valid_reference;
    valid_reference.joint_names.assign(
        config.joint_names.begin(), config.joint_names.end());
    valid_reference.positions.assign(config.joint_names.size(), 0.0);
    EXPECT_TRUE(controller->update_reference(valid_reference));

    const auto stop_result = controller->stop();
    EXPECT_TRUE(stop_result.success) << stop_result.message;
    EXPECT_EQ(
        controller->status(),
        massage_motion::ComplianceStatus::kStopped);
    EXPECT_EQ(soft_limit_calls.load(), 1);
    EXPECT_EQ(config_calls.load(), 6);
    {
        std::lock_guard<std::mutex> lock(enable_mutex);
        EXPECT_EQ(enable_requests.size(), 2U);
        if (enable_requests.size() >= 2U)
        {
            EXPECT_TRUE(enable_requests[0]);
            EXPECT_FALSE(enable_requests[1]);
        }
    }
    EXPECT_TRUE(controller->reset());
    EXPECT_EQ(controller->status(), massage_motion::ComplianceStatus::kIdle);

    shutdown();
    (void)soft_limit_service;
    (void)config_service;
    (void)enable_service;
}

}  // namespace
