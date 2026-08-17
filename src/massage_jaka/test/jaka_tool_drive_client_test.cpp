#include <array>
#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "jaka_msgs/srv/get_tool_drive_state.hpp"
#include "jaka_msgs/srv/set_tool_drive_config.hpp"
#include "jaka_msgs/srv/set_tool_drive_frame.hpp"
#include "jaka_msgs/srv/set_tool_drive_tuning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "massage_jaka/jaka_tool_drive_client.hpp"

TEST(JakaToolDriveClientTest, ConfiguresStartsReadsAndStops)
{
    if (!rclcpp::ok())
    {
        int argc = 0;
        rclcpp::init(argc, nullptr);
    }
    auto node = std::make_shared<rclcpp::Node>("jaka_tool_drive_client_test");
    std::array<std::int32_t, 6> options{};
    std::array<double, 6> rebound{};
    std::array<double, 6> rigidity{};
    std::int32_t frame = 0;
    std::int32_t sensitivity_level = 3;
    std::int32_t warning_range = 2;
    std::size_t tuning_calls = 0;
    bool enabled = false;
    bool ignore_next_enable = false;
    std::size_t disable_calls = 0;

    auto config_service =
        node->create_service<jaka_msgs::srv::SetToolDriveConfig>(
        "/test_tool_drive/config",
        [&](const jaka_msgs::srv::SetToolDriveConfig::Request::SharedPtr request,
            jaka_msgs::srv::SetToolDriveConfig::Response::SharedPtr response)
        {
            const auto axis = static_cast<std::size_t>(request->axis);
            options[axis] = request->option;
            rebound[axis] = request->rebound;
            rigidity[axis] = request->rigidity;
            response->success = true;
        });
    auto frame_service =
        node->create_service<jaka_msgs::srv::SetToolDriveFrame>(
        "/test_tool_drive/frame",
        [&](const jaka_msgs::srv::SetToolDriveFrame::Request::SharedPtr request,
            jaka_msgs::srv::SetToolDriveFrame::Response::SharedPtr response)
        {
            frame = request->frame;
            response->success = true;
        });
    auto tuning_service =
        node->create_service<jaka_msgs::srv::SetToolDriveTuning>(
        "/test_tool_drive/tuning",
        [&](const jaka_msgs::srv::SetToolDriveTuning::Request::SharedPtr request,
            jaka_msgs::srv::SetToolDriveTuning::Response::SharedPtr response)
        {
            sensitivity_level = request->sensitivity_level;
            warning_range = request->warning_range;
            ++tuning_calls;
            response->success = true;
        });
    auto state_service =
        node->create_service<jaka_msgs::srv::GetToolDriveState>(
        "/test_tool_drive/state",
        [&](const jaka_msgs::srv::GetToolDriveState::Request::SharedPtr,
            jaka_msgs::srv::GetToolDriveState::Response::SharedPtr response)
        {
            response->success = true;
            response->enabled = enabled;
            response->frame = frame;
            response->sensitivity_level = sensitivity_level;
            response->warning_range = warning_range;
            response->control_owner = enabled ? "tool_drive" : "idle";
            response->axis_options = options;
            response->rebound = rebound;
            response->rigidity = rigidity;
        });
    auto enable_service = node->create_service<std_srvs::srv::SetBool>(
        "/test_tool_drive/enable",
        [&](const std_srvs::srv::SetBool::Request::SharedPtr request,
            std_srvs::srv::SetBool::Response::SharedPtr response)
        {
            if (request->data && ignore_next_enable)
            {
                ignore_next_enable = false;
            }
            else
            {
                enabled = request->data;
            }
            if (!request->data)
            {
                ++disable_calls;
            }
            response->success = true;
        });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});

    massage_jaka::JakaToolDriveConfig config;
    config.config_service = "/test_tool_drive/config";
    config.frame_service = "/test_tool_drive/frame";
    config.tuning_service = "/test_tool_drive/tuning";
    config.state_service = "/test_tool_drive/state";
    config.enable_service = "/test_tool_drive/enable";
    config.axis = 2;
    config.frame = 0;
    config.rebound = 0.2;
    config.rigidity = 0.3;
    config.preserve_existing_axis_parameters = false;
    config.service_timeout = 1.0;
    massage_jaka::JakaToolDriveClient client(node, config);

    std::string message;
    EXPECT_TRUE(client.configure(message));
    EXPECT_EQ(options[2], 1);
    EXPECT_DOUBLE_EQ(rebound[2], 0.2);
    EXPECT_DOUBLE_EQ(rigidity[2], 0.3);
    EXPECT_EQ(tuning_calls, 0U);
    EXPECT_EQ(client.state().sensitivity_level, 3);
    EXPECT_EQ(client.state().warning_range, 2);
    EXPECT_TRUE(client.start(message));
    EXPECT_TRUE(client.state().enabled);
    EXPECT_TRUE(client.stop(message));
    EXPECT_FALSE(client.state().enabled);

    auto preserve_config = config;
    preserve_config.rebound = 9.0;
    preserve_config.rigidity = 9.0;
    preserve_config.preserve_existing_axis_parameters = true;
    massage_jaka::JakaToolDriveClient preserving_client(
        node, preserve_config);
    EXPECT_TRUE(preserving_client.configure(message));
    EXPECT_DOUBLE_EQ(rebound[2], 0.2);
    EXPECT_DOUBLE_EQ(rigidity[2], 0.3);

    auto sensitivity_config = config;
    sensitivity_config.preserve_existing_sensitivity = false;
    sensitivity_config.sensitivity_level = 1;
    sensitivity_config.preserve_existing_warning_range = true;
    sensitivity_config.warning_range = 5;
    massage_jaka::JakaToolDriveClient sensitivity_client(
        node, sensitivity_config);
    EXPECT_TRUE(sensitivity_client.configure(message));
    EXPECT_EQ(tuning_calls, 1U);
    EXPECT_EQ(sensitivity_level, 1);
    EXPECT_EQ(warning_range, 2);
    EXPECT_TRUE(sensitivity_client.start(message));
    EXPECT_TRUE(sensitivity_client.stop(message));

    const auto disables_before_failed_start = disable_calls;
    ignore_next_enable = true;
    EXPECT_FALSE(sensitivity_client.start(message));
    EXPECT_NE(message.find("启用失败清理"), std::string::npos);
    EXPECT_EQ(disable_calls, disables_before_failed_start + 1U);
    EXPECT_FALSE(sensitivity_client.state().enabled);

    executor.cancel();
    spin_thread.join();
    (void)config_service;
    (void)frame_service;
    (void)tuning_service;
    (void)state_service;
    (void)enable_service;
    rclcpp::shutdown();
}
