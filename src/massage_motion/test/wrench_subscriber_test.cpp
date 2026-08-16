#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "massage_motion/wrench_subscriber.hpp"

namespace massage_motion
{
namespace
{

using namespace std::chrono_literals;

class WrenchSubscriberTest : public ::testing::Test
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

TEST_F(WrenchSubscriberTest, RejectsInvalidConfiguration)
{
    auto node = std::make_shared<rclcpp::Node>(
        "wrench_subscriber_invalid_config_test");

    WrenchSubscriberConfig config;
    config.stale_timeout = 0.1;
    EXPECT_THROW(WrenchSubscriber reader(node, config), std::invalid_argument);

    config.topic_name = "/test/wrench";
    config.stale_timeout = 0.0;
    EXPECT_THROW(WrenchSubscriber reader(node, config), std::invalid_argument);
}

TEST_F(WrenchSubscriberTest, ReadsValidSamplesAndRejectsInvalidSamples)
{
    auto subscriber_node = std::make_shared<rclcpp::Node>(
        "wrench_subscriber_reader_test");
    auto publisher_node = std::make_shared<rclcpp::Node>(
        "wrench_subscriber_publisher_test");
    auto publisher =
        publisher_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/test/wrench_reader", rclcpp::SensorDataQoS());

    WrenchSubscriberConfig config;
    config.topic_name = "/test/wrench_reader";
    config.expected_frame_id = "test_ft";
    config.stale_timeout = 0.05;
    WrenchSubscriber reader(subscriber_node, config);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);

    geometry_msgs::msg::WrenchStamped message;
    message.header.frame_id = config.expected_frame_id;
    message.wrench.force.x = 1.0;
    message.wrench.force.y = 2.0;
    message.wrench.force.z = 3.0;
    message.wrench.torque.x = 0.1;
    message.wrench.torque.y = 0.2;
    message.wrench.torque.z = 0.3;

    const auto receive_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < receive_deadline &&
        !reader.latest().received)
    {
        message.header.stamp = publisher_node->now();
        publisher->publish(message);
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }

    const auto valid_state = reader.latest();
    ASSERT_TRUE(valid_state.received);
    EXPECT_FALSE(valid_state.stale);
    EXPECT_EQ(valid_state.sample.frame_id, config.expected_frame_id);
    EXPECT_DOUBLE_EQ(valid_state.sample.values[0], 1.0);
    EXPECT_DOUBLE_EQ(valid_state.sample.values[5], 0.3);
    EXPECT_GT(valid_state.accepted_samples, 0U);

    message.header.frame_id = "wrong_frame";
    message.wrench.force.x = std::numeric_limits<double>::quiet_NaN();
    const auto stale_deadline = std::chrono::steady_clock::now() + 100ms;
    while (std::chrono::steady_clock::now() < stale_deadline)
    {
        message.header.stamp = publisher_node->now();
        publisher->publish(message);
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }

    const auto invalid_state = reader.latest();
    EXPECT_TRUE(invalid_state.stale);
    EXPECT_GT(invalid_state.rejected_samples, 0U);
    EXPECT_DOUBLE_EQ(invalid_state.sample.values[0], 1.0);
}

}  // namespace
}  // namespace massage_motion
