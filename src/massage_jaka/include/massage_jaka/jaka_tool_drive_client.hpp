#ifndef MASSAGE_JAKA__JAKA_TOOL_DRIVE_CLIENT_HPP_
#define MASSAGE_JAKA__JAKA_TOOL_DRIVE_CLIENT_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "jaka_msgs/srv/get_tool_drive_state.hpp"
#include "jaka_msgs/srv/set_tool_drive_config.hpp"
#include "jaka_msgs/srv/set_tool_drive_frame.hpp"
#include "jaka_msgs/srv/set_tool_drive_tuning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace massage_jaka
{

struct JakaToolDriveConfig
{
    std::string config_service{"/jaka_driver/set_tool_drive_config"};
    std::string frame_service{"/jaka_driver/set_tool_drive_frame"};
    std::string tuning_service{"/jaka_driver/set_tool_drive_tuning"};
    std::string state_service{"/jaka_driver/get_tool_drive_state"};
    std::string enable_service{"/jaka_driver/enable_tool_drive"};
    std::int32_t axis{2};
    std::int32_t frame{0};
    double rebound{0.0};
    double rigidity{0.0};
    bool preserve_existing_axis_parameters{true};
    bool preserve_existing_sensitivity{true};
    bool preserve_existing_warning_range{true};
    std::int32_t sensitivity_level{1};
    std::int32_t warning_range{1};
    double service_timeout{3.0};
};

struct JakaToolDriveState
{
    bool success{false};
    bool enabled{false};
    std::int32_t warning_state{0};
    std::int32_t frame{0};
    std::int32_t sensitivity_level{0};
    std::int32_t warning_range{0};
    std::string control_owner;
    std::array<std::int32_t, 6> axis_options{};
    std::array<double, 6> rebound{};
    std::array<double, 6> rigidity{};
    std::string message;
};

class JakaToolDriveClient
{
public:
    JakaToolDriveClient(
        rclcpp::Node::SharedPtr node,
        JakaToolDriveConfig config = {});

    bool configure(std::string & message);
    bool start(std::string & message);
    bool stop(std::string & message);
    JakaToolDriveState state() const;

private:
    bool set_axis(
        std::int32_t axis, bool enabled, std::string & message);
    bool verify(
        bool expected_enabled,
        const std::string & expected_owner,
        std::string & message) const;

    rclcpp::Node::SharedPtr node_;
    JakaToolDriveConfig config_;
    rclcpp::Client<jaka_msgs::srv::SetToolDriveConfig>::SharedPtr
        config_client_;
    rclcpp::Client<jaka_msgs::srv::SetToolDriveFrame>::SharedPtr frame_client_;
    rclcpp::Client<jaka_msgs::srv::SetToolDriveTuning>::SharedPtr
        tuning_client_;
    rclcpp::Client<jaka_msgs::srv::GetToolDriveState>::SharedPtr state_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
};

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__JAKA_TOOL_DRIVE_CLIENT_HPP_
