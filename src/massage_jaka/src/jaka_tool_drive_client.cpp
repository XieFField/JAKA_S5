#include "massage_jaka/jaka_tool_drive_client.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <utility>

namespace massage_jaka
{

JakaToolDriveClient::JakaToolDriveClient(
    rclcpp::Node::SharedPtr node,
    JakaToolDriveConfig config)
    : node_(std::move(node)), config_(std::move(config))
{
    if (!node_ || config_.axis < 0 || config_.axis >= 6 ||
        (config_.frame != 0 && config_.frame != 1) ||
        config_.sensitivity_level < 0 || config_.sensitivity_level > 5 ||
        config_.warning_range < 1 || config_.warning_range > 5 ||
        !std::isfinite(config_.rebound) || config_.rebound < 0.0 ||
        !std::isfinite(config_.rigidity) || config_.rigidity < 0.0 ||
        !std::isfinite(config_.service_timeout) ||
        config_.service_timeout <= 0.0)
    {
        throw std::invalid_argument("JAKA tool-drive 客户端配置无效");
    }
    config_client_ = node_->create_client<jaka_msgs::srv::SetToolDriveConfig>(
        config_.config_service);
    frame_client_ = node_->create_client<jaka_msgs::srv::SetToolDriveFrame>(
        config_.frame_service);
    tuning_client_ = node_->create_client<jaka_msgs::srv::SetToolDriveTuning>(
        config_.tuning_service);
    state_client_ = node_->create_client<jaka_msgs::srv::GetToolDriveState>(
        config_.state_service);
    enable_client_ = node_->create_client<std_srvs::srv::SetBool>(
        config_.enable_service);
}

bool JakaToolDriveClient::set_axis(
    std::int32_t axis, bool enabled, std::string & message)
{
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!config_client_->wait_for_service(timeout))
    {
        message = "set_tool_drive_config 服务不可用";
        return false;
    }
    auto request =
        std::make_shared<jaka_msgs::srv::SetToolDriveConfig::Request>();
    request->axis = axis;
    request->option = enabled ? 1 : 0;
    request->rebound = enabled ? config_.rebound : 0.0;
    request->rigidity = enabled ? config_.rigidity : 0.0;
    auto future = config_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        message = "set_tool_drive_config 调用超时";
        return false;
    }
    const auto response = future.get();
    message = response->message;
    return response->success;
}

bool JakaToolDriveClient::configure(std::string & message)
{
    if (config_.preserve_existing_axis_parameters ||
        config_.preserve_existing_sensitivity ||
        config_.preserve_existing_warning_range)
    {
        const auto current = state();
        if (!current.success)
        {
            message = "读取现有 tool drive 参数失败: " + current.message;
            return false;
        }
        if (config_.preserve_existing_axis_parameters)
        {
            const auto axis = static_cast<std::size_t>(config_.axis);
            config_.rebound = current.rebound[axis];
            config_.rigidity = current.rigidity[axis];
        }
        if (config_.preserve_existing_sensitivity)
        {
            config_.sensitivity_level = current.sensitivity_level;
        }
        if (config_.preserve_existing_warning_range)
        {
            config_.warning_range = current.warning_range;
        }
    }
    for (std::int32_t axis = 0; axis < 6; ++axis)
    {
        if (!set_axis(axis, axis == config_.axis, message))
        {
            return false;
        }
    }

    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!frame_client_->wait_for_service(timeout))
    {
        message = "set_tool_drive_frame 服务不可用";
        return false;
    }
    auto request =
        std::make_shared<jaka_msgs::srv::SetToolDriveFrame::Request>();
    request->frame = config_.frame;
    auto future = frame_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        message = "set_tool_drive_frame 调用超时";
        return false;
    }
    const auto response = future.get();
    if (!response->success)
    {
        message = response->message;
        return false;
    }

    if (!config_.preserve_existing_sensitivity ||
        !config_.preserve_existing_warning_range)
    {
        if (!tuning_client_->wait_for_service(timeout))
        {
            message = "set_tool_drive_tuning 服务不可用";
            return false;
        }
        auto tuning_request =
            std::make_shared<jaka_msgs::srv::SetToolDriveTuning::Request>();
        tuning_request->sensitivity_level = config_.sensitivity_level;
        tuning_request->warning_range = config_.warning_range;
        auto tuning_future = tuning_client_->async_send_request(tuning_request);
        if (tuning_future.wait_for(timeout) != std::future_status::ready)
        {
            message = "set_tool_drive_tuning 调用超时";
            return false;
        }
        const auto tuning_response = tuning_future.get();
        if (!tuning_response->success)
        {
            message = tuning_response->message;
            return false;
        }
    }
    return verify(false, "idle", message);
}

bool JakaToolDriveClient::start(std::string & message)
{
    const auto before_start = state();
    if (!before_start.success)
    {
        message = "启用前读取 tool drive 状态失败: " + before_start.message;
        return false;
    }
    if (before_start.sensitivity_level == 0)
    {
        message =
            "fusion-drive sensitivity=0，tool-drive 功能处于关闭状态";
        return false;
    }

    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!enable_client_->wait_for_service(timeout))
    {
        message = "enable_tool_drive 服务不可用";
        return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = enable_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        message = "enable_tool_drive 启用调用超时";
        return false;
    }
    const auto response = future.get();
    if (!response->success)
    {
        message = response->message;
        return false;
    }
    if (verify(true, "tool_drive", message))
    {
        return true;
    }

    const std::string verification_error = message;
    std::string cleanup_message;
    const bool cleanup_ok = stop(cleanup_message);
    message = verification_error + "; 启用失败清理(" +
        (cleanup_ok ? std::string("成功") : std::string("失败")) +
        "): " + cleanup_message;
    return false;
}

bool JakaToolDriveClient::stop(std::string & message)
{
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!enable_client_->wait_for_service(timeout))
    {
        message = "enable_tool_drive 服务不可用，无法关闭";
        return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    auto future = enable_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        message = "enable_tool_drive 关闭调用超时";
        return false;
    }
    const auto response = future.get();
    if (!response->success)
    {
        message = response->message;
        return false;
    }
    return verify(false, "idle", message);
}

JakaToolDriveState JakaToolDriveClient::state() const
{
    JakaToolDriveState result;
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!state_client_->wait_for_service(timeout))
    {
        result.message = "get_tool_drive_state 服务不可用";
        return result;
    }
    auto request =
        std::make_shared<jaka_msgs::srv::GetToolDriveState::Request>();
    auto future = state_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        result.message = "get_tool_drive_state 调用超时";
        return result;
    }
    const auto response = future.get();
    result.success = response->success;
    result.enabled = response->enabled;
    result.warning_state = response->warning_state;
    result.frame = response->frame;
    result.sensitivity_level = response->sensitivity_level;
    result.warning_range = response->warning_range;
    result.control_owner = response->control_owner;
    result.axis_options = response->axis_options;
    result.rebound = response->rebound;
    result.rigidity = response->rigidity;
    result.message = response->message;
    return result;
}

bool JakaToolDriveClient::verify(
    bool expected_enabled,
    const std::string & expected_owner,
    std::string & message) const
{
    const auto current = state();
    if (!current.success)
    {
        message = current.message;
        return false;
    }
    if (current.enabled != expected_enabled ||
        current.control_owner != expected_owner ||
        current.frame != config_.frame ||
        current.sensitivity_level != config_.sensitivity_level ||
        current.warning_range != config_.warning_range)
    {
        message = "tool drive 状态读回不一致: enabled=" +
            std::string(current.enabled ? "true" : "false") +
            ", owner=" + current.control_owner +
            ", frame=" + std::to_string(current.frame) +
            ", sensitivity=" +
            std::to_string(current.sensitivity_level) +
            ", warning_range=" + std::to_string(current.warning_range) +
            ", warning=" + std::to_string(current.warning_state);
        return false;
    }
    for (std::size_t axis = 0; axis < current.axis_options.size(); ++axis)
    {
        const std::int32_t expected =
            axis == static_cast<std::size_t>(config_.axis) ? 1 : 0;
        const double expected_rebound = expected == 1 ? config_.rebound : 0.0;
        const double expected_rigidity = expected == 1 ? config_.rigidity : 0.0;
        if (current.axis_options[axis] != expected ||
            std::abs(current.rebound[axis] - expected_rebound) > 1e-9 ||
            std::abs(current.rigidity[axis] - expected_rigidity) > 1e-9)
        {
            message = "tool drive 轴参数读回不一致";
            return false;
        }
    }
    message = expected_enabled ?
        "tool drive 配置、启用状态和控制权读回一致" :
        "tool drive 配置、关闭状态和控制权读回一致";
    return true;
}

}  // namespace massage_jaka
