#include "massage_jaka/jaka_compliance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "tf2/exceptions.h"

#include "massage_motion/compliance_validation.hpp"
#include "massage_jaka/jaka_compliance_guard.hpp"

namespace massage_jaka
{

JakaComplianceController::JakaComplianceController(
    rclcpp::Node::SharedPtr node,
    JakaComplianceConfig config)
    : node_(std::move(node)),
      config_(std::move(config)),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_)
{
    const std::set<std::string> unique_joint_names(
        config_.joint_names.begin(), config_.joint_names.end());
    const auto finite_positive = [](double value)
        {return std::isfinite(value) && value > 0.0;};
    if (config_.wrench_topic.empty() || config_.joint_state_topic.empty() ||
        config_.robot_state_topic.empty() ||
        config_.soft_limit_service.empty() || config_.config_service.empty() ||
        config_.enable_service.empty() ||
        config_.force_control_frame_service.empty() ||
        config_.admittance_state_service.empty() ||
        config_.wrench_frame.empty() || config_.base_frame.empty() ||
        config_.tool_frame.empty() ||
        (config_.force_control_frame != 0 &&
        config_.force_control_frame != 1) ||
        unique_joint_names.size() != config_.joint_names.size() ||
        unique_joint_names.find("") != unique_joint_names.end() ||
        !finite_positive(config_.service_timeout) ||
        !finite_positive(config_.feedback_timeout) ||
        !finite_positive(config_.state_timeout) ||
        !finite_positive(config_.monitor_period) ||
        !std::all_of(
            config_.disabled_axis_soft_limits.begin(),
            config_.disabled_axis_soft_limits.end(),
            finite_positive) ||
        !std::all_of(
            config_.maximum_speed_wrench.begin(),
            config_.maximum_speed_wrench.end(),
            [](double value)
            {
                return std::isfinite(value) && value >= 0.0;
            }) ||
        !std::all_of(
            config_.rebound_wrench.begin(), config_.rebound_wrench.end(),
            [](double value)
            {
                return std::isfinite(value) && value >= 0.0;
            }))
    {
        throw std::invalid_argument("JAKA 柔顺适配器配置无效");
    }

    massage_motion::WrenchSubscriberConfig wrench_config;
    wrench_config.topic_name = config_.wrench_topic;
    wrench_config.expected_frame_id = config_.wrench_frame;
    wrench_config.stale_timeout = config_.feedback_timeout;
    wrench_subscriber_ =
        std::make_shared<massage_motion::WrenchSubscriber>(
        node_, std::move(wrench_config));
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        config_.joint_state_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr message)
        {
            if (message->name.size() != message->position.size())
            {
                return;
            }
            std::vector<double> ordered_positions;
            ordered_positions.reserve(config_.joint_names.size());
            for (const auto & expected_name : config_.joint_names)
            {
                const auto iterator = std::find(
                    message->name.begin(), message->name.end(), expected_name);
                if (iterator == message->name.end())
                {
                    return;
                }
                const auto index = static_cast<std::size_t>(
                    std::distance(message->name.begin(), iterator));
                if (!std::isfinite(message->position[index]))
                {
                    return;
                }
                ordered_positions.push_back(message->position[index]);
            }
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_joint_positions_ = std::move(ordered_positions);
            latest_joint_state_time_ = std::chrono::steady_clock::now();
            has_joint_state_ = true;
        });
    robot_state_sub_ = node_->create_subscription<jaka_msgs::msg::RobotMsg>(
        config_.robot_state_topic,
        rclcpp::SensorDataQoS(),
        [this](const jaka_msgs::msg::RobotMsg::SharedPtr message)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_robot_state_ = *message;
            latest_robot_state_time_ = std::chrono::steady_clock::now();
            has_robot_state_ = true;
        });
    soft_limit_client_ =
        node_->create_client<jaka_msgs::srv::SetTorqueSensorSoftLimit>(
            config_.soft_limit_service);
    config_client_ = node_->create_client<jaka_msgs::srv::SetAdmittanceConfig>(
        config_.config_service);
    enable_client_ = node_->create_client<std_srvs::srv::SetBool>(
        config_.enable_service);
    force_control_frame_client_ =
        node_->create_client<jaka_msgs::srv::SetForceControlFrame>(
            config_.force_control_frame_service);
    admittance_state_client_ =
        node_->create_client<jaka_msgs::srv::GetAdmittanceState>(
            config_.admittance_state_service);
}

JakaComplianceController::~JakaComplianceController()
{
    stop_requested_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    if (enable_may_be_active_.load())
    {
        std::string ignored;
        set_enabled(false, ignored);
        enable_may_be_active_.store(false);
    }
}

massage_motion::ComplianceResult JakaComplianceController::start(
    const massage_motion::ComplianceRequest & request)
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (status_.load() == massage_motion::ComplianceStatus::kActive)
    {
        return {
            false,
            massage_motion::ComplianceError::kAlreadyActive,
            0,
            "JAKA 柔顺控制已经启用",
            massage_motion::ComplianceStatus::kActive};
    }
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }

    const auto validation = massage_motion::validate_compliance_request(request);
    if (!validation.valid)
    {
        const massage_motion::ComplianceResult result{
            false,
            validation.error,
            0,
            validation.message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    for (std::size_t axis = 0; axis < request.enabled_axes.size(); ++axis)
    {
        if (request.enabled_axes[axis] &&
            config_.maximum_speed_wrench[axis] <= 0.0)
        {
            const massage_motion::ComplianceResult result{
                false,
                massage_motion::ComplianceError::kInvalidRequest,
                0,
                "启用轴必须显式配置正的 JAKA maximum_speed_wrench",
                massage_motion::ComplianceStatus::kFault};
            status_.store(result.status);
            set_result(result);
            return result;
        }
    }

    std::string robot_state_message;
    if (!robot_ready(robot_state_message))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kBackendUnavailable,
            0,
            robot_state_message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    const auto current_feedback = feedback();
    if (current_feedback.stale)
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kFeedbackUnavailable,
            0,
            "JAKA FT 反馈不可用或已经过期",
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    std::string initial_state_message;
    if (!capture_initial_state(initial_state_message))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kFeedbackUnavailable,
            0,
            initial_state_message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    std::string frame_message;
    if (!set_force_control_frame(frame_message) ||
        !set_soft_limits(request) || !configure_axes(request))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kControlFailed,
            0,
            "JAKA 力控坐标系、软限幅或导纳参数配置失败: " +
                frame_message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    std::string verification_message;
    if (!verify_configuration(
            request, false, "idle", verification_message))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kControlFailed,
            0,
            "JAKA 导纳配置读回不一致: " + verification_message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    std::string enable_message;
    bool enable_response_received = false;
    if (!set_enabled(true, enable_message, &enable_response_received))
    {
        std::string cleanup_message;
        bool cleanup_succeeded = true;
        if (!enable_response_received)
        {
            // 服务响应超时时无法确定控制柜是否已经完成启用，必须反向调用关闭。
            enable_may_be_active_.store(true);
            cleanup_succeeded = set_enabled(false, cleanup_message);
            if (cleanup_succeeded)
            {
                enable_may_be_active_.store(false);
            }
        }
        else
        {
            cleanup_message = "服务已明确拒绝启用，无需反向关闭";
        }
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kControlFailed,
            0,
            "启用 JAKA 柔顺失败: " + enable_message +
                "; 关闭尝试: " + cleanup_message +
                (cleanup_succeeded ? "" : " (未确认关闭)"),
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }
    enable_may_be_active_.store(true);

    if (!verify_configuration(
            request, true, "compliance", verification_message))
    {
        std::string cleanup_message;
        const bool cleanup_succeeded = set_enabled(false, cleanup_message);
        if (cleanup_succeeded)
        {
            enable_may_be_active_.store(false);
        }
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kControlFailed,
            0,
            "启用后状态读回失败: " + verification_message +
                "; 关闭尝试: " + cleanup_message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        request_ = request;
    }
    stop_requested_.store(false);
    status_.store(massage_motion::ComplianceStatus::kActive);
    const massage_motion::ComplianceResult result{
        true,
        massage_motion::ComplianceError::kNone,
        0,
        "JAKA 柔顺已启用并开始反馈监控",
        massage_motion::ComplianceStatus::kActive};
    set_result(result);
    monitor_thread_ = std::thread(&JakaComplianceController::monitor_loop, this);
    return result;
}

massage_motion::ComplianceResult JakaComplianceController::configure(
    const massage_motion::ComplianceRequest & request)
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (status_.load() == massage_motion::ComplianceStatus::kActive ||
        enable_may_be_active_.load())
    {
        return {
            false,
            massage_motion::ComplianceError::kAlreadyActive,
            0,
            "JAKA 柔顺控制仍处于启用或未确认关闭状态",
            status_.load()};
    }
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }

    const auto validation = massage_motion::validate_compliance_request(request);
    if (!validation.valid)
    {
        const massage_motion::ComplianceResult result{
            false,
            validation.error,
            0,
            validation.message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }
    for (std::size_t axis = 0; axis < request.enabled_axes.size(); ++axis)
    {
        if (request.enabled_axes[axis] &&
            config_.maximum_speed_wrench[axis] <= 0.0)
        {
            const massage_motion::ComplianceResult result{
                false,
                massage_motion::ComplianceError::kInvalidRequest,
                0,
                "启用轴必须显式配置正的 JAKA maximum_speed_wrench",
                massage_motion::ComplianceStatus::kFault};
            status_.store(result.status);
            set_result(result);
            return result;
        }
    }

    std::string robot_state_message;
    if (!robot_ready(robot_state_message))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kBackendUnavailable,
            0,
            robot_state_message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }
    if (feedback().stale)
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kFeedbackUnavailable,
            0,
            "JAKA FT 反馈不可用或已经过期",
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    std::string frame_message;
    if (!set_force_control_frame(frame_message) ||
        !set_soft_limits(request) || !configure_axes(request))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kControlFailed,
            0,
            "JAKA 力控坐标系、软限幅或导纳参数配置失败: " +
                frame_message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    std::string verification_message;
    if (!verify_configuration(
            request, false, "idle", verification_message))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kControlFailed,
            0,
            "JAKA 导纳配置读回不一致: " + verification_message,
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    const massage_motion::ComplianceResult result{
        true,
        massage_motion::ComplianceError::kNone,
        0,
        "JAKA 导纳配置写入和关闭状态读回一致，未启用力控",
        massage_motion::ComplianceStatus::kIdle};
    status_.store(result.status);
    set_result(result);
    return result;
}

massage_motion::ComplianceResult JakaComplianceController::stop()
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    stop_requested_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    if (!enable_may_be_active_.load())
    {
        if (status_.load() == massage_motion::ComplianceStatus::kStopped)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            return last_result_;
        }
        const massage_motion::ComplianceResult result{
            true,
            massage_motion::ComplianceError::kNone,
            0,
            "JAKA 柔顺已经处于关闭状态",
            massage_motion::ComplianceStatus::kStopped};
        status_.store(result.status);
        set_result(result);
        return result;
    }
    if (enable_may_be_active_.load())
    {
        std::string message;
        bool disabled = set_enabled(false, message);
        std::string verification_message;
        if (disabled)
        {
            disabled = verify_configuration(
                request_, false, "idle", verification_message);
            message += "; " + verification_message;
        }
        if (disabled)
        {
            enable_may_be_active_.store(false);
        }
        const massage_motion::ComplianceResult result{
            disabled,
            disabled ? massage_motion::ComplianceError::kNone :
                massage_motion::ComplianceError::kControlFailed,
            0,
            message,
            disabled ? massage_motion::ComplianceStatus::kStopped :
                massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_result_;
}

bool JakaComplianceController::update_reference(
    const massage_motion::ComplianceReference & reference)
{
    const std::set<std::string> reference_joint_names(
        reference.joint_names.begin(), reference.joint_names.end());
    const std::set<std::string> expected_joint_names(
        config_.joint_names.begin(), config_.joint_names.end());
    if (status_.load() != massage_motion::ComplianceStatus::kActive ||
        reference.joint_names.size() != config_.joint_names.size() ||
        reference_joint_names != expected_joint_names ||
        reference.joint_names.size() != reference.positions.size() ||
        (!reference.velocities.empty() &&
         reference.velocities.size() != reference.positions.size()) ||
        !std::isfinite(reference.time_from_start) ||
        reference.time_from_start < 0.0 ||
        !std::all_of(
            reference.positions.begin(), reference.positions.end(),
            [](double value) {return std::isfinite(value);}) ||
        !std::all_of(
            reference.velocities.begin(), reference.velocities.end(),
            [](double value) {return std::isfinite(value);}))
    {
        return false;
    }

    // 当前 JAKA 驱动没有把轨迹参考与力控放在同一 SDK 控制通道中。
    // 在实现组合执行入口前必须明确拒绝，禁止只缓存数据却向状态机报告成功。
    RCLCPP_ERROR(
        node_->get_logger(),
        "JAKA 真机尚未实现导纳模式下的运动参考下发，拒绝伪成功");
    return false;
}

massage_motion::ComplianceFeedback JakaComplianceController::feedback() const
{
    massage_motion::ComplianceFeedback result;
    result.status = status_.load();
    const auto wrench_state = wrench_subscriber_->latest();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!wrench_state.received || !has_joint_state_)
    {
        result.age = std::numeric_limits<double>::infinity();
        return result;
    }
    result.wrench = wrench_state.sample.values;
    result.wrench_stamp_nanoseconds =
        wrench_state.sample.stamp_nanoseconds;
    result.joint_positions = latest_joint_positions_;
    const double joint_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_joint_state_time_).count();
    result.age = std::max(wrench_state.age, joint_age);
    result.stale = wrench_state.stale ||
        joint_age > config_.state_timeout;
    return result;
}

bool JakaComplianceController::current_tool_translation(
    std::array<double, 3> & translation,
    std::string & message)
{
    try
    {
        const auto transform = tf_buffer_.lookupTransform(
            config_.base_frame, config_.tool_frame, tf2::TimePointZero);
        translation = {
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z};
        if (!std::all_of(
                translation.begin(), translation.end(),
                [](double value) {return std::isfinite(value);}))
        {
            message = "TCP TF 包含非有限数值";
            return false;
        }
        return true;
    }
    catch (const tf2::TransformException & exception)
    {
        message = std::string("读取 TCP TF 失败: ") + exception.what();
        return false;
    }
}

bool JakaComplianceController::capture_initial_state(std::string & message)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!has_joint_state_ || latest_joint_positions_.empty())
        {
            message = "JAKA 关节反馈不可用";
            return false;
        }
        initial_joint_positions_ = latest_joint_positions_;
    }
    if (!current_tool_translation(initial_tool_translation_, message))
    {
        return false;
    }
    message = "初始关节状态和 TCP 位姿有效";
    return true;
}

bool JakaComplianceController::reset()
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (status_.load() == massage_motion::ComplianceStatus::kActive ||
        enable_may_be_active_.load())
    {
        return false;
    }
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    status_.store(massage_motion::ComplianceStatus::kIdle);
    set_result({
        true,
        massage_motion::ComplianceError::kNone,
        0,
        "JAKA 柔顺适配器已复位",
        massage_motion::ComplianceStatus::kIdle});
    return true;
}

massage_motion::ComplianceStatus JakaComplianceController::status() const
{
    return status_.load();
}

bool JakaComplianceController::robot_ready(
    std::string & message,
    bool require_stationary) const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_robot_state_)
    {
        message = "JAKA 机器人状态尚未收到";
        return false;
    }
    const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_robot_state_time_).count();
    if (age > config_.state_timeout)
    {
        message = "JAKA 机器人状态已经过期";
        return false;
    }
    if (latest_robot_state_.power_state != 1 ||
        latest_robot_state_.servo_state != 1)
    {
        message = "JAKA 尚未上电并使能";
        return false;
    }
    if (latest_robot_state_.collision_state != 0)
    {
        message = "JAKA 当前处于碰撞状态";
        return false;
    }
    if (require_stationary && latest_robot_state_.motion_state != 0)
    {
        message = "JAKA 当前不处于静止状态";
        return false;
    }
    message = "JAKA 已上电使能、无碰撞且处于静止状态";
    return true;
}

bool JakaComplianceController::set_force_control_frame(std::string & message)
{
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!force_control_frame_client_->wait_for_service(timeout))
    {
        message = "set_force_control_frame service unavailable";
        return false;
    }
    auto request =
        std::make_shared<jaka_msgs::srv::SetForceControlFrame::Request>();
    request->frame = config_.force_control_frame;
    auto future = force_control_frame_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        message = "set_force_control_frame service timeout";
        return false;
    }
    const auto response = future.get();
    message = response->message;
    return response->success;
}

bool JakaComplianceController::verify_configuration(
    const massage_motion::ComplianceRequest & request,
    bool expected_enabled,
    const std::string & expected_owner,
    std::string & message)
{
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!admittance_state_client_->wait_for_service(timeout))
    {
        message = "get_admittance_state service unavailable";
        return false;
    }
    auto service_request =
        std::make_shared<jaka_msgs::srv::GetAdmittanceState::Request>();
    auto future = admittance_state_client_->async_send_request(service_request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        message = "get_admittance_state service timeout";
        return false;
    }
    const auto response = future.get();
    if (!response->success)
    {
        message = response->message;
        return false;
    }

    constexpr double kReadbackTolerance = 1e-9;
    const auto equal = [](double left, double right)
        {return std::abs(left - right) <= kReadbackTolerance;};
    if (response->force_control_enabled != expected_enabled ||
        response->force_control_frame != config_.force_control_frame ||
        response->control_owner != expected_owner)
    {
        message =
            "力控状态读回不一致: enabled expected=" +
            std::string(expected_enabled ? "true" : "false") +
            " actual=" +
            std::string(response->force_control_enabled ? "true" : "false") +
            ", frame expected=" +
            std::to_string(config_.force_control_frame) + " actual=" +
            std::to_string(response->force_control_frame) +
            ", owner expected=" + expected_owner + " actual=" +
            response->control_owner + ", sensor_compensation=" +
            std::to_string(response->sensor_compensation) +
            ", compliance_type=" +
            std::to_string(response->compliance_type);
        return false;
    }
    for (std::size_t axis = 0; axis < request.enabled_axes.size(); ++axis)
    {
        const double expected_limit = request.enabled_axes[axis] ?
            request.max_absolute_wrench[axis] :
            config_.disabled_axis_soft_limits[axis];
        const std::int32_t expected_option = request.enabled_axes[axis] ? 1 : 0;
        if (!equal(response->soft_limits[axis], expected_limit) ||
            response->axis_options[axis] != expected_option ||
            !equal(
                response->maximum_speed_wrench[axis],
                config_.maximum_speed_wrench[axis]) ||
            !equal(
                response->constant_wrench[axis],
                request.target_wrench[axis]) ||
            response->normal_track[axis] != 0 ||
            !equal(
                response->rebound_wrench[axis],
                config_.rebound_wrench[axis]))
        {
            message = "软限幅或第 " + std::to_string(axis) +
                " 轴导纳参数读回不一致";
            return false;
        }
    }
    message = expected_enabled ?
        "导纳配置与启用状态读回一致" :
        "导纳配置、关闭状态和控制权读回一致";
    return true;
}

bool JakaComplianceController::set_soft_limits(
    const massage_motion::ComplianceRequest & request)
{
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!soft_limit_client_->wait_for_service(timeout))
    {
        return false;
    }
    auto service_request =
        std::make_shared<jaka_msgs::srv::SetTorqueSensorSoftLimit::Request>();
    for (std::size_t axis = 0; axis < request.enabled_axes.size(); ++axis)
    {
        service_request->limits[axis] = request.enabled_axes[axis] ?
            request.max_absolute_wrench[axis] :
            config_.disabled_axis_soft_limits[axis];
    }
    auto future = soft_limit_client_->async_send_request(service_request);
    return future.wait_for(timeout) == std::future_status::ready &&
        future.get()->success;
}

bool JakaComplianceController::configure_axes(
    const massage_motion::ComplianceRequest & request)
{
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!config_client_->wait_for_service(timeout))
    {
        return false;
    }
    for (std::size_t axis = 0; axis < request.enabled_axes.size(); ++axis)
    {
        auto service_request =
            std::make_shared<jaka_msgs::srv::SetAdmittanceConfig::Request>();
        service_request->axis = static_cast<std::int32_t>(axis);
        service_request->option = request.enabled_axes[axis] ? 1 : 0;
        service_request->maximum_speed_wrench =
            config_.maximum_speed_wrench[axis];
        service_request->constant_wrench = request.target_wrench[axis];
        service_request->normal_track = 0;
        service_request->rebound_wrench = config_.rebound_wrench[axis];
        auto future = config_client_->async_send_request(service_request);
        if (future.wait_for(timeout) != std::future_status::ready ||
            !future.get()->success)
        {
            return false;
        }
    }
    return true;
}

bool JakaComplianceController::set_enabled(
    bool enabled,
    std::string & message,
    bool * response_received)
{
    if (response_received)
    {
        *response_received = false;
    }
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!enable_client_->wait_for_service(timeout))
    {
        message = "enable_admittance service unavailable";
        return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = enable_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        message = "enable_admittance service timeout";
        return false;
    }
    const auto response = future.get();
    if (response_received)
    {
        *response_received = true;
    }
    message = response->message;
    return response->success;
}

void JakaComplianceController::monitor_loop()
{
    const auto started_at = std::chrono::steady_clock::now();
    massage_motion::ComplianceError error = massage_motion::ComplianceError::kNone;
    std::string message = "收到停止请求";
    std::array<double, massage_motion::kCartesianDof> peak_absolute_wrench{};
    std::vector<double> initial_joints;
    std::array<double, 3> initial_translation{};
    massage_motion::ComplianceRequest request;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        initial_joints = initial_joint_positions_;
        initial_translation = initial_tool_translation_;
        request = request_;
    }

    while (rclcpp::ok() && !stop_requested_.load())
    {
        const auto current = feedback();
        if (current.stale)
        {
            error = massage_motion::ComplianceError::kFeedbackUnavailable;
            message = "JAKA FT 反馈超时";
            break;
        }
        std::string robot_state_message;
        if (!robot_ready(robot_state_message, false))
        {
            error = massage_motion::ComplianceError::kBackendUnavailable;
            message = robot_state_message;
            break;
        }
        std::array<double, 3> current_translation{};
        std::string transform_message;
        if (!current_tool_translation(current_translation, transform_message))
        {
            error = massage_motion::ComplianceError::kFeedbackUnavailable;
            message = transform_message;
            break;
        }
        const auto guard = evaluate_compliance_guard(
            initial_joints,
            current.joint_positions,
            initial_translation,
            current_translation,
            current.wrench,
            request.max_absolute_wrench,
            request.max_joint_displacement,
            request.max_linear_displacement);
        if (!guard.valid)
        {
            error = massage_motion::ComplianceError::kFeedbackUnavailable;
            message = guard.message;
            break;
        }
        if (guard.limit_exceeded)
        {
            error = massage_motion::ComplianceError::kLimitExceeded;
            message = guard.message;
            break;
        }
        for (std::size_t axis = 0; axis < current.wrench.size(); ++axis)
        {
            peak_absolute_wrench[axis] = std::max(
                peak_absolute_wrench[axis], std::abs(current.wrench[axis]));
        }
        if (std::chrono::steady_clock::now() - started_at >=
            std::chrono::duration<double>(request.timeout))
        {
            error = massage_motion::ComplianceError::kTimeout;
            message = "JAKA 柔顺运行超时";
            break;
        }
        std::this_thread::sleep_for(
            std::chrono::duration<double>(config_.monitor_period));
    }

    std::string disable_message;
    bool disabled = set_enabled(false, disable_message);
    std::string verification_message;
    if (disabled)
    {
        disabled = verify_configuration(
            request, false, "idle", verification_message);
        disable_message += "; " + verification_message;
    }
    if (disabled)
    {
        enable_may_be_active_.store(false);
    }
    const bool normal = error == massage_motion::ComplianceError::kNone && disabled;
    const auto final_status = disabled ?
        massage_motion::ComplianceStatus::kStopped :
        massage_motion::ComplianceStatus::kFault;
    status_.store(final_status);
    set_result({
        normal,
        disabled ? error : massage_motion::ComplianceError::kControlFailed,
        0,
        message + "; " + disable_message,
        final_status,
        peak_absolute_wrench});
}

void JakaComplianceController::set_result(
    const massage_motion::ComplianceResult & result)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_result_ = result;
}

}  // namespace massage_jaka
