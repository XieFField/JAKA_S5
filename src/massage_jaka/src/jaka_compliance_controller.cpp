#include "massage_jaka/jaka_compliance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <utility>

#include "massage_motion/compliance_validation.hpp"

namespace massage_jaka
{

namespace
{

std::array<double, 6> wrench_values(
    const geometry_msgs::msg::Wrench & wrench)
{
    return {
        wrench.force.x, wrench.force.y, wrench.force.z,
        wrench.torque.x, wrench.torque.y, wrench.torque.z};
}

}  // namespace

JakaComplianceController::JakaComplianceController(
    rclcpp::Node::SharedPtr node,
    JakaComplianceConfig config)
    : node_(std::move(node)), config_(std::move(config))
{
    wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
        config_.wrench_topic,
        rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_wrench_ = *message;
            latest_wrench_time_ = std::chrono::steady_clock::now();
            has_wrench_ = true;
        });
    soft_limit_client_ =
        node_->create_client<jaka_msgs::srv::SetTorqueSensorSoftLimit>(
            config_.soft_limit_service);
    config_client_ = node_->create_client<jaka_msgs::srv::SetAdmittanceConfig>(
        config_.config_service);
    enable_client_ = node_->create_client<std_srvs::srv::SetBool>(
        config_.enable_service);
}

JakaComplianceController::~JakaComplianceController()
{
    stop_requested_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    if (status_.load() == massage_motion::ComplianceStatus::kActive)
    {
        std::string ignored;
        set_enabled(false, ignored);
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

    if (!set_soft_limits(request) || !configure_axes(request))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kControlFailed,
            0,
            "JAKA 软限幅或导纳参数配置失败",
            massage_motion::ComplianceStatus::kFault};
        status_.store(result.status);
        set_result(result);
        return result;
    }

    std::string enable_message;
    if (!set_enabled(true, enable_message))
    {
        const massage_motion::ComplianceResult result{
            false,
            massage_motion::ComplianceError::kControlFailed,
            0,
            "启用 JAKA 柔顺失败: " + enable_message,
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

massage_motion::ComplianceResult JakaComplianceController::stop()
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    stop_requested_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    if (status_.load() == massage_motion::ComplianceStatus::kActive)
    {
        std::string message;
        const bool disabled = set_enabled(false, message);
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
    if (status_.load() != massage_motion::ComplianceStatus::kActive ||
        reference.joint_names.empty() ||
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

    // JAKA 的导纳后端由目标力和内部参考驱动。这里保留状态机给出的名义
    // 关节参考用于诊断，但不会绕过驱动再开启第二条 servo 命令通道。
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_reference_ = reference;
    return true;
}

massage_motion::ComplianceFeedback JakaComplianceController::feedback() const
{
    massage_motion::ComplianceFeedback result;
    result.status = status_.load();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_wrench_)
    {
        result.age = std::numeric_limits<double>::infinity();
        return result;
    }
    result.wrench = wrench_values(latest_wrench_.wrench);
    result.age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_wrench_time_).count();
    result.stale = result.age > config_.feedback_timeout;
    return result;
}

bool JakaComplianceController::reset()
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (status_.load() == massage_motion::ComplianceStatus::kActive)
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
        service_request->target_wrench = request.target_wrench[axis];
        service_request->constant = config_.constant[axis];
        service_request->normal_track = 0;
        service_request->rebound = config_.rebound[axis];
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
    std::string & message)
{
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
    message = response->message;
    return response->success;
}

void JakaComplianceController::monitor_loop()
{
    const auto started_at = std::chrono::steady_clock::now();
    massage_motion::ComplianceError error = massage_motion::ComplianceError::kNone;
    std::string message = "收到停止请求";
    std::array<double, massage_motion::kCartesianDof> peak_absolute_wrench{};

    while (rclcpp::ok() && !stop_requested_.load())
    {
        const auto current = feedback();
        massage_motion::ComplianceRequest request;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            request = request_;
        }
        if (current.stale)
        {
            error = massage_motion::ComplianceError::kFeedbackUnavailable;
            message = "JAKA FT 反馈超时";
            break;
        }
        for (std::size_t axis = 0; axis < current.wrench.size(); ++axis)
        {
            peak_absolute_wrench[axis] = std::max(
                peak_absolute_wrench[axis], std::abs(current.wrench[axis]));
            if (request.enabled_axes[axis] &&
                std::abs(current.wrench[axis]) >
                    request.max_absolute_wrench[axis])
            {
                error = massage_motion::ComplianceError::kLimitExceeded;
                message = "JAKA FT 反馈超过项目安全上限";
                break;
            }
        }
        if (error != massage_motion::ComplianceError::kNone)
        {
            break;
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
    const bool disabled = set_enabled(false, disable_message);
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
