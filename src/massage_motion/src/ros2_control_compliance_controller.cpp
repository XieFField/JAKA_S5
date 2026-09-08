#include "massage_motion/ros2_control_compliance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
#include <utility>

#include "massage_motion/compliance_validation.hpp"
#include "tf2/exceptions.h"

namespace massage_motion
{

namespace
{

std::array<double, kCartesianDof> wrench_to_array(
    const geometry_msgs::msg::Wrench & wrench)
{
    return {
        wrench.force.x,
        wrench.force.y,
        wrench.force.z,
        wrench.torque.x,
        wrench.torque.y,
        wrench.torque.z};
}

}  // namespace

Ros2ControlComplianceController::Ros2ControlComplianceController(
    rclcpp::Node::SharedPtr node,
    Ros2ControlComplianceConfig config)
    : node_(std::move(node)),
      config_(std::move(config)),
      logger_(node_->get_logger())
{
    if (config_.joint_names.empty())
    {
        throw std::invalid_argument("Ros2ControlComplianceConfig.joint_names 不能为空");
    }
    if (config_.cartesian_reference_frame.empty() ||
        config_.cartesian_tool_frame.empty())
    {
        throw std::invalid_argument("笛卡尔位移监控坐标系不能为空");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_, node_, false);

    switch_client_ = node_->create_client<SwitchController>(
        config_.controller_manager + "/switch_controller");

    reference_publisher_ =
        node_->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            config_.reference_topic,
            rclcpp::QoS(10).reliable());

    joint_state_subscription_ =
        node_->create_subscription<sensor_msgs::msg::JointState>(
            config_.joint_state_topic,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr message)
            {
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    latest_joint_state_ = *message;
                    latest_joint_receive_time_ = std::chrono::steady_clock::now();
                    has_joint_state_ = true;
                }
                feedback_condition_.notify_all();
            });

    wrench_subscription_ =
        node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            config_.wrench_topic,
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message)
            {
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    latest_wrench_ = *message;
                    latest_wrench_receive_time_ = std::chrono::steady_clock::now();
                    has_wrench_ = true;
                }
                feedback_condition_.notify_all();
            });

    status_subscription_ =
        node_->create_subscription<control_msgs::msg::AdmittanceControllerState>(
            config_.status_topic,
            rclcpp::SensorDataQoS(),
            [this](
                const control_msgs::msg::AdmittanceControllerState::SharedPtr message)
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_status_ = *message;
                has_status_ = true;
            });
}

Ros2ControlComplianceController::~Ros2ControlComplianceController()
{
    stop_requested_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
}

ComplianceCapabilities Ros2ControlComplianceController::capabilities() const
{
    return {true, true};
}

ComplianceResult Ros2ControlComplianceController::start(
    const ComplianceRequest & request)
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);

    if (status_.load() == ComplianceStatus::kActive)
    {
        return {
            false,
            ComplianceError::kAlreadyActive,
            0,
            "柔顺控制器已经处于 active 状态",
            ComplianceStatus::kActive};
    }

    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }

    const auto validation = validate_compliance_request(request);
    if (!validation.valid)
    {
        const ComplianceResult result{
            false,
            validation.error,
            0,
            validation.message,
            ComplianceStatus::kFault};
        status_.store(ComplianceStatus::kFault);
        set_last_result(result);
        return result;
    }

    if (!wait_for_initial_feedback())
    {
        const ComplianceResult result{
            false,
            ComplianceError::kFeedbackUnavailable,
            0,
            "等待关节状态和力传感器反馈超时",
            ComplianceStatus::kFault};
        status_.store(ComplianceStatus::kFault);
        set_last_result(result);
        return result;
    }

    const auto current_positions = current_joint_positions();
    if (current_positions.size() != config_.joint_names.size())
    {
        const ComplianceResult result{
            false,
            ComplianceError::kFeedbackUnavailable,
            0,
            "关节反馈不完整，无法建立安全参考姿态",
            ComplianceStatus::kFault};
        status_.store(ComplianceStatus::kFault);
        set_last_result(result);
        return result;
    }

    std::array<double, 3> current_cartesian_position{};
    std::string cartesian_error;
    if (!wait_for_cartesian_position(current_cartesian_position, cartesian_error))
    {
        const ComplianceResult result{
            false,
            ComplianceError::kFeedbackUnavailable,
            0,
            "无法建立笛卡尔安全基准: " + cartesian_error,
            ComplianceStatus::kFault};
        status_.store(ComplianceStatus::kFault);
        set_last_result(result);
        return result;
    }

    if (!publish_joint_reference(current_positions))
    {
        const ComplianceResult result{
            false,
            ComplianceError::kControlFailed,
            0,
            "发布当前关节参考失败",
            ComplianceStatus::kFault};
        status_.store(ComplianceStatus::kFault);
        set_last_result(result);
        return result;
    }

    // 让 inactive 控制器先收到与当前状态一致的参考，避免激活瞬间跳变。
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if (!switch_controllers(
            {config_.admittance_controller},
            {config_.trajectory_controller}))
    {
        const ComplianceResult result{
            false,
            ComplianceError::kControlFailed,
            0,
            "轨迹控制器切换到导纳控制器失败",
            ComplianceStatus::kFault};
        status_.store(ComplianceStatus::kFault);
        set_last_result(result);
        return result;
    }

    publish_joint_reference(current_positions);

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        baseline_joint_positions_ = current_positions;
        baseline_cartesian_position_ = current_cartesian_position;
        baseline_wrench_ = config_.subtract_startup_wrench_bias ?
            wrench_to_array(latest_wrench_.wrench) :
            std::array<double, kCartesianDof>{};
        active_request_ = request;
        has_status_ = false;
    }

    stop_requested_.store(false);
    status_.store(ComplianceStatus::kActive);

    const ComplianceResult result{
        true,
        ComplianceError::kNone,
        0,
        "导纳控制器已激活，安全监控已启动",
        ComplianceStatus::kActive};
    set_last_result(result);

    monitor_thread_ = std::thread(
        &Ros2ControlComplianceController::monitor_loop,
        this);

    return result;
}

ComplianceResult Ros2ControlComplianceController::stop()
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);

    if (status_.load() != ComplianceStatus::kActive)
    {
        if (monitor_thread_.joinable())
        {
            monitor_thread_.join();
        }
        return last_result();
    }

    stop_requested_.store(true);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }

    return last_result();
}

bool Ros2ControlComplianceController::update_reference(
    const ComplianceReference & reference)
{
    if (status_.load() != ComplianceStatus::kActive ||
        reference.joint_names.size() != reference.positions.size() ||
        (!reference.velocities.empty() &&
         reference.velocities.size() != reference.positions.size()) ||
        !std::isfinite(reference.time_from_start) ||
        reference.time_from_start < 0.0)
    {
        return false;
    }

    std::vector<double> positions;
    std::vector<double> velocities;
    positions.reserve(config_.joint_names.size());
    velocities.reserve(config_.joint_names.size());

    for (const auto & expected_name : config_.joint_names)
    {
        const auto iterator = std::find(
            reference.joint_names.begin(),
            reference.joint_names.end(),
            expected_name);
        if (iterator == reference.joint_names.end())
        {
            return false;
        }

        const auto index = static_cast<std::size_t>(
            std::distance(reference.joint_names.begin(), iterator));
        const double position = reference.positions[index];
        const double velocity = reference.velocities.empty() ?
            0.0 : reference.velocities[index];
        if (!std::isfinite(position) || !std::isfinite(velocity))
        {
            return false;
        }
        positions.push_back(position);
        velocities.push_back(velocity);
    }

    return publish_joint_reference(positions, velocities);
}

ComplianceFeedback Ros2ControlComplianceController::feedback() const
{
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);

    ComplianceFeedback feedback;
    feedback.status = status_.load();
    if (!has_joint_state_ || !has_wrench_)
    {
        feedback.age = std::numeric_limits<double>::infinity();
        return feedback;
    }

    feedback.wrench = wrench_to_array(latest_wrench_.wrench);
    for (std::size_t index = 0; index < kCartesianDof; ++index)
    {
        feedback.wrench[index] -= baseline_wrench_[index];
    }
    feedback.joint_names = config_.joint_names;
    feedback.joint_positions = ordered_joint_positions(latest_joint_state_);
    const auto oldest_receive_time = std::min(
        latest_joint_receive_time_, latest_wrench_receive_time_);
    feedback.age = std::chrono::duration<double>(
        now - oldest_receive_time).count();
    feedback.stale = feedback.joint_positions.size() != config_.joint_names.size() ||
        feedback.age > config_.feedback_timeout;
    return feedback;
}

bool Ros2ControlComplianceController::reset()
{
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (status_.load() == ComplianceStatus::kActive)
    {
        return false;
    }

    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }

    status_.store(ComplianceStatus::kIdle);
    set_last_result({
        true,
        ComplianceError::kNone,
        0,
        "柔顺控制器状态已复位",
        ComplianceStatus::kIdle});
    return true;
}

ComplianceStatus Ros2ControlComplianceController::status() const
{
    return status_.load();
}

ComplianceResult Ros2ControlComplianceController::last_result() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_result_;
}

bool Ros2ControlComplianceController::publish_joint_reference(
    const std::vector<double> & positions,
    const std::vector<double> & velocities)
{
    if (positions.size() != config_.joint_names.size() ||
        (!velocities.empty() && velocities.size() != positions.size()))
    {
        RCLCPP_ERROR(logger_, "关节参考维度与配置关节数量不一致");
        return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint reference;
    reference.positions = positions;
    reference.velocities = velocities.empty() ?
        std::vector<double>(positions.size(), 0.0) : velocities;
    reference_publisher_->publish(reference);
    return true;
}

std::vector<double>
Ros2ControlComplianceController::current_joint_positions() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_joint_state_)
    {
        return {};
    }
    return ordered_joint_positions(latest_joint_state_);
}

bool Ros2ControlComplianceController::lookup_cartesian_position(
    std::array<double, 3> & position,
    std::string & error_message) const
{
    try
    {
        const auto transform = tf_buffer_->lookupTransform(
            config_.cartesian_reference_frame,
            config_.cartesian_tool_frame,
            tf2::TimePointZero);
        position = {
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z};
        if (!std::all_of(
                position.begin(), position.end(),
                [](double value) {return std::isfinite(value);}))
        {
            error_message = "TCP TF 包含非有限数值";
            return false;
        }
        error_message.clear();
        return true;
    }
    catch (const tf2::TransformException & exception)
    {
        error_message = exception.what();
        return false;
    }
}

bool Ros2ControlComplianceController::wait_for_cartesian_position(
    std::array<double, 3> & position,
    std::string & error_message) const
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(config_.service_timeout);
    do
    {
        if (lookup_cartesian_position(position, error_message))
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    while (std::chrono::steady_clock::now() < deadline);
    return false;
}

bool Ros2ControlComplianceController::wait_for_initial_feedback()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    return feedback_condition_.wait_for(
        lock,
        std::chrono::duration<double>(config_.service_timeout),
        [this]() {return has_joint_state_ && has_wrench_;});
}

bool Ros2ControlComplianceController::switch_controllers(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate)
{
    const auto timeout = std::chrono::duration<double>(config_.service_timeout);
    if (!switch_client_->wait_for_service(timeout))
    {
        RCLCPP_ERROR(logger_, "等待 controller_manager/switch_controller 超时");
        return false;
    }

    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout = static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(config_.service_timeout));

    auto future = switch_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "controller switch 服务响应超时");
        return false;
    }

    const auto response = future.get();
    return response && response->ok;
}

void Ros2ControlComplianceController::monitor_loop()
{
    const auto started_at = std::chrono::steady_clock::now();
    ComplianceError stop_error = ComplianceError::kNone;
    std::string stop_message = "收到停止请求";
    std::array<double, kCartesianDof> peak_absolute_wrench{};
    double peak_joint_displacement = 0.0;
    double peak_selected_axis_translation = 0.0;

    while (rclcpp::ok() && !stop_requested_.load())
    {
        const auto now = std::chrono::steady_clock::now();

        ComplianceRequest request;
        sensor_msgs::msg::JointState joint_state;
        geometry_msgs::msg::WrenchStamped wrench;
        std::vector<double> baseline;
        std::array<double, 3> baseline_cartesian{};
        std::array<double, kCartesianDof> wrench_bias{};
        std::chrono::steady_clock::time_point joint_receive_time;
        std::chrono::steady_clock::time_point wrench_receive_time;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            request = active_request_;
            joint_state = latest_joint_state_;
            wrench = latest_wrench_;
            baseline = baseline_joint_positions_;
            baseline_cartesian = baseline_cartesian_position_;
            wrench_bias = baseline_wrench_;
            joint_receive_time = latest_joint_receive_time_;
            wrench_receive_time = latest_wrench_receive_time_;
        }

        const auto feedback_timeout =
            std::chrono::duration<double>(config_.feedback_timeout);
        if (now - joint_receive_time > feedback_timeout ||
            now - wrench_receive_time > feedback_timeout)
        {
            stop_error = ComplianceError::kFeedbackUnavailable;
            stop_message = "关节或力传感器反馈超时";
            break;
        }

        const auto positions = ordered_joint_positions(joint_state);
        if (positions.size() != baseline.size())
        {
            stop_error = ComplianceError::kFeedbackUnavailable;
            stop_message = "运行中关节反馈不完整";
            break;
        }

        double max_joint_delta = 0.0;
        std::size_t max_joint_index = 0U;
        for (std::size_t index = 0; index < positions.size(); ++index)
        {
            const double delta = std::abs(positions[index] - baseline[index]);
            if (delta > max_joint_delta)
            {
                max_joint_delta = delta;
                max_joint_index = index;
            }
        }
        if (max_joint_delta > request.max_joint_displacement)
        {
            stop_error = ComplianceError::kLimitExceeded;
            stop_message =
                "关节位移超过柔顺安全上限: joint=" +
                config_.joint_names[max_joint_index] +
                ", displacement=" + std::to_string(max_joint_delta) +
                " rad, limit=" +
                std::to_string(request.max_joint_displacement) + " rad";
            break;
        }
        peak_joint_displacement = std::max(
            peak_joint_displacement, max_joint_delta);

        auto wrench_values = wrench_to_array(wrench.wrench);
        for (std::size_t index = 0; index < kCartesianDof; ++index)
        {
            wrench_values[index] -= wrench_bias[index];
        }
        for (std::size_t index = 0; index < kCartesianDof; ++index)
        {
            peak_absolute_wrench[index] = std::max(
                peak_absolute_wrench[index], std::abs(wrench_values[index]));
        }
        const bool wrench_limit_arming =
            now - started_at <
            std::chrono::duration<double>(config_.wrench_limit_arming_delay);
        for (std::size_t index = 0; index < kCartesianDof; ++index)
        {
            const double active_wrench_limit = wrench_limit_arming ?
                std::max(
                    request.max_absolute_wrench[index],
                    config_.startup_max_absolute_wrench) :
                request.max_absolute_wrench[index];
            if (request.enabled_axes[index] &&
                std::abs(wrench_values[index]) >
                    active_wrench_limit)
            {
                const double elapsed = std::chrono::duration<double>(
                    now - started_at).count();
                RCLCPP_WARN(
                    logger_,
                    "柔顺力限位触发: axis=%zu, measured=%.6f, limit=%.6f, "
                    "elapsed=%.3f s, arming=%s",
                    index,
                    wrench_values[index],
                    active_wrench_limit,
                    elapsed,
                    wrench_limit_arming ? "true" : "false");
                stop_error = ComplianceError::kLimitExceeded;
                stop_message = wrench_limit_arming ?
                    "控制器接管阶段的力/力矩超过硬安全上限" :
                    "启用轴的力/力矩超过安全上限";
                break;
            }
        }
        if (stop_error != ComplianceError::kNone)
        {
            break;
        }

        std::array<double, 3> cartesian_position{};
        std::string cartesian_error;
        if (!lookup_cartesian_position(cartesian_position, cartesian_error))
        {
            stop_error = ComplianceError::kFeedbackUnavailable;
            stop_message = "笛卡尔位移安全监控失效: " + cartesian_error;
            break;
        }
        double selected_translation_squared = 0.0;
        for (std::size_t index = 0; index < 3U; ++index)
        {
            if (request.enabled_axes[index])
            {
                const double delta =
                    cartesian_position[index] - baseline_cartesian[index];
                selected_translation_squared += delta * delta;
            }
        }
        const double selected_translation_norm =
            std::sqrt(selected_translation_squared);
        peak_selected_axis_translation = std::max(
            peak_selected_axis_translation, selected_translation_norm);
        if (selected_translation_norm > request.max_linear_displacement)
        {
            stop_error = ComplianceError::kLimitExceeded;
            stop_message =
                "TCP 位移超过柔顺安全上限: displacement=" +
                std::to_string(selected_translation_norm) + " m, limit=" +
                std::to_string(request.max_linear_displacement) + " m";
            break;
        }

        if (now - started_at >= std::chrono::duration<double>(request.timeout))
        {
            stop_error = ComplianceError::kTimeout;
            stop_message = "柔顺运行达到配置的最长时间";
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::duration<double>(config_.monitor_period));
    }

    const bool switched_back = switch_controllers(
        {config_.trajectory_controller},
        {config_.admittance_controller});

    if (!switched_back)
    {
        status_.store(ComplianceStatus::kFault);
        set_last_result({
            false,
            ComplianceError::kControlFailed,
            0,
            stop_message + "，且切回轨迹控制器失败",
            ComplianceStatus::kFault,
            peak_absolute_wrench,
            peak_joint_displacement,
            peak_selected_axis_translation});
        return;
    }

    status_.store(ComplianceStatus::kStopped);
    const bool normal_stop = stop_error == ComplianceError::kNone;
    set_last_result({
        normal_stop,
        stop_error,
        0,
        stop_message + "，已安全切回轨迹控制器",
        ComplianceStatus::kStopped,
        peak_absolute_wrench,
        peak_joint_displacement,
        peak_selected_axis_translation});

    if (normal_stop)
    {
        RCLCPP_INFO(logger_, "%s", stop_message.c_str());
    }
    else
    {
        RCLCPP_WARN(logger_, "%s", stop_message.c_str());
    }
}

void Ros2ControlComplianceController::set_last_result(
    const ComplianceResult & result)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_result_ = result;
}

std::vector<double>
Ros2ControlComplianceController::ordered_joint_positions(
    const sensor_msgs::msg::JointState & state) const
{
    if (state.name.size() != state.position.size())
    {
        return {};
    }

    std::vector<double> ordered;
    ordered.reserve(config_.joint_names.size());
    for (const auto & expected_name : config_.joint_names)
    {
        const auto iterator = std::find(
            state.name.begin(),
            state.name.end(),
            expected_name);
        if (iterator == state.name.end())
        {
            return {};
        }
        const auto index = static_cast<std::size_t>(
            std::distance(state.name.begin(), iterator));
        ordered.push_back(state.position[index]);
    }
    return ordered;
}

}  // namespace massage_motion
