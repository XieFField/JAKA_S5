#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

class HardwareReadinessNode final : public rclcpp::Node
{
public:
    HardwareReadinessNode()
        : Node("massage_jaka_hardware_readiness"),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {
        joint_timeout_ = declare_parameter<double>("joint_state_timeout", 0.5);
        world_frame_ = declare_parameter<std::string>("world_frame", "world");
        tool_frame_ = declare_parameter<std::string>("tool_frame", "Link_06");
        action_name_ = declare_parameter<std::string>(
            "trajectory_action",
            "/jaka_s5_controller/follow_joint_trajectory");

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::SharedPtr message)
            {
                latest_joint_state_ = std::move(message);
                latest_joint_receive_time_ = std::chrono::steady_clock::now();
            });
        action_client_ = rclcpp_action::create_client<
            control_msgs::action::FollowJointTrajectory>(this, action_name_);
        diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HardwareReadinessNode::check, this));
    }

private:
    diagnostic_msgs::msg::DiagnosticStatus status(
        std::uint8_t level,
        const std::string & name,
        const std::string & message) const
    {
        diagnostic_msgs::msg::DiagnosticStatus result;
        result.level = level;
        result.name = name;
        result.hardware_id = "jaka_s5";
        result.message = message;
        return result;
    }

    void check()
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();

        const auto status_topic = action_name_ + "/_action/status";
        const auto action_publishers = get_publishers_info_by_topic(status_topic);
        if (action_publishers.size() > 1)
        {
            array.status.push_back(status(
                diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "JAKA control ownership",
                "发现多个 FollowJointTrajectory Action server"));
        }
        else if (!action_client_->action_server_is_ready())
        {
            array.status.push_back(status(
                diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "JAKA trajectory Action",
                "Action server 尚未就绪；connect=false 时这是预期状态"));
        }
        else
        {
            array.status.push_back(status(
                diagnostic_msgs::msg::DiagnosticStatus::OK,
                "JAKA trajectory Action",
                "Action 名称和唯一性检查通过"));
        }

        const std::set<std::string> expected_joints{
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"};
        if (!latest_joint_state_)
        {
            array.status.push_back(status(
                diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "JAKA joint state",
                "尚未收到 /joint_states"));
        }
        else
        {
            const std::set<std::string> actual_joints(
                latest_joint_state_->name.begin(), latest_joint_state_->name.end());
            const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                latest_joint_receive_time_).count();
            const bool finite = std::all_of(
                latest_joint_state_->position.begin(),
                latest_joint_state_->position.end(),
                [](double value) {return std::isfinite(value);});
            const bool valid = actual_joints == expected_joints &&
                latest_joint_state_->position.size() == expected_joints.size() &&
                finite && age <= joint_timeout_ &&
                rclcpp::Time(latest_joint_state_->header.stamp).nanoseconds() > 0;
            array.status.push_back(status(
                valid ? diagnostic_msgs::msg::DiagnosticStatus::OK :
                    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                "JAKA joint state",
                valid ? "关节名、数值和时间戳检查通过" :
                    "关节名、数值、数量或时间戳无效"));
        }

        const bool frames_exist = tf_buffer_._frameExists(world_frame_) &&
            tf_buffer_._frameExists(tool_frame_);
        const bool has_tf = frames_exist && tf_buffer_.canTransform(
            world_frame_, tool_frame_, tf2::TimePointZero);
        array.status.push_back(status(
            has_tf ? diagnostic_msgs::msg::DiagnosticStatus::OK :
                diagnostic_msgs::msg::DiagnosticStatus::WARN,
            "JAKA TF",
            has_tf ? "world 到工具坐标系 TF 可用" :
                "world 到工具坐标系 TF 尚不可用"));

        diagnostic_pub_->publish(array);
    }

    double joint_timeout_{0.5};
    std::string world_frame_;
    std::string tool_frame_;
    std::string action_name_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    std::chrono::steady_clock::time_point latest_joint_receive_time_{};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp_action::Client<
        control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostic_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareReadinessNode>());
    rclcpp::shutdown();
    return 0;
}
