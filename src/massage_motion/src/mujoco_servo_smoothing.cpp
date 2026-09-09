#include <cstddef>
#include <vector>

#include <moveit/online_signal_smoothing/smoothing_base_class.h>
#include <pluginlib/class_list_macros.hpp>

namespace massage_motion
{

// Humble Servo filters absolute feedback-relative positions before deriving
// velocity. A position low-pass therefore attenuates a sustained velocity
// command. MuJoCo's force-limited dynamics provide the actuator response instead.
class MujocoServoSmoothing : public online_signal_smoothing::SmoothingBaseClass
{
public:
  bool initialize(
    rclcpp::Node::SharedPtr, moveit::core::RobotModelConstPtr,
    std::size_t num_joints) override
  {
    num_joints_ = num_joints;
    return num_joints_ > 0;
  }

  bool doSmoothing(std::vector<double> & positions) override
  {
    return positions.size() == num_joints_;
  }

  bool reset(const std::vector<double> & positions) override
  {
    return positions.size() == num_joints_;
  }

private:
  std::size_t num_joints_{0};
};

}  // namespace massage_motion

PLUGINLIB_EXPORT_CLASS(
  massage_motion::MujocoServoSmoothing, online_signal_smoothing::SmoothingBaseClass)
