#ifndef MASSAGE_MOTION__TRAJECTORY_ENDPOINT_ERROR_HPP_
#define MASSAGE_MOTION__TRAJECTORY_ENDPOINT_ERROR_HPP_

#include <string>
#include <vector>

#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace massage_motion
{

struct JointPositionError
{
    std::string joint_name;
    double target_position{0.0};
    double actual_position{0.0};
    double absolute_error{0.0};
};

struct TrajectoryEndpointErrorResult
{
    bool valid{false};
    std::string message;
    std::vector<JointPositionError> joint_errors;
    double max_absolute_error{0.0};
};

TrajectoryEndpointErrorResult calculate_trajectory_endpoint_error(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const sensor_msgs::msg::JointState & actual_state);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__TRAJECTORY_ENDPOINT_ERROR_HPP_
