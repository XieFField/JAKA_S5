#include <limits>

#include "gtest/gtest.h"
#include "massage_motion/execution_types.hpp"
#include "massage_motion/execution_validation.hpp"

// 有效的 joint trajectory + 有效 timeout -> 通过
TEST(ExecutionValidationTest, AcceptsValidRequest)
{
    massage_motion::ExecutionRequest request;
    request.request_id = "test_request";
    request.timeout = 5.0;

    // 构造一个有效的 RobotTrajectory
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.points.resize(1); // 添加一个点
    request.robot_trajectory = trajectory;

    const massage_motion::ExecutionValidationResult result =
        massage_motion::validate_execution_request(request);

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.error, massage_motion::ExecutionError::kNone);
}

// timeout = 0.0 -> kInvalidRequest
TEST(ExecutionValidationTest, RejectsZeroTimeout)
{
    massage_motion::ExecutionRequest request;
    request.request_id = "test_request";
    request.timeout = 0.0;

    // 构造一个有效的 RobotTrajectory
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.points.resize(1); // 添加一个点
    request.robot_trajectory = trajectory;

    const massage_motion::ExecutionValidationResult result =
        massage_motion::validate_execution_request(request);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, massage_motion::ExecutionError::kInvalidRequest);
}

// timeout = NaN -> kInvalidRequest
TEST(ExecutionValidationTest, RejectsNaNTimeout)
{
    massage_motion::ExecutionRequest request;
    request.request_id = "test_request";
    request.timeout = std::numeric_limits<double>::quiet_NaN();

    // 构造一个有效的 RobotTrajectory
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.points.resize(1); // 添加一个点
    request.robot_trajectory = trajectory;

    const massage_motion::ExecutionValidationResult result =
        massage_motion::validate_execution_request(request);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, massage_motion::ExecutionError::kInvalidRequest);
}

// joint 和 multi-DOF 轨迹都为空 -> kEmptyTrajectory
TEST(ExecutionValidationTest, RejectsEmptyTrajectory)
{
    massage_motion::ExecutionRequest request;
    request.request_id = "test_request";
    request.timeout = 5.0;

    // 构造一个空的 RobotTrajectory
    moveit_msgs::msg::RobotTrajectory trajectory;
    request.robot_trajectory = trajectory;

    const massage_motion::ExecutionValidationResult result =
        massage_motion::validate_execution_request(request);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, massage_motion::ExecutionError::kEmptyTrajectory);
}