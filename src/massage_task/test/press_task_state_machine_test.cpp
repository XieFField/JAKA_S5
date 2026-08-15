#include <chrono>
#include <deque>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "massage_task/press_task_state_machine.hpp"

namespace
{

massage_motion::PlanResult successful_plan()
{
    massage_motion::PlanResult result;
    result.success = true;
    result.trajectory.joint_trajectory.joint_names = {
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
    point.velocities = std::vector<double>(6, 0.0);
    result.trajectory.joint_trajectory.points.push_back(point);
    return result;
}

massage_motion::ExecutionResult successful_execution()
{
    massage_motion::ExecutionResult result;
    result.success = true;
    result.status = massage_motion::ExecutionStatus::kSucceeded;
    return result;
}

class FakePlanner final : public massage_motion::IMotionPlanner
{
public:
    std::deque<massage_motion::PlanResult> results;
    int calls{0};

    massage_motion::PlanResult plan(
        const massage_motion::MotionRequest & request) override
    {
        ++calls;
        requests.push_back(request);
        if (results.empty())
        {
            return successful_plan();
        }
        const auto result = results.front();
        results.pop_front();
        return result;
    }

    std::vector<massage_motion::MotionRequest> requests;
};

class FakeExecutor final : public massage_motion::ITrajectoryExecutor
{
public:
    std::deque<massage_motion::ExecutionResult> results;
    int execute_calls{0};
    int cancel_calls{0};

    massage_motion::ExecutionResult execute(
        const massage_motion::ExecutionRequest &) override
    {
        ++execute_calls;
        if (results.empty())
        {
            return successful_execution();
        }
        const auto result = results.front();
        results.pop_front();
        return result;
    }

    bool cancel() override
    {
        ++cancel_calls;
        return true;
    }

    massage_motion::ExecutionStatus status() const override
    {
        return massage_motion::ExecutionStatus::kIdle;
    }
};

class FakeCompliance final : public massage_motion::IComplianceController
{
public:
    massage_motion::ComplianceResult start_result{
        true,
        massage_motion::ComplianceError::kNone,
        0,
        "started",
        massage_motion::ComplianceStatus::kActive};
    massage_motion::ComplianceResult stop_result{
        true,
        massage_motion::ComplianceError::kNone,
        0,
        "stopped",
        massage_motion::ComplianceStatus::kStopped};
    massage_motion::ComplianceFeedback next_feedback;
    massage_motion::ComplianceStatus current_status{
        massage_motion::ComplianceStatus::kIdle};
    int start_calls{0};
    int stop_calls{0};
    int update_calls{0};
    int reset_calls{0};

    FakeCompliance()
    {
        next_feedback.status = massage_motion::ComplianceStatus::kActive;
        next_feedback.stale = false;
    }

    massage_motion::ComplianceResult start(
        const massage_motion::ComplianceRequest &) override
    {
        ++start_calls;
        current_status = start_result.status;
        return start_result;
    }

    massage_motion::ComplianceResult stop() override
    {
        ++stop_calls;
        current_status = stop_result.status;
        return stop_result;
    }

    bool update_reference(
        const massage_motion::ComplianceReference &) override
    {
        ++update_calls;
        return true;
    }

    massage_motion::ComplianceFeedback feedback() const override
    {
        auto feedback = next_feedback;
        feedback.status = current_status;
        return feedback;
    }

    bool reset() override
    {
        ++reset_calls;
        current_status = massage_motion::ComplianceStatus::kIdle;
        return true;
    }

    massage_motion::ComplianceStatus status() const override
    {
        return current_status;
    }
};

class FakeSceneManager final : public massage_task::IContactSceneManager
{
public:
    bool prepare_result{true};
    bool allow_result{true};
    bool restore_result{true};
    int prepare_calls{0};
    int allow_calls{0};
    int restore_calls{0};

    bool prepare() override
    {
        ++prepare_calls;
        return prepare_result;
    }

    bool allow_tool_contact(bool) override
    {
        ++allow_calls;
        return allow_result;
    }

    bool restore() override
    {
        ++restore_calls;
        return restore_result;
    }
};

massage_task::PressTaskRequest valid_request()
{
    massage_task::PressTaskRequest request;
    request.task_id = "press_test";
    request.contact_target.pose.header.frame_id = "world";
    request.contact_target.pose.pose.position.x = 0.75;
    request.contact_target.pose.pose.position.y = -0.0205;
    request.contact_target.pose.pose.position.z = 0.106382;
    request.surface_normal.y = 1.0;
    request.hold_duration = 0.0;
    request.contact_wait_timeout = 1e-6;
    request.compliance_request.request_id = "compliance_test";
    request.compliance_request.enabled_axes[2] = true;
    request.compliance_request.max_absolute_wrench[2] = 1.0;
    request.compliance_request.max_joint_displacement = 0.2;
    request.compliance_request.max_linear_displacement = 0.03;
    request.compliance_request.timeout = 5.0;
    return request;
}

struct Fixture
{
    std::shared_ptr<FakePlanner> planner{std::make_shared<FakePlanner>()};
    std::shared_ptr<FakeExecutor> executor{std::make_shared<FakeExecutor>()};
    std::shared_ptr<FakeCompliance> compliance{
        std::make_shared<FakeCompliance>()};
    std::shared_ptr<FakeSceneManager> scene{
        std::make_shared<FakeSceneManager>()};
};

}  // namespace

TEST(PressTaskGeometryTest, OffsetsPoseAlongNormalizedNormal)
{
    massage_motion::PoseTarget source;
    source.pose.header.frame_id = "world";
    source.pose.pose.position.y = -0.02;
    geometry_msgs::msg::Vector3 normal;
    normal.y = 2.0;
    massage_motion::PoseTarget target;
    std::string error;

    ASSERT_TRUE(massage_task::offset_pose_along_normal(
        source, normal, 0.04, target, error));
    EXPECT_NEAR(target.pose.pose.position.y, 0.02, 1e-12);
    EXPECT_TRUE(error.empty());
}

TEST(PressTaskGeometryTest, RejectsZeroNormal)
{
    massage_motion::PoseTarget source;
    source.pose.header.frame_id = "world";
    geometry_msgs::msg::Vector3 normal;
    massage_motion::PoseTarget target;
    std::string error;

    EXPECT_FALSE(massage_task::offset_pose_along_normal(
        source, normal, 0.04, target, error));
    EXPECT_FALSE(error.empty());
}

TEST(PressTaskStateMachineTest, CompletesPressAndRetreat)
{
    Fixture fixture;
    fixture.compliance->next_feedback.wrench[2] = 0.2;
    massage_task::PressTaskStateMachine machine(
        fixture.planner, fixture.executor,
        fixture.compliance, fixture.scene);

    const auto result = machine.run(valid_request());

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.final_state, massage_task::PressTaskState::kCompleted);
    EXPECT_TRUE(result.contact_detected);
    EXPECT_TRUE(result.recovery_succeeded);
    EXPECT_EQ(fixture.planner->calls, 4);
    EXPECT_EQ(fixture.executor->execute_calls, 3);
    EXPECT_EQ(fixture.compliance->start_calls, 1);
    EXPECT_EQ(fixture.compliance->stop_calls, 1);
    EXPECT_EQ(fixture.scene->allow_calls, 2);
    EXPECT_EQ(fixture.scene->restore_calls, 1);
}

TEST(PressTaskStateMachineTest, StopsAfterPlanningFailure)
{
    Fixture fixture;
    massage_motion::PlanResult failure;
    failure.message = "planning failed";
    fixture.planner->results.push_back(failure);
    massage_task::PressTaskStateMachine machine(
        fixture.planner, fixture.executor,
        fixture.compliance, fixture.scene);

    const auto result = machine.run(valid_request());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.primary_error, massage_task::PressTaskError::kPlanningFailed);
    EXPECT_EQ(fixture.executor->execute_calls, 0);
    EXPECT_EQ(fixture.scene->restore_calls, 1);
}

TEST(PressTaskStateMachineTest, RecoversAfterExecutionFailure)
{
    Fixture fixture;
    massage_motion::ExecutionResult failure;
    failure.status = massage_motion::ExecutionStatus::kFailed;
    failure.message = "execution failed";
    fixture.executor->results.push_back(successful_execution());
    fixture.executor->results.push_back(failure);
    fixture.executor->results.push_back(successful_execution());
    massage_task::PressTaskStateMachine machine(
        fixture.planner, fixture.executor,
        fixture.compliance, fixture.scene);

    const auto result = machine.run(valid_request());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.primary_error, massage_task::PressTaskError::kExecutionFailed);
    EXPECT_TRUE(result.recovery_succeeded);
    EXPECT_EQ(fixture.executor->execute_calls, 3);
}

TEST(PressTaskStateMachineTest, RecoversAfterComplianceStartFailure)
{
    Fixture fixture;
    fixture.compliance->start_result.success = false;
    fixture.compliance->start_result.error =
        massage_motion::ComplianceError::kBackendUnavailable;
    fixture.compliance->start_result.status =
        massage_motion::ComplianceStatus::kFault;
    fixture.compliance->start_result.message = "backend unavailable";
    massage_task::PressTaskStateMachine machine(
        fixture.planner, fixture.executor,
        fixture.compliance, fixture.scene);

    const auto result = machine.run(valid_request());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.primary_error, massage_task::PressTaskError::kComplianceFailed);
    EXPECT_TRUE(result.recovery_succeeded);
    EXPECT_EQ(fixture.executor->execute_calls, 3);
}

TEST(PressTaskStateMachineTest, ReportsMissingContact)
{
    Fixture fixture;
    fixture.compliance->next_feedback.wrench[2] = 0.0;
    massage_task::PressTaskStateMachine machine(
        fixture.planner, fixture.executor,
        fixture.compliance, fixture.scene);

    const auto result = machine.run(valid_request());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(
        result.primary_error,
        massage_task::PressTaskError::kContactNotDetected);
    EXPECT_TRUE(result.recovery_succeeded);
}

TEST(PressTaskStateMachineTest, ReportsForceLimitAndStillRetreats)
{
    Fixture fixture;
    fixture.compliance->next_feedback.wrench[2] = 1.2;
    auto request = valid_request();
    request.hold_duration = 0.01;
    massage_task::PressTaskStateMachine machine(
        fixture.planner, fixture.executor,
        fixture.compliance, fixture.scene);

    const auto result = machine.run(request);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.primary_error, massage_task::PressTaskError::kComplianceFailed);
    EXPECT_TRUE(result.recovery_succeeded);
    EXPECT_GE(result.peak_absolute_wrench, 1.2);
}

TEST(PressTaskStateMachineTest, CancelsCompliantPressAndStillRetreats)
{
    Fixture fixture;
    fixture.compliance->next_feedback.wrench[2] = 0.2;
    auto request = valid_request();
    request.hold_duration = 0.5;
    massage_task::PressTaskStateMachine machine(
        fixture.planner, fixture.executor,
        fixture.compliance, fixture.scene);

    auto future = std::async(
        std::launch::async,
        [&machine, request]() {return machine.run(request);});
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(1);
    while (machine.state() != massage_task::PressTaskState::kCompliantPress &&
        std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    ASSERT_EQ(machine.state(), massage_task::PressTaskState::kCompliantPress);
    ASSERT_TRUE(machine.cancel());
    const auto result = future.get();

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.primary_error, massage_task::PressTaskError::kCanceled);
    EXPECT_EQ(result.final_state, massage_task::PressTaskState::kCanceled);
    EXPECT_TRUE(result.recovery_succeeded);
    EXPECT_GE(fixture.compliance->stop_calls, 1);
    EXPECT_EQ(fixture.scene->restore_calls, 1);
}

TEST(PressTaskStateMachineTest, ReportsRetreatFailureSeparately)
{
    Fixture fixture;
    fixture.compliance->next_feedback.wrench[2] = 0.2;
    fixture.executor->results.push_back(successful_execution());
    fixture.executor->results.push_back(successful_execution());
    massage_motion::ExecutionResult retreat_failure;
    retreat_failure.status = massage_motion::ExecutionStatus::kFailed;
    retreat_failure.message = "retreat failed";
    fixture.executor->results.push_back(retreat_failure);
    massage_task::PressTaskStateMachine machine(
        fixture.planner, fixture.executor,
        fixture.compliance, fixture.scene);

    const auto result = machine.run(valid_request());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error, massage_task::PressTaskError::kRecoveryFailed);
    EXPECT_FALSE(result.recovery_succeeded);
}
