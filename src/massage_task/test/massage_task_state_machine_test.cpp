#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "massage_task/massage_task_state_machine.hpp"

namespace
{

massage_motion::PlanResult plan_result(double duration = 0.0)
{
  massage_motion::PlanResult result;
  result.success = true;
  result.message = "planned";
  result.trajectory.joint_trajectory.joint_names = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = std::vector<double>(6, 0.0);
  point.velocities = std::vector<double>(6, 0.0);
  point.time_from_start.sec = static_cast<std::int32_t>(duration);
  point.time_from_start.nanosec = static_cast<std::uint32_t>(
    (duration - static_cast<double>(point.time_from_start.sec)) * 1.0e9);
  result.trajectory.joint_trajectory.points.push_back(point);
  return result;
}

class FakeMotionPlanner final : public massage_motion::IMotionPlanner
{
public:
  std::deque<massage_motion::PlanResult> results;
  std::vector<massage_motion::MotionRequest> requests;

  massage_motion::PlanResult plan(
    const massage_motion::MotionRequest & request) override
  {
    requests.push_back(request);
    if (results.empty())
    {
      return plan_result();
    }
    auto result = results.front();
    results.pop_front();
    return result;
  }
};

class FakeTechniquePlanner final : public massage_task::ITechniqueTrajectoryPlanner
{
public:
  massage_motion::PlanResult result{plan_result()};
  std::vector<massage_motion::TechniquePath> paths;
  std::vector<bool> align_to_current_tcp;

  massage_motion::PlanResult plan(
    const massage_motion::TechniquePath & path,
    const moveit_msgs::msg::RobotState &,
    bool align_to_current) override
  {
    paths.push_back(path);
    align_to_current_tcp.push_back(align_to_current);
    return result;
  }
};

class FakeAlignmentValidator final :
  public massage_task::IToolAlignmentValidator
{
public:
  std::deque<massage_motion::ToolAlignmentMetrics> trajectory_results;
  std::deque<massage_motion::ToolAlignmentMetrics> current_results;
  std::vector<massage_motion::ToolAlignmentGateConfig> trajectory_configs;
  std::vector<massage_motion::ToolAlignmentGateConfig> current_configs;

  static massage_motion::ToolAlignmentMetrics accepted()
  {
    massage_motion::ToolAlignmentMetrics result;
    result.valid = true;
    result.accepted = true;
    result.message = "aligned";
    result.sample_count = 1U;
    result.minimum_signed_alignment = 1.0;
    return result;
  }

  massage_motion::ToolAlignmentMetrics validate_trajectory(
    const moveit_msgs::msg::RobotTrajectory &,
    const massage_motion::ToolAlignmentGateConfig & config) override
  {
    trajectory_configs.push_back(config);
    if (trajectory_results.empty()) return accepted();
    auto result = trajectory_results.front();
    trajectory_results.pop_front();
    return result;
  }

  massage_motion::ToolAlignmentMetrics validate_current(
    const std::string &,
    const massage_motion::ToolAlignmentGateConfig & config) override
  {
    current_configs.push_back(config);
    if (current_results.empty()) return accepted();
    auto result = current_results.front();
    current_results.pop_front();
    return result;
  }
};

class FakeReturnSafetyValidator final :
  public massage_task::IReturnTrajectorySafetyValidator
{
public:
  massage_task::ReturnTrajectorySafetyResult result{
    true, true, "safe", 0.5, 0.5};
  std::vector<massage_task::ReturnTrajectorySafetyConfig> configs;
  int calls{0};

  massage_task::ReturnTrajectorySafetyResult validate(
    const moveit_msgs::msg::RobotTrajectory &,
    const massage_task::ReturnTrajectorySafetyConfig & config) override
  {
    ++calls;
    configs.push_back(config);
    return result;
  }
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
    if (!results.empty())
    {
      auto result = results.front();
      results.pop_front();
      return result;
    }
    massage_motion::ExecutionResult result;
    result.success = true;
    result.status = massage_motion::ExecutionStatus::kSucceeded;
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
  massage_motion::ComplianceCapabilities supported{true, true};
  massage_motion::ComplianceResult start_result{
    true, massage_motion::ComplianceError::kNone, 0, "started",
    massage_motion::ComplianceStatus::kActive};
  massage_motion::ComplianceResult stop_result{
    true, massage_motion::ComplianceError::kNone, 0, "stopped",
    massage_motion::ComplianceStatus::kStopped};
  std::atomic<massage_motion::ComplianceStatus> current{
    massage_motion::ComplianceStatus::kIdle};
  double normal_force{1.0};
  int start_calls{0};
  int stop_calls{0};
  int update_calls{0};
  int reset_calls{0};

  massage_motion::ComplianceCapabilities capabilities() const override
  {
    return supported;
  }

  massage_motion::ComplianceResult start(
    const massage_motion::ComplianceRequest &) override
  {
    ++start_calls;
    current.store(start_result.status);
    return start_result;
  }

  massage_motion::ComplianceResult stop() override
  {
    ++stop_calls;
    current.store(stop_result.status);
    return stop_result;
  }

  bool update_reference(const massage_motion::ComplianceReference &) override
  {
    ++update_calls;
    return true;
  }

  massage_motion::ComplianceFeedback feedback() const override
  {
    massage_motion::ComplianceFeedback feedback;
    feedback.status = current.load();
    feedback.stale = false;
    feedback.wrench[2] = normal_force;
    feedback.joint_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    feedback.joint_positions = std::vector<double>(6, 0.0);
    return feedback;
  }

  bool reset() override
  {
    ++reset_calls;
    current.store(massage_motion::ComplianceStatus::kIdle);
    return true;
  }

  massage_motion::ComplianceStatus status() const override
  {
    return current.load();
  }
};

class FakeFtManager final : public massage_task::IForceTorqueManager
{
public:
  massage_task::ForceTorqueResult result{true, "zeroed", 0.01};
  int calls{0};

  massage_task::ForceTorqueResult zero_and_validate() override
  {
    ++calls;
    return result;
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

massage_task::MassageTaskRequest valid_request(
  massage_motion::TechniquePathType type)
{
  massage_task::MassageTaskRequest request;
  request.task_id = "massage_test";
  request.technique = type;
  request.standby_target = massage_motion::JointTarget{
    std::vector<double>(6, 0.0)};
  request.contact_pose.header.frame_id = "world";
  request.contact_pose.pose.position.z = 0.3;
  request.contact_pose.pose.orientation.w = 1.0;
  request.surface_normal.z = 1.0;
  request.surface_tangent.y = 1.0;
  request.execute = true;
  request.contact_wait_timeout = 1.0e-6;
  request.force_ramp_timeout = 1.0e-6;
  request.compliance_request.request_id = "compliance";
  request.compliance_request.enabled_axes[2] = true;
  request.compliance_request.max_absolute_wrench[2] = 5.0;
  request.compliance_request.max_joint_displacement = 0.2;
  request.compliance_request.max_linear_displacement = 0.03;
  request.compliance_request.timeout = 5.0;
  request.push.direction_y = 1.0;
  request.push.length = 0.05;
  request.push.speed = 0.01;
  request.push.sample_period = 0.1;
  request.push.maximum_speed = 0.02;
  request.press.stroke = 0.004;
  request.press.cycle_duration = 1.0;
  request.press.cycles = 1U;
  request.press.sample_period = 0.1;
  request.press.maximum_speed = 0.02;
  request.knead.radius = 0.01;
  request.knead.cycle_duration = 4.0;
  request.knead.cycles = 1U;
  request.knead.sample_period = 0.1;
  request.knead.maximum_speed = 0.02;
  return request;
}

struct Fixture
{
  std::shared_ptr<FakeMotionPlanner> motion{
    std::make_shared<FakeMotionPlanner>()};
  std::shared_ptr<FakeTechniquePlanner> technique{
    std::make_shared<FakeTechniquePlanner>()};
  std::shared_ptr<FakeAlignmentValidator> alignment{
    std::make_shared<FakeAlignmentValidator>()};
  std::shared_ptr<FakeReturnSafetyValidator> return_safety{
    std::make_shared<FakeReturnSafetyValidator>()};
  std::shared_ptr<FakeExecutor> executor{std::make_shared<FakeExecutor>()};
  std::shared_ptr<FakeCompliance> compliance{
    std::make_shared<FakeCompliance>()};
  std::shared_ptr<FakeFtManager> ft{std::make_shared<FakeFtManager>()};
  std::shared_ptr<FakeSceneManager> scene{
    std::make_shared<FakeSceneManager>()};

  massage_task::MassageTaskStateMachine machine()
  {
    return massage_task::MassageTaskStateMachine(
      motion, technique, alignment, return_safety, executor, compliance, ft, scene);
  }
};

}  // namespace

TEST(MassageTaskStateMachineTest, ExecutesRequiredStateOrder)
{
  Fixture fixture;
  auto machine = fixture.machine();
  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kPush));

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.contact_detected);
  EXPECT_TRUE(result.target_force_reached);
  EXPECT_TRUE(result.recovery_succeeded);
  const std::vector<massage_task::MassageTaskState> expected{
    massage_task::MassageTaskState::kReady,
    massage_task::MassageTaskState::kMoveStandby,
    massage_task::MassageTaskState::kMoveWorkReady,
    massage_task::MassageTaskState::kVerifyToolAlignment,
    massage_task::MassageTaskState::kMovePreContact,
    massage_task::MassageTaskState::kZeroAndValidateFt,
    massage_task::MassageTaskState::kGuardedContactEntry,
    massage_task::MassageTaskState::kContactConfirmed,
    massage_task::MassageTaskState::kRampNormalForce,
    massage_task::MassageTaskState::kExecuteTechnique,
    massage_task::MassageTaskState::kReleaseNormalForce,
    massage_task::MassageTaskState::kRetreat,
    massage_task::MassageTaskState::kReturnOverhead,
    massage_task::MassageTaskState::kMoveSafeReturnExit,
    massage_task::MassageTaskState::kComplete};
  EXPECT_EQ(result.state_trace, expected);
}

TEST(MassageTaskStateMachineTest, EstablishesDownwardPoseBeforeLinearApproach)
{
  Fixture fixture;
  auto machine = fixture.machine();
  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kPush));

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_GE(fixture.motion->requests.size(), 5U);
  EXPECT_EQ(fixture.motion->requests[0].motion_type, massage_motion::MotionType::kPtp);
  EXPECT_EQ(fixture.motion->requests[1].motion_type, massage_motion::MotionType::kPtp);
  EXPECT_EQ(fixture.motion->requests[2].motion_type, massage_motion::MotionType::kLin);
  const auto & ready = std::get<massage_motion::PoseTarget>(
    fixture.motion->requests[1].target).pose.pose;
  const auto & precontact = std::get<massage_motion::PoseTarget>(
    fixture.motion->requests[2].target).pose.pose;
  EXPECT_NEAR(ready.position.z - precontact.position.z, 0.05, 1.0e-12);
  EXPECT_DOUBLE_EQ(ready.orientation.x, precontact.orientation.x);
  EXPECT_DOUBLE_EQ(ready.orientation.y, precontact.orientation.y);
  EXPECT_DOUBLE_EQ(ready.orientation.z, precontact.orientation.z);
  EXPECT_DOUBLE_EQ(ready.orientation.w, precontact.orientation.w);
  ASSERT_FALSE(fixture.alignment->trajectory_configs.empty());
  EXPECT_FALSE(fixture.alignment->trajectory_configs.front().inspect_all_samples);
  ASSERT_FALSE(fixture.alignment->current_configs.empty());
  EXPECT_NEAR(
    fixture.alignment->current_configs.front().desired_tool_z_world[2],
    -1.0, 1.0e-12);
}

TEST(MassageTaskStateMachineTest, RejectsCurrentToolMisalignmentBeforePrecontact)
{
  Fixture fixture;
  auto rejected = FakeAlignmentValidator::accepted();
  rejected.accepted = false;
  rejected.message = "points upward";
  fixture.alignment->current_results.push_back(rejected);
  auto machine = fixture.machine();

  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kPush));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(
    result.primary_error,
    massage_task::MassageTaskError::kToolAlignmentFailed);
  EXPECT_EQ(fixture.ft->calls, 0);
  EXPECT_EQ(fixture.compliance->start_calls, 0);
  ASSERT_GE(fixture.motion->requests.size(), 2U);
  EXPECT_EQ(fixture.motion->requests[1].request_id, "massage_test_work_ready");
}

TEST(MassageTaskStateMachineTest, DispatchesAllTechniqueGenerators)
{
  for (const auto type : {
      massage_motion::TechniquePathType::kPush,
      massage_motion::TechniquePathType::kPress,
      massage_motion::TechniquePathType::kKnead})
  {
    Fixture fixture;
    auto machine = fixture.machine();
    const auto result = machine.run(valid_request(type));
    ASSERT_TRUE(result.success) << static_cast<int>(type);
    ASSERT_EQ(fixture.technique->paths.size(), 1U);
    EXPECT_EQ(fixture.technique->paths.front().type, type);
    EXPECT_FALSE(fixture.technique->paths.front().points.empty());
  }
}

TEST(MassageTaskStateMachineTest, PushRepeatsThreeTimesViaPrecontact)
{
  Fixture fixture;
  auto request = valid_request(massage_motion::TechniquePathType::kPush);
  request.push_repetitions = 3U;
  auto machine = fixture.machine();

  const auto result = machine.run(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.completed_repetitions, 3U);
  EXPECT_EQ(result.completed_technique_cycles, 3U);
  EXPECT_EQ(fixture.technique->paths.size(), 3U);
  EXPECT_EQ(fixture.compliance->start_calls, 3);
  EXPECT_EQ(fixture.compliance->stop_calls, 3);
  EXPECT_EQ(
    std::count(
      result.state_trace.begin(), result.state_trace.end(),
      massage_task::MassageTaskState::kExecuteTechnique),
    3);
  EXPECT_EQ(
    std::count(
      result.state_trace.begin(), result.state_trace.end(),
      massage_task::MassageTaskState::kInterCycleRetreat),
    2);
  EXPECT_EQ(
    std::count(
      result.state_trace.begin(), result.state_trace.end(),
      massage_task::MassageTaskState::kInterCycleReturn),
    2);

  std::size_t return_requests = 0U;
  for (const auto & motion : fixture.motion->requests)
  {
    if (motion.request_id.find("inter_cycle_return") != std::string::npos)
    {
      ++return_requests;
      const auto & pose = std::get<massage_motion::PoseTarget>(motion.target).pose;
      EXPECT_NEAR(pose.pose.position.x, request.contact_pose.pose.position.x, 1.0e-12);
      EXPECT_NEAR(pose.pose.position.y, request.contact_pose.pose.position.y, 1.0e-12);
      EXPECT_NEAR(
        pose.pose.position.z,
        request.contact_pose.pose.position.z + request.precontact_distance,
        1.0e-12);
    }
  }
  EXPECT_EQ(return_requests, 2U);
}

TEST(MassageTaskStateMachineTest, PressAndKneadUseOneContactSessionForThreeCycles)
{
  for (const auto type : {
      massage_motion::TechniquePathType::kPress,
      massage_motion::TechniquePathType::kKnead})
  {
    Fixture fixture;
    auto request = valid_request(type);
    request.press.cycles = 3U;
    request.knead.cycles = 3U;
    auto machine = fixture.machine();

    const auto result = machine.run(request);

    ASSERT_TRUE(result.success) << result.message;
    EXPECT_EQ(result.completed_repetitions, 1U);
    EXPECT_EQ(result.completed_technique_cycles, 3U);
    EXPECT_EQ(fixture.compliance->start_calls, 1);
    EXPECT_EQ(fixture.compliance->stop_calls, 1);
    ASSERT_EQ(fixture.technique->paths.size(), 1U);
    const double expected_duration = type == massage_motion::TechniquePathType::kPress ?
      3.0 : 12.0;
    EXPECT_NEAR(
      fixture.technique->paths.front().duration, expected_duration, 1.0e-12);
  }
}

TEST(MassageTaskStateMachineTest, RejectsUnsafeReturnBeforeStandbyExecution)
{
  Fixture fixture;
  fixture.return_safety->result = {
    true, false, "below patient surface", 0.29, 0.1};
  auto machine = fixture.machine();
  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kPush));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, massage_task::MassageTaskError::kRecoveryFailed);
  EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kExecutionFailed);
  EXPECT_EQ(fixture.return_safety->calls, 1);
  EXPECT_EQ(fixture.executor->execute_calls, 6);
  ASSERT_EQ(fixture.return_safety->configs.size(), 1U);
  EXPECT_NEAR(fixture.return_safety->configs.front().minimum_tool_z, 0.0, 1.0e-12);
  EXPECT_NEAR(
    fixture.return_safety->configs.front().minimum_surface_clearance,
    0.005, 1.0e-12);
}

TEST(MassageTaskStateMachineTest, RealExecutionRequiresConfirmationBeforeAnyAction)
{
  Fixture fixture;
  auto machine = fixture.machine();
  auto request = valid_request(massage_motion::TechniquePathType::kPush);
  request.execution_environment = "real";
  request.parameters_confirmed = false;

  const auto result = machine.run(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, massage_task::MassageTaskError::kAuthorizationRequired);
  EXPECT_TRUE(fixture.motion->requests.empty());
  EXPECT_TRUE(fixture.technique->paths.empty());
  EXPECT_EQ(fixture.executor->execute_calls, 0);
  EXPECT_EQ(fixture.compliance->start_calls, 0);
  EXPECT_EQ(fixture.ft->calls, 0);
  EXPECT_EQ(fixture.scene->prepare_calls, 0);
}

TEST(MassageTaskStateMachineTest, UnsupportedReferenceTrackingFailsBeforeAnyAction)
{
  Fixture fixture;
  fixture.compliance->supported.reference_tracking = false;
  auto machine = fixture.machine();
  auto request = valid_request(massage_motion::TechniquePathType::kPush);
  request.execution_environment = "real";
  request.parameters_confirmed = true;

  const auto result = machine.run(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(
    result.error,
    massage_task::MassageTaskError::kUnsupportedComplianceMode);
  EXPECT_TRUE(fixture.motion->requests.empty());
  EXPECT_TRUE(fixture.technique->paths.empty());
  EXPECT_EQ(fixture.executor->execute_calls, 0);
  EXPECT_EQ(fixture.compliance->start_calls, 0);
  EXPECT_EQ(fixture.ft->calls, 0);
  EXPECT_EQ(fixture.scene->prepare_calls, 0);
}

TEST(MassageTaskStateMachineTest, FreeSpaceUsesOnlyNormalTrajectoryExecution)
{
  for (const auto type : {
      massage_motion::TechniquePathType::kPush,
      massage_motion::TechniquePathType::kPress,
      massage_motion::TechniquePathType::kKnead})
  {
    Fixture fixture;
    auto request = valid_request(type);
    request.execution_mode = massage_task::MassageExecutionMode::kFreeSpace;
    auto machine = fixture.machine();

    const auto result = machine.run(request);

    ASSERT_TRUE(result.success) << result.message;
    EXPECT_EQ(fixture.ft->calls, 0);
    EXPECT_EQ(fixture.scene->prepare_calls, 0);
    EXPECT_EQ(fixture.scene->allow_calls, 0);
    EXPECT_EQ(fixture.scene->restore_calls, 0);
    EXPECT_EQ(fixture.compliance->start_calls, 0);
    EXPECT_EQ(fixture.compliance->stop_calls, 0);
    EXPECT_EQ(fixture.compliance->update_calls, 0);
    ASSERT_EQ(fixture.technique->paths.size(), 1U);
    ASSERT_EQ(fixture.technique->align_to_current_tcp.size(), 1U);
    EXPECT_TRUE(fixture.technique->align_to_current_tcp.front());
    EXPECT_GT(fixture.executor->execute_calls, 0);
  }
}

TEST(MassageTaskStateMachineTest, FreeSpacePushPreservesThreeRepetitions)
{
  Fixture fixture;
  auto request = valid_request(massage_motion::TechniquePathType::kPush);
  request.execution_mode = massage_task::MassageExecutionMode::kFreeSpace;
  request.push_repetitions = 3U;
  auto machine = fixture.machine();

  const auto result = machine.run(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.completed_repetitions, 3U);
  EXPECT_EQ(result.completed_technique_cycles, 3U);
  EXPECT_EQ(fixture.technique->paths.size(), 3U);
  EXPECT_EQ(fixture.compliance->start_calls, 0);
  EXPECT_EQ(
    std::count(
      result.state_trace.begin(), result.state_trace.end(),
      massage_task::MassageTaskState::kMoveTechniqueStart),
    3);
}

TEST(MassageTaskStateMachineTest, FreeSpacePressAndKneadPreserveThreeCycles)
{
  for (const auto type : {
      massage_motion::TechniquePathType::kPress,
      massage_motion::TechniquePathType::kKnead})
  {
    Fixture fixture;
    auto request = valid_request(type);
    request.execution_mode = massage_task::MassageExecutionMode::kFreeSpace;
    request.press.cycles = 3U;
    request.knead.cycles = 3U;
    auto machine = fixture.machine();

    const auto result = machine.run(request);

    ASSERT_TRUE(result.success) << result.message;
    EXPECT_EQ(result.completed_repetitions, 1U);
    EXPECT_EQ(result.completed_technique_cycles, 3U);
    EXPECT_EQ(fixture.technique->paths.size(), 1U);
    EXPECT_EQ(fixture.compliance->start_calls, 0);
  }
}

TEST(MassageTaskStateMachineTest, FreeSpaceTechniqueFailureRunsSafeReturn)
{
  Fixture fixture;
  auto request = valid_request(massage_motion::TechniquePathType::kPush);
  request.execution_mode = massage_task::MassageExecutionMode::kFreeSpace;
  for (int index = 0; index < 4; ++index)
  {
    massage_motion::ExecutionResult success;
    success.success = true;
    success.status = massage_motion::ExecutionStatus::kSucceeded;
    fixture.executor->results.push_back(success);
  }
  massage_motion::ExecutionResult failed;
  failed.status = massage_motion::ExecutionStatus::kFailed;
  failed.message = "technique controller failed";
  fixture.executor->results.push_back(failed);
  auto machine = fixture.machine();

  const auto result = machine.run(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kExecutionFailed);
  EXPECT_TRUE(result.recovery_attempted);
  EXPECT_TRUE(result.recovery_succeeded);
  EXPECT_EQ(fixture.compliance->start_calls, 0);
  EXPECT_EQ(fixture.ft->calls, 0);
  EXPECT_GT(fixture.executor->execute_calls, 5);
}

TEST(MassageTaskStateMachineTest, FreeSpaceExecutionFailureStillUsesSafeReturn)
{
  Fixture fixture;
  auto request = valid_request(massage_motion::TechniquePathType::kPush);
  request.execution_mode = massage_task::MassageExecutionMode::kFreeSpace;
  for (int index = 0; index < 4; ++index)
  {
    massage_motion::ExecutionResult success;
    success.success = true;
    success.status = massage_motion::ExecutionStatus::kSucceeded;
    fixture.executor->results.push_back(success);
  }
  massage_motion::ExecutionResult failure;
  failure.status = massage_motion::ExecutionStatus::kFailed;
  failure.message = "technique execution failed";
  fixture.executor->results.push_back(failure);
  auto machine = fixture.machine();

  const auto result = machine.run(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kExecutionFailed);
  EXPECT_TRUE(result.recovery_attempted);
  EXPECT_TRUE(result.recovery_succeeded);
  EXPECT_EQ(fixture.compliance->start_calls, 0);
  EXPECT_GT(fixture.return_safety->calls, 0);
  EXPECT_GT(fixture.executor->execute_calls, 5);
}

TEST(MassageTaskStateMachineTest, PlanOnlyChainsRouteWithoutRuntimeDependencies)
{
  Fixture fixture;
  auto request = valid_request(massage_motion::TechniquePathType::kKnead);
  request.execution_mode = massage_task::MassageExecutionMode::kPlanOnly;
  request.execution_environment = "real";
  request.execute = false;
  request.parameters_confirmed = false;
  massage_task::MassageTaskStateMachine machine(
    fixture.motion, fixture.technique, fixture.alignment,
    fixture.return_safety, nullptr, nullptr, nullptr, nullptr);

  const auto result = machine.run(request);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(fixture.executor->execute_calls, 0);
  EXPECT_EQ(fixture.ft->calls, 0);
  EXPECT_EQ(fixture.scene->prepare_calls, 0);
  EXPECT_EQ(fixture.compliance->start_calls, 0);
  ASSERT_EQ(fixture.technique->align_to_current_tcp.size(), 1U);
  EXPECT_FALSE(fixture.technique->align_to_current_tcp.front());
  ASSERT_GT(fixture.motion->requests.size(), 1U);
  EXPECT_FALSE(fixture.motion->requests.front().start_state.has_value());
  for (std::size_t index = 1U; index < fixture.motion->requests.size(); ++index)
  {
    EXPECT_TRUE(fixture.motion->requests[index].start_state.has_value());
  }
}

TEST(MassageTaskStateMachineTest, RealFreeSpaceStillRequiresConfirmation)
{
  Fixture fixture;
  auto request = valid_request(massage_motion::TechniquePathType::kPress);
  request.execution_mode = massage_task::MassageExecutionMode::kFreeSpace;
  request.execution_environment = "real";
  request.parameters_confirmed = false;
  auto machine = fixture.machine();

  const auto result = machine.run(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, massage_task::MassageTaskError::kAuthorizationRequired);
  EXPECT_TRUE(fixture.motion->requests.empty());
  EXPECT_EQ(fixture.executor->execute_calls, 0);
}

TEST(MassageTaskStateMachineTest, ReportsMissingContactAndRecovers)
{
  Fixture fixture;
  fixture.compliance->normal_force = 0.0;
  auto machine = fixture.machine();
  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kPress));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kContactNotDetected);
  EXPECT_TRUE(result.recovery_succeeded);
  EXPECT_EQ(fixture.compliance->stop_calls, 1);
  EXPECT_EQ(fixture.scene->restore_calls, 1);
}

TEST(MassageTaskStateMachineTest, ReportsForceLimitAndRecovers)
{
  Fixture fixture;
  fixture.compliance->normal_force = 5.1;
  auto machine = fixture.machine();
  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kKnead));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kForceLimitExceeded);
  EXPECT_TRUE(result.recovery_succeeded);
}

TEST(MassageTaskStateMachineTest, StopsBeforeContactWhenFtValidationFails)
{
  Fixture fixture;
  fixture.ft->result = {false, "bias too high", 0.5};
  auto machine = fixture.machine();
  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kPush));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kFtValidationFailed);
  EXPECT_EQ(fixture.compliance->start_calls, 0);
  EXPECT_TRUE(result.recovery_succeeded);
}

TEST(MassageTaskStateMachineTest, ReportsPlanningAndExecutionFailures)
{
  {
    Fixture fixture;
    massage_motion::PlanResult failed;
    failed.message = "no plan";
    fixture.motion->results.push_back(failed);
    auto machine = fixture.machine();
    const auto result = machine.run(valid_request(
      massage_motion::TechniquePathType::kPush));
    EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kPlanningFailed);
    EXPECT_EQ(fixture.executor->execute_calls, 0);
  }
  {
    Fixture fixture;
    massage_motion::ExecutionResult failed;
    failed.status = massage_motion::ExecutionStatus::kFailed;
    failed.message = "controller failed";
    fixture.executor->results.push_back(failed);
    auto machine = fixture.machine();
    const auto result = machine.run(valid_request(
      massage_motion::TechniquePathType::kPush));
    EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kExecutionFailed);
  }
}

TEST(MassageTaskStateMachineTest, ReportsComplianceStartFailure)
{
  Fixture fixture;
  fixture.compliance->start_result = {
    false, massage_motion::ComplianceError::kControlFailed, 0,
    "switch failed", massage_motion::ComplianceStatus::kFault};
  auto machine = fixture.machine();
  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kPress));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kComplianceFailed);
  EXPECT_TRUE(result.recovery_succeeded);
}

TEST(MassageTaskStateMachineTest, CancellationStillRunsRecovery)
{
  Fixture fixture;
  auto machine = fixture.machine();
  auto request = valid_request(massage_motion::TechniquePathType::kPush);
  request.target_normal_force = 2.0;
  request.force_ramp_timeout = 0.5;
  auto future = std::async(std::launch::async, [&]() {return machine.run(request);});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (machine.state() != massage_task::MassageTaskState::kRampNormalForce &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(machine.state(), massage_task::MassageTaskState::kRampNormalForce);
  EXPECT_TRUE(machine.cancel());
  const auto result = future.get();
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.primary_error, massage_task::MassageTaskError::kCanceled);
  EXPECT_EQ(result.final_state, massage_task::MassageTaskState::kCanceled);
  EXPECT_TRUE(result.recovery_succeeded);
}

TEST(MassageTaskStateMachineTest, RecoveryFailureIsReportedSeparately)
{
  Fixture fixture;
  fixture.scene->restore_result = false;
  auto machine = fixture.machine();
  const auto result = machine.run(valid_request(
    massage_motion::TechniquePathType::kPush));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, massage_task::MassageTaskError::kRecoveryFailed);
  EXPECT_FALSE(result.recovery_succeeded);
}
