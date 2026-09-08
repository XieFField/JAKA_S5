#include <memory>

#include "gtest/gtest.h"

#include "massage_motion/hybrid_trajectory_executor.hpp"

namespace
{

class FakeExecutor final : public massage_motion::ITrajectoryExecutor
{
public:
  massage_motion::ExecutionResult execute(
    const massage_motion::ExecutionRequest &) override
  {
    ++calls;
    return {
      true, massage_motion::ExecutionError::kNone, 0, "ok",
      massage_motion::ExecutionStatus::kSucceeded};
  }
  bool cancel() override {return true;}
  massage_motion::ExecutionStatus status() const override
  {
    return massage_motion::ExecutionStatus::kSucceeded;
  }
  int calls{0};
};

TEST(HybridTrajectoryExecutorTest, RoutesOnlySemanticPtpToNativeBackend)
{
  auto native = std::make_shared<FakeExecutor>();
  auto moveit = std::make_shared<FakeExecutor>();
  massage_motion::HybridTrajectoryExecutor hybrid(native, moveit);

  massage_motion::ExecutionRequest ptp;
  ptp.has_motion_semantics = true;
  ptp.motion_type = massage_motion::MotionType::kPtp;
  EXPECT_TRUE(hybrid.execute(ptp).success);

  massage_motion::ExecutionRequest lin;
  lin.has_motion_semantics = true;
  lin.motion_type = massage_motion::MotionType::kLin;
  EXPECT_TRUE(hybrid.execute(lin).success);

  massage_motion::ExecutionRequest unspecified;
  EXPECT_TRUE(hybrid.execute(unspecified).success);

  EXPECT_EQ(native->calls, 1);
  EXPECT_EQ(moveit->calls, 2);
}

}  // namespace
