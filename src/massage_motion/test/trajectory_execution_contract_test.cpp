#include <limits>

#include "gtest/gtest.h"

#include "massage_motion/trajectory_execution_contract.hpp"

namespace massage_motion
{
namespace
{

TEST(TrajectoryExecutionContractTest, AcceptsAlignedAndConservativeController)
{
  EXPECT_TRUE(validate_trajectory_execution_contract(
    0.002, 15.0, 0.002, 15.0).valid);
  EXPECT_TRUE(validate_trajectory_execution_contract(
    0.0008, 15.0, 0.002, 10.0).valid);
}

TEST(TrajectoryExecutionContractTest, RejectsPrematureControllerDeadline)
{
  const auto result = validate_trajectory_execution_contract(
    0.002, 2.0, 0.002, 15.0);

  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.message.find("小于任务执行余量"), std::string::npos);
}

TEST(TrajectoryExecutionContractTest, RejectsControllerToleranceAboveTaskLimit)
{
  const auto result = validate_trajectory_execution_contract(
    0.003, 15.0, 0.002, 15.0);

  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.message.find("大于任务终点容差"), std::string::npos);
}

TEST(TrajectoryExecutionContractTest, RejectsInvalidNumbers)
{
  EXPECT_FALSE(validate_trajectory_execution_contract(
    std::numeric_limits<double>::quiet_NaN(), 15.0, 0.002, 15.0).valid);
  EXPECT_FALSE(validate_trajectory_execution_contract(
    0.002, 0.0, 0.002, 15.0).valid);
  EXPECT_FALSE(validate_trajectory_execution_contract(
    0.002, 15.0, 0.0, 15.0).valid);
  EXPECT_FALSE(validate_trajectory_execution_contract(
    0.002, 15.0, 0.002, -1.0).valid);
}

}  // namespace
}  // namespace massage_motion
