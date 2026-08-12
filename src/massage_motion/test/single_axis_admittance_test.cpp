#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "massage_motion/single_axis_admittance.hpp"

namespace massage_motion
{
namespace
{

TEST(SingleAxisAdmittanceTest, RejectsInvalidParameters)
{
    SingleAxisAdmittanceParameters parameters;
    parameters.mass = 0.0;

    EXPECT_THROW(
        SingleAxisAdmittance controller(parameters),
        std::invalid_argument);
}

TEST(SingleAxisAdmittanceTest, ZeroForceKeepsZeroState)
{
    const SingleAxisAdmittanceParameters parameters{2.0, 4.0, 8.0};
    SingleAxisAdmittance controller(parameters);

    const auto update = controller.update(0.0, 0.01);

    ASSERT_TRUE(update.valid) << update.message;
    EXPECT_DOUBLE_EQ(update.state.position, 0.0);
    EXPECT_DOUBLE_EQ(update.state.velocity, 0.0);
    EXPECT_DOUBLE_EQ(update.state.acceleration, 0.0);
}

TEST(SingleAxisAdmittanceTest, ConstantForceProducesExpectedFirstStep)
{
    const SingleAxisAdmittanceParameters parameters{2.0, 0.0, 0.0};
    SingleAxisAdmittance controller(parameters);

    const auto update = controller.update(4.0, 0.1);

    ASSERT_TRUE(update.valid) << update.message;
    EXPECT_NEAR(update.state.acceleration, 2.0, 1e-12);
    EXPECT_NEAR(update.state.velocity, 0.2, 1e-12);
    EXPECT_NEAR(update.state.position, 0.02, 1e-12);
}

TEST(SingleAxisAdmittanceTest, InvalidInputDoesNotChangeState)
{
    SingleAxisAdmittance controller({});
    ASSERT_TRUE(controller.update(1.0, 0.1).valid);
    const auto state_before_error = controller.state();

    const auto update = controller.update(
        std::numeric_limits<double>::quiet_NaN(),
        0.1);

    EXPECT_FALSE(update.valid);
    EXPECT_DOUBLE_EQ(controller.state().position, state_before_error.position);
    EXPECT_DOUBLE_EQ(controller.state().velocity, state_before_error.velocity);
    EXPECT_DOUBLE_EQ(
        controller.state().acceleration,
        state_before_error.acceleration);
}

TEST(SingleAxisAdmittanceTest, ResetClearsDynamicState)
{
    SingleAxisAdmittance controller({});
    ASSERT_TRUE(controller.update(1.0, 0.1).valid);

    controller.reset();

    EXPECT_DOUBLE_EQ(controller.state().position, 0.0);
    EXPECT_DOUBLE_EQ(controller.state().velocity, 0.0);
    EXPECT_DOUBLE_EQ(controller.state().acceleration, 0.0);
}

}  // namespace
}  // namespace massage_motion
