#include <gtest/gtest.h>

#include "massage_motion/compliance_validation.hpp"

namespace massage_motion
{
namespace
{

ComplianceRequest valid_request()
{
    ComplianceRequest request;
    request.request_id = "compliance_validation";
    request.enabled_axes[2] = true;
    request.max_absolute_wrench[2] = 5.0;
    request.max_joint_displacement = 0.05;
    request.max_linear_displacement = 0.02;
    request.timeout = 3.0;
    return request;
}

TEST(ComplianceValidation, AcceptsBoundedSingleAxisRequest)
{
    const auto result = validate_compliance_request(valid_request());
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.error, ComplianceError::kNone);
}

TEST(ComplianceValidation, RejectsRequestWithoutEnabledAxis)
{
    auto request = valid_request();
    request.enabled_axes.fill(false);

    const auto result = validate_compliance_request(request);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, ComplianceError::kInvalidRequest);
}

TEST(ComplianceValidation, RejectsMissingEnabledAxisWrenchLimit)
{
    auto request = valid_request();
    request.max_absolute_wrench[2] = 0.0;

    const auto result = validate_compliance_request(request);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, ComplianceError::kInvalidRequest);
}

TEST(ComplianceValidation, RejectsTargetAboveLimit)
{
    auto request = valid_request();
    request.target_wrench[2] = 6.0;

    const auto result = validate_compliance_request(request);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, ComplianceError::kInvalidRequest);
}

TEST(ComplianceValidation, RejectsNonPositiveMotionLimitsAndTimeout)
{
    auto request = valid_request();
    request.max_joint_displacement = 0.0;
    EXPECT_FALSE(validate_compliance_request(request).valid);

    request = valid_request();
    request.max_linear_displacement = 0.0;
    EXPECT_FALSE(validate_compliance_request(request).valid);

    request = valid_request();
    request.timeout = 0.0;
    EXPECT_FALSE(validate_compliance_request(request).valid);
}

}  // namespace
}  // namespace massage_motion
