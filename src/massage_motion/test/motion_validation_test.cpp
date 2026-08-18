#include <limits>

#include "gtest/gtest.h"

#include "massage_motion/motion_types.hpp"
#include "massage_motion/motion_validation.hpp"

namespace massage_motion
{

TEST(MotionValidationTest, AcceptsValidPtpJointTarget)
{
    MotionRequest request;
    request.motion_type = MotionType::kPtp;

    JointTarget target;
    target.positions = {0.0, 1.0, -1.0, 0.5, 0.0, 0.0};
    request.target = target;

    // 调用被测试函数
    const ValidationResult result = validate_motion_request(request);

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.error, MotionError::kNone);
}

TEST(MotionValidationTest, AcceptsValidPtpNamedJointTarget)
{
    MotionRequest request;
    request.motion_type = MotionType::kPtp;
    request.target = NamedJointTarget{"massage_home"};

    const ValidationResult result = validate_motion_request(request);

    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.error, MotionError::kNone);
}

TEST(MotionValidationTest, RejectsEmptyPtpNamedJointTarget)
{
    MotionRequest request;
    request.motion_type = MotionType::kPtp;
    request.target = NamedJointTarget{};

    const ValidationResult result = validate_motion_request(request);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, MotionError::kInvalidRequest);
}

TEST(MotionValidationTest, RejectsZeroVelocityScale)
{
    // 先构造一个其他字段都有效的请求，只让 velocity_scale 出错。
    MotionRequest request;
    request.motion_type = MotionType::kPtp;
    request.velocity_scale = 0.0;

    JointTarget target;
    target.positions = {0.0, 1.0, -1.0, 0.5, 0.0, 0.0};
    request.target = target;

    const ValidationResult result = validate_motion_request(request);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, MotionError::kInvalidRequest);
}

TEST(MotionValidationTest, RejectsLinTargetWithoutFrameId)
{
    MotionRequest request;
    request.motion_type = MotionType::kLin;

    PoseTarget target;
    // 故意不设置 target.pose.header.frame_id。
    target.pose.pose.orientation.w = 1.0;
    request.target = target;

    const ValidationResult result = validate_motion_request(request);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.error, MotionError::kInvalidRequest);
}

}
