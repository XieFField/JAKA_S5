#ifndef MASSAGE_MOTION__MOTION_VALIDATION_HPP_
#define MASSAGE_MOTION__MOTION_VALIDATION_HPP_

#include "massage_motion/motion_types.hpp"

namespace massage_motion
{

// 在规划器操作 MoveIt 状态前，检查与具体规划后端无关的输入条件。
ValidationResult validate_motion_request(const MotionRequest & request);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__MOTION_VALIDATION_HPP_
