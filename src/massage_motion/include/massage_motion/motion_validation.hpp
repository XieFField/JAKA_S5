/**
 * @file motion_validation.hpp
 * @brief 只进行与具体机械臂无关的静态校验。
 *        TF 是否存在、关节数量是否匹配、
 *        目标是否可达，应留给后续 MoveIt 规划器判断
 */

#ifndef MASSAGE_MOTION__MOTION_VALIDATION_HPP_
#define MASSAGE_MOTION__MOTION_VALIDATION_HPP_

#include "massage_motion/motion_types.hpp"

namespace massage_motion
{

// 在规划器操作 MoveIt 状态前，检查与具体规划后端无关的输入条件。
ValidationResult validate_motion_request(const MotionRequest & request);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__MOTION_VALIDATION_HPP_
