#ifndef MASSAGE_BRINGUP__DEFAULT_TARGETS_HPP_
#define MASSAGE_BRINGUP__DEFAULT_TARGETS_HPP_

#include <array>
#include <string>
#include <vector>

#include "massage_motion/motion_types.hpp"

namespace massage_bringup
{

inline constexpr std::array<const char *, 6> kJakaS5JointNames{
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

// 项目启动后的默认业务待机位，不表示编码器标定零位。
inline constexpr std::array<double, 6> kMassageHomeJointPositions{
  -3.160921066038505,
  1.6080484013290657,
  -2.6790895150928824,
  2.6720179543589007,
  0.027910401738015497,
  -2.4753825550678896};

/*
-3.160921066038505,
  1.6080484013290657,
  -2.6790895150928824,
  2.6720179543589007,
  0.027910401738015497,
  -2.4753825550678896
  第一版的初始待机位置

 */

inline massage_motion::JointTarget make_massage_home_target()
{
  return {
    std::vector<double>(
      kMassageHomeJointPositions.begin(), kMassageHomeJointPositions.end())};
}

}  // namespace massage_bringup

#endif  // MASSAGE_BRINGUP__DEFAULT_TARGETS_HPP_
