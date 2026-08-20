#ifndef MASSAGE_MOTION__TECHNIQUE_PATH_GENERATOR_HPP_
#define MASSAGE_MOTION__TECHNIQUE_PATH_GENERATOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace massage_motion
{

enum class TechniquePathType : std::uint8_t
{
  kPush = 0,
};

enum class TechniquePathError : std::int32_t
{
  kNone = 0,
  kInvalidRequest,
  kPathTooLarge,
};

struct TechniquePathPoint
{
  geometry_msgs::msg::Pose pose;
  double time_from_start{0.0};
};

// 几何路径只描述名义工具参考，不包含 MoveIt 轨迹、关节命令或力控参数。
struct TechniquePath
{
  TechniquePathType type{TechniquePathType::kPush};
  std::string reference_frame;
  builtin_interfaces::msg::Time reference_stamp;
  std::vector<TechniquePathPoint> points;
  double duration{0.0};
  double nominal_speed{0.0};
};

struct PushPathRequest
{
  geometry_msgs::msg::PoseStamped start_pose;
  double direction_x{0.0};
  double direction_y{0.0};
  double length{0.0};
  double speed{0.0};
  double sample_period{0.0};
  double maximum_speed{0.0};
};

struct TechniquePathResult
{
  bool success{false};
  TechniquePathError error{TechniquePathError::kNone};
  std::string message;
  TechniquePath path;
};

class TechniquePathGenerator
{
public:
  // 在 start_pose 所属参考系的 XY 平面生成定高、定姿态直线路径。
  static TechniquePathResult generate_push(const PushPathRequest & request);
};

std::string to_string(TechniquePathType type);
std::string to_string(TechniquePathError error);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__TECHNIQUE_PATH_GENERATOR_HPP_
