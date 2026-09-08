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
  kPress,
  kKnead,
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

struct PressPathRequest
{
  // 接触基准点。路径从该点开始，只沿 surface_normal 的反方向压入，
  // 每周期回到该接触点，不越过表面进入自由空间。
  geometry_msgs::msg::PoseStamped contact_pose;
  double surface_normal_x{0.0};
  double surface_normal_y{0.0};
  double surface_normal_z{1.0};
  double stroke{0.0};
  double cycle_duration{0.0};
  std::size_t cycles{0U};
  double sample_period{0.0};
  double maximum_speed{0.0};
};

struct KneadPathRequest
{
  // center_pose 是揉动圆心；initial_direction_xy 决定路径首点。
  geometry_msgs::msg::PoseStamped center_pose;
  double initial_direction_x{1.0};
  double initial_direction_y{0.0};
  double radius{0.0};
  double cycle_duration{0.0};
  std::size_t cycles{0U};
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

  // 生成从接触基准向表面内的半余弦周期按压路径。
  static TechniquePathResult generate_press(const PressPathRequest & request);

  // 在 center_pose 所属参考系的 XY 平面生成定高、定姿态闭合圆路径。
  static TechniquePathResult generate_knead(const KneadPathRequest & request);
};

std::string to_string(TechniquePathType type);
std::string to_string(TechniquePathError error);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__TECHNIQUE_PATH_GENERATOR_HPP_
