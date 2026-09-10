#ifndef CONTROL_S5_CONTROLLER__ROBOT_SHUTDOWN_UTILS_HPP_
#define CONTROL_S5_CONTROLLER__ROBOT_SHUTDOWN_UTILS_HPP_

#include <functional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace control_s5_controller
{

struct RobotShutdownOperations
{
  std::function<int()> stop_motion;
  std::function<int()> disable_servo;
  std::function<int()> disable_robot;
  std::function<int()> power_off;
  std::function<int()> logout;
};

struct RobotShutdownResult
{
  std::vector<std::pair<std::string, int>> steps;

  bool success() const
  {
    for (const auto & step : steps) {
      if (step.second != 0) {
        return false;
      }
    }
    return true;
  }

  std::string summary() const
  {
    if (success()) {
      return "robot shutdown completed";
    }
    std::ostringstream stream;
    stream << "robot shutdown failed:";
    for (const auto & step : steps) {
      if (step.second != 0) {
        stream << ' ' << step.first << '=' << step.second;
      }
    }
    return stream.str();
  }
};

struct RobotShutdownState
{
  bool motion_stopped{false};
  bool servo_disabled{false};
  bool robot_disabled{false};
  bool powered_off{false};
  bool logged_out{false};
};

inline RobotShutdownResult runRobotShutdownSequence(
  bool auto_power_off,
  bool logout,
  const RobotShutdownOperations & operations,
  RobotShutdownState & state)
{
  RobotShutdownResult result;
  const auto run = [&result](
    const char * name, bool & complete, const std::function<int()> & operation)
    {
      const int operation_result = complete ? 0 : operation();
      if (operation_result == 0) {
        complete = true;
      }
      result.steps.emplace_back(name, operation_result);
    };
  run("stop_motion", state.motion_stopped, operations.stop_motion);
  run("disable_servo", state.servo_disabled, operations.disable_servo);
  if (auto_power_off) {
    run("disable_robot", state.robot_disabled, operations.disable_robot);
    run("power_off", state.powered_off, operations.power_off);
  }
  if (logout) {
    run("logout", state.logged_out, operations.logout);
  }
  return result;
}

}  // namespace control_s5_controller

#endif  // CONTROL_S5_CONTROLLER__ROBOT_SHUTDOWN_UTILS_HPP_
