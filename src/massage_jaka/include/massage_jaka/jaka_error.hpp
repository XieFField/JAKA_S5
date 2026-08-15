#ifndef MASSAGE_JAKA__JAKA_ERROR_HPP_
#define MASSAGE_JAKA__JAKA_ERROR_HPP_

#include <cstdint>
#include <string>

namespace massage_jaka
{

enum class JakaError : std::int32_t
{
    kNone = 0,
    kFunctionCall,
    kInvalidHandler,
    kInvalidParameter,
    kCommunication,
    kInverseKinematics,
    kEmergencyStop,
    kNotPowered,
    kNotEnabled,
    kServoModeDisabled,
    kProgramRunning,
    kMotionAbnormal,
    kUnknown
};

JakaError map_jaka_error(std::int32_t sdk_code);
std::string to_string(JakaError error);

}  // namespace massage_jaka

#endif  // MASSAGE_JAKA__JAKA_ERROR_HPP_
