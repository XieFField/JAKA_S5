#include "massage_jaka/jaka_error.hpp"

namespace massage_jaka
{

JakaError map_jaka_error(std::int32_t sdk_code)
{
    switch (sdk_code)
    {
        case 0: return JakaError::kNone;
        case 2: return JakaError::kFunctionCall;
        case -1: return JakaError::kInvalidHandler;
        case -2: return JakaError::kInvalidParameter;
        case -3: return JakaError::kCommunication;
        case -4: return JakaError::kInverseKinematics;
        case -5: return JakaError::kEmergencyStop;
        case -6: return JakaError::kNotPowered;
        case -7: return JakaError::kNotEnabled;
        case -8: return JakaError::kServoModeDisabled;
        case -10: return JakaError::kProgramRunning;
        case -12: return JakaError::kMotionAbnormal;
        default: return JakaError::kUnknown;
    }
}

std::string to_string(JakaError error)
{
    switch (error)
    {
        case JakaError::kNone: return "none";
        case JakaError::kFunctionCall: return "function_call_error";
        case JakaError::kInvalidHandler: return "invalid_handler";
        case JakaError::kInvalidParameter: return "invalid_parameter";
        case JakaError::kCommunication: return "communication_error";
        case JakaError::kInverseKinematics: return "inverse_kinematics_error";
        case JakaError::kEmergencyStop: return "emergency_stop";
        case JakaError::kNotPowered: return "not_powered";
        case JakaError::kNotEnabled: return "not_enabled";
        case JakaError::kServoModeDisabled: return "servo_mode_disabled";
        case JakaError::kProgramRunning: return "program_running";
        case JakaError::kMotionAbnormal: return "motion_abnormal";
        case JakaError::kUnknown: return "unknown";
    }
    return "unknown";
}

}  // namespace massage_jaka
