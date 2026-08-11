#ifndef EXECUTION_VALIDATION_HPP_
#define EXECUTION_VALIDATION_HPP_

#pragma once

#include "massage_motion/execution_types.hpp"

namespace massage_motion
{

ExecutionValidationResult validate_execution_request(
    const ExecutionRequest & request);

} // namespace massage_motion
#endif // EXECUTION_VALIDATION_HPP_