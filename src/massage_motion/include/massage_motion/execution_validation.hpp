#ifndef EXECUTION_VALIDATION_HPP_
#define EXECUTION_VALIDATION_HPP_

#pragma once
#include "massage_motion/execution_types.hpp"

namespace massage_motion
{

class ITrajectoryExecutor
{
public:
    virtual ~ITrajectoryExecutor() = default;

    virtual ExecutionResult execute(
        const ExecutionRequest & request) = 0;

    virtual bool cancel() = 0;

    virtual ExecutionStatus status() const = 0;
};

} // namespace massage_motion
#endif // EXECUTION_VALIDATION_HPP_