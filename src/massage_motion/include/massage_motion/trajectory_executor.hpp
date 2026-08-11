#ifndef TRAJECTORY_EXECUTOR_HPP_
#define TRAJECTORY_EXECUTOR_HPP_

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

}


#endif // TRAJECTORY_EXECUTOR_HPP_