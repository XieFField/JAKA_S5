#ifndef MINIMAL_TASK_STATE_MACHINE_HPP_ 
#define MINIMAL_TASK_STATE_MACHINE_HPP_

#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "massage_motion/motion_planner.hpp"
#include "massage_motion/trajectory_executor.hpp"
#include "massage_task/task_types.hpp"

namespace massage_task
{

class MinimalTaskStateMachine
{
public:
    MinimalTaskStateMachine(
        std::shared_ptr<massage_motion::IMotionPlanner> planner,
        std::shared_ptr<massage_motion::ITrajectoryExecutor> executor);

    TaskResult run(const TaskRequest & request);

    bool cancel();
    bool reset();

    TaskState state() const;

private:
    std::shared_ptr<massage_motion::IMotionPlanner> planner_;
    std::shared_ptr<massage_motion::ITrajectoryExecutor> executor_;

    std::mutex run_mutex_;
    std::atomic<TaskState> state_{TaskState::kIdle};
};


}

#endif  // MINIMAL_TASK_STATE_MACHINE_HPP_