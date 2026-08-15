#ifndef MASSAGE_TASK__PRESS_TASK_STATE_MACHINE_HPP_
#define MASSAGE_TASK__PRESS_TASK_STATE_MACHINE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "massage_motion/compliance_controller.hpp"
#include "massage_motion/motion_planner.hpp"
#include "massage_motion/trajectory_executor.hpp"
#include "massage_task/press_task_types.hpp"

namespace massage_task
{

class IContactSceneManager
{
public:
    virtual ~IContactSceneManager() = default;
    virtual bool prepare() = 0;
    virtual bool allow_tool_contact(bool allowed) = 0;
    virtual bool restore() = 0;
};

bool offset_pose_along_normal(
    const massage_motion::PoseTarget & source,
    const geometry_msgs::msg::Vector3 & normal,
    double distance,
    massage_motion::PoseTarget & target,
    std::string & error_message);

class PressTaskStateMachine
{
public:
    PressTaskStateMachine(
        std::shared_ptr<massage_motion::IMotionPlanner> planner,
        std::shared_ptr<massage_motion::ITrajectoryExecutor> executor,
        std::shared_ptr<massage_motion::IComplianceController> compliance,
        std::shared_ptr<IContactSceneManager> scene_manager);

    PressTaskResult run(const PressTaskRequest & request);
    bool cancel();
    bool reset();
    PressTaskState state() const;

private:
    std::shared_ptr<massage_motion::IMotionPlanner> planner_;
    std::shared_ptr<massage_motion::ITrajectoryExecutor> executor_;
    std::shared_ptr<massage_motion::IComplianceController> compliance_;
    std::shared_ptr<IContactSceneManager> scene_manager_;
    std::mutex run_mutex_;
    std::atomic<PressTaskState> state_{PressTaskState::kIdle};
    std::atomic_bool cancel_requested_{false};
};

}  // namespace massage_task

#endif  // MASSAGE_TASK__PRESS_TASK_STATE_MACHINE_HPP_
