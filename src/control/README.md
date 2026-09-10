# Control JAKA S5

Independent ROS 2 packages for the Control JAKA S5 Qt joint controller. These
packages live under `JAKA_S5/src/control`; the official
`JAKARobotics/jaka_ros2` repository remains an external workspace dependency.

## Repository Layout

- `control_jaka_msgs`: controller-specific messages and services.
- `control_jaka_sdk_vendor`: JAKA SDK 2.2.2 headers and runtime library wrapper.
- `control_s5_controller`: SDK motion server, Qt GUI, and launch files.
- `docs`: Chinese setup, simulation, and real-robot acceptance documents.

## Workspace Setup

Clone the official driver and `JAKA_S5` workspaces next to each other:

```text
robot_project/
├── jaka_ros2/
└── massage_robot_ws/
    └── src/control/
```

```bash
mkdir -p ~/robot_project
cd ~/robot_project
git clone https://github.com/JAKARobotics/jaka_ros2.git jaka_ros2
git clone https://github.com/XieFField/JAKA_S5.git massage_robot_ws

cd ~/robot_project/massage_robot_ws
source /opt/ros/humble/setup.bash
source ~/robot_project/jaka_ros2/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to control_s5_controller --symlink-install
source install/setup.bash
```

## Start

```bash
ros2 launch control_s5_controller s5_joint_controller.launch.py \
  ip:=192.168.66.200 \
  use_rviz:=false \
  auto_move_to_initial_on_start:=false
```

The motion server owns the only JAKA SDK connection. Do not start the official
`jaka_driver` against the same robot while this controller is running.

Real robot motion can cause injury or equipment damage. Start acceptance at 3%
speed and acceleration, keep the physical emergency stop available, and verify
the complete path before issuing a command.
