# JAKA S5 仿真测试

## 1. 构建工作空间

在包含本仓库的 ROS 2 工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 2. 启动 RViz 仿真

```bash
ros2 launch jaka_s5_moveit_config demo.launch.py use_rviz_sim:=true
```

保持该终端运行，然后在新终端中加载工作空间并检查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 topic echo --once /joint_states
```

## 3. 验证 MoveIt 规划

可以在 RViz 中设置目标姿态后点击 `Plan` 检查规划。也可运行：

```bash
ros2 run jaka_planner moveit_test --ros-args -p model:=s5
```

核对以下内容：

- MoveIt 中的机器人模型为 JAKA S5。
- `/joint_states` 含 `joint_1` 至 `joint_6`，数值为有限弧度值。
- 规划能正常成功，碰撞检测和关节限位正常生效。
- 关闭 launch 后没有残留 ROS 节点。

## 限制

`s5_joint_controller` 的 JOG 按钮依赖真实机器人服务，不是 Gazebo
控制器。仿真时不要启动会连接真实机器人的 `jaka_driver` 或
`moveit_server`。

---

> **以下为后续开发产物**

新版 `s5_joint_controller` 通过真实机器人 `control_s5_motion_server` 的 `/jaka_planner/joint_move`
服务下发运动，不能用于 Gazebo。仿真验证继续使用 MoveIt、RViz 和 Gazebo 控制器；仿真时
不要启动 `control_s5_motion_server` 或新版真机控制窗口。
