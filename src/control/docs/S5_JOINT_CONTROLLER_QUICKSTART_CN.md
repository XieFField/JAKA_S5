# JAKA S5 Qt 六关节控制器：协作快速上手

本文档用于让协作者在 ROS 2 Humble 环境中构建并运行 JAKA S5 的 Qt 六关节控制器。

> 该程序控制真实机器人。执行任何运动前，确认实体急停可用、工作区已清空、负载和 TCP
> 参数正确，并先以低速度完成实机验证。

## 1. 功能与架构

控制器是 `control_s5_controller` ROS 2 包中的原生 Qt 5 桌面程序，不是浏览器网页。

```text
Qt 控制窗口
  |  ROS 2 服务：jog_joint / joint_move / stop_motion
  v
s5_motion_server
  |  JAKA SDK
  v
JAKA S5 真实机器人
```

控制窗口订阅 `/joint_states` 显示实际关节角度，并订阅
`/jaka_planner/joint_move_status` 显示命令速度、控制器倍率和运动状态。

## 2. 环境要求

- Ubuntu 22.04
- ROS 2 Humble Desktop
- MoveIt 2
- Qt 5 Widgets 开发包（`qtbase5-dev`）
- JAKA SDK 2.2.2，Linux AMD64
- 可访问的 JAKA S5 控制器网络

不要让 `jaka_driver` 与 `control_s5_controller/s5_motion_server` 同时连接同一台机器人。

## 3. 获取和构建

将仓库放入 ROS 2 工作空间后执行：

```bash
mkdir -p ~/jaka_ws/src
cd ~/jaka_ws/src
git clone https://github.com/JAKARobotics/jaka_ros2.git
git clone git@github.com:Yu-changqing/Zomassage_jaka.git control
cd ~/jaka_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to control_s5_controller --symlink-install
source install/setup.bash
```

## 4. 启动控制器

控制器默认不启动 RViz，也不会在启动后自动移动：

```bash
cd ~/jaka_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch control_s5_controller s5_joint_controller.launch.py \
  ip:=192.168.66.200 \
  use_rviz:=false \
  auto_move_to_initial_on_start:=false
```

将 `192.168.66.200` 改为现场机器人的实际 IP。只有在已完成低速验证后，才可显式启用
`auto_move_to_initial_on_start:=true`。

需要额外观察机器人模型时可传入 `use_rviz:=true`。RViz 仅用于可视化；Qt 控制器的
`joint_move` 命令直接由 JAKA 控制器执行，不经过 MoveIt 碰撞规划。

如果 `s5_motion_server` 已在另一个终端启动，可只打开控制窗口：

```bash
ros2 run control_s5_controller s5_joint_controller
```

## 5. 启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `ip` | `10.5.5.100` | JAKA 控制器 IP 地址 |
| `use_rviz` | `false` | 是否额外启动 MoveIt/RViz 可视化栈 |
| `auto_move_to_initial_on_start` | `false` | 收到有效关节状态后，是否自动尝试一次初始位 |
| `auto_power_off_on_exit` | `true` | 退出时是否停止、下使能并断电 |
| `joint_move_max_speed_rad_s` | `1.57` | 速度滑块 100% 对应的 SDK 速度上限 |
| `joint_move_max_acceleration_rad_s2` | `1.57` | 加速度滑块 100% 对应的 SDK 加速度上限 |
| `diagnostics_enabled` | `false` | 是否发布 JAKA 控制器诊断状态 |

## 6. 界面操作

1. 等待窗口显示“关节状态：已连接”。关节状态超过 1 秒未更新时，运动按钮会被禁用。
2. 选择“单关节”或“六关节”，并选择目标单位“度”或“弧度”。
3. 点击“同步当前姿态”将目标输入更新为当前真实关节位置。
4. 设置速度和加速度。首次真实机器人验证使用 `3%`。
5. 输入目标后点击“执行关节运动”。六关节任务会显示完整确认信息；确认前核对路径。
6. 需要小幅调整时，使用各关节的左/右点动按钮。点动步长为 `0.1` 至 `10` 度。
7. 出现异常运动时，优先使用实体急停。“停止运动”不能替代实体急停。

初始位和任务预备位由程序固定定义。六关节模式会先到任务预备位，再执行输入目标；单关节
运动和点动不会经过任务预备位。

## 7. 安全限制

- `joint_move` 的关节空间路径由 JAKA 控制器插补，不是 MoveIt 已验证的无碰撞轨迹。
- 点动调用 JAKA SDK `jog(INCR)`，同样不经过 MoveIt 碰撞规划。
- 程序会检查完整且新鲜的关节状态和 S5 关节限位，但这不能替代现场风险评估。
- 默认关闭窗口会停止运动、退出伺服模式、下使能并调用 `power_off()`；现场如需人工断电，
  请使用 `auto_power_off_on_exit:=false`。

## 8. 常见问题

### 窗口显示“关节状态：已超时”

检查机器人 IP、网络连通性、机器人是否上电和使能，以及 `s5_motion_server` 日志是否存在 SDK
连接错误。控制器在没有有效 `/joint_states` 时拒绝运动。

### 按钮不可点击

等待相应 ROS 服务就绪。`执行关节运动` 需要 `/jaka_planner/joint_move`，点动需要
`/jaka_planner/jog_joint`。运动、点动和关机操作互斥，前一操作未结束时会禁用其他命令。

### 构建时找不到 Qt

确认已安装 Qt 5 Widgets 开发包：

```bash
sudo apt-get install qtbase5-dev
```

随后重新执行 `colcon build --packages-up-to control_s5_controller --symlink-install`。

## 9. 提交到协作仓库

控制器功能至少应一起提交以下内容：

```text
README.md
LICENSE
docs/S5_JOINT_CONTROLLER_QUICKSTART_CN.md
docs/JAKA_S5_REAL_ROBOT_TEST.md
docs/JAKA_S5_SIM_TEST.md
control_jaka_msgs/CMakeLists.txt
control_jaka_msgs/package.xml
control_jaka_msgs/msg/JointMoveStatus.msg
control_jaka_msgs/msg/RobotDiagnostic.msg
control_jaka_msgs/srv/JogJoint.srv
control_jaka_msgs/srv/MoveJoint.srv
control_jaka_msgs/srv/SetRapidRate.srv
control_jaka_sdk_vendor/
control_s5_controller/CMakeLists.txt
control_s5_controller/package.xml
control_s5_controller/include/control_s5_controller/s5_joint_utils.hpp
control_s5_controller/launch/s5_joint_controller.launch.py
control_s5_controller/src/s5_motion_server.cpp
control_s5_controller/src/s5_joint_controller.cpp
```

不要提交 `build/`、`install/`、`log/`、本机环境文件、机器人实际 IP、账号密码或许可证密钥。
推送前使用 `git diff --cached` 审核暂存内容，确保不会包含其他协作者未确认的删除或改动。

## 10. 进一步资料

- 完整真机验收清单：[JAKA_S5_REAL_ROBOT_TEST.md](JAKA_S5_REAL_ROBOT_TEST.md)
- 仿真检查：[JAKA_S5_SIM_TEST.md](JAKA_S5_SIM_TEST.md)
- 官方 ROS 2 说明位于同一工作空间的 `jaka_ros2` 仓库中。
