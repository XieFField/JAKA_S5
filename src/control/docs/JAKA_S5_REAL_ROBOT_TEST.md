# JAKA S5 实机准备位姿测试

> 本仓库的独立控制器从第 11 节开始验收；第 4 至第 9 节保留为官方
> `jaka_ros2` MoveIt 链路的可选对照流程。两条真机控制链路不得同时启动。

机器人 IP：`192.168.66.200`

操作记录时间：`2026-09-06（星期日）13:56`

> 本文用于控制真实机械臂。首次执行必须清空工作区、降低全局速度，并确保急停按钮随时可用。

## 1. 停止仿真和其他驱动

在 Gazebo 启动终端按 `Ctrl+C`，并确认没有遗留的仿真节点。

```bash
source /opt/ros/humble/setup.bash
source /home/banana/jaka_ws/install/setup.bash
ros2 node list
```

真机 MoveIt 模式下不要同时运行 `jaka_driver`，即不要执行
`ros2 launch jaka_driver robot_start.launch.py ip:=192.168.66.200`。

## 2. 检查网络

电脑和机器人必须处于同一网段，且不能使用相同 IP。

```bash
ip -brief address
ping -c 4 192.168.66.200
```

必须收到机器人回复后才能继续。

## 3. 实机安全检查

- 确认机械臂、底座和末端工具安装牢固。
- 在 JAKA App 中正确设置负载、重心和 TCP。
- 清空机械臂整个运动范围内的人员、线缆和障碍物。
- 确认实体急停按钮有效，并安排人员随时准备操作。
- 将 JAKA App 全局速度限制调到较低值，首次建议 `5%`。
- 确认当前姿态到准备位姿的路径不会碰撞现场设备。

## 4. 启动真机 MoveIt 服务器

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /home/banana/jaka_ws/install/setup.bash

ros2 launch jaka_planner moveit_server.launch.py \
  ip:=192.168.66.200 \
  model:=s5
```

`moveit_server` 启动时会自动登录、上电并使能机械臂。执行命令前必须完成安全检查。

保持终端 1 运行，并检查是否出现 SDK、通信、上电或使能错误。

## 5. 启动 RViz 真机界面

终端 2：

```bash
source /opt/ros/humble/setup.bash
source /home/banana/jaka_ws/install/setup.bash

ros2 launch jaka_s5_moveit_config demo.launch.py \
  use_rviz_sim:=false
```

不要使用 `demo_gazebo.launch.py`。RViz 中显示的当前姿态必须与实机一致。

## 6. 验证状态，不执行运动

终端 3：

```bash
source /opt/ros/humble/setup.bash
source /home/banana/jaka_ws/install/setup.bash

ros2 topic echo --once /joint_states
ros2 action list | grep jaka_s5_controller
```

应当存在以下 Action：

```text
/jaka_s5_controller/follow_joint_trajectory
```

将 `/joint_states` 中的六个关节值与 JAKA App 对照。名称与位置必须一一对应；位置单位为弧度。

## 7. 仅规划准备位姿

`s5_ready_pose` 的默认目标现为任务预备位：
`[-179.753, 90.057, -90.199, 90.196, 91.724, -64.680]°`。

以下命令只规划，不会移动实机：

```bash
ros2 launch jaka_planner s5_ready_pose.launch.py \
  execute:=false velocity_scaling:=0.03 acceleration_scaling:=0.03
```

只有看到以下内容才表示规划成功：

```text
Planning succeeded. No motion was executed
```

规划成功不代表现场路径一定安全。MoveIt 无法识别未加入规划场景的真实桌面、墙壁、人员和线缆。

## 8. 低速执行准备位姿

再次清场，确认急停人员就位后执行：

```bash
ros2 launch jaka_planner s5_ready_pose.launch.py \
  execute:=true velocity_scaling:=0.03 acceleration_scaling:=0.03
```

成功时终端会显示：

```text
Execute request success!
The S5 reached the requested ready pose.
```

运动过程中不要关闭网络、控制柜或运行终端。发生异常运动时优先按实体急停。

## 9. 监视执行状态

控制器状态：

```bash
ros2 topic echo \
  /jaka_s5_controller/follow_joint_trajectory/_action/status
```

实际关节反馈：

```bash
ros2 topic echo /joint_states
```

Action 状态码：`1` 已接受，`2` 执行中，`4` 成功，`5` 已取消，`6` 已中止。

## 10. 测试结束与下使能

1. 确认机械臂已经停止运动。
2. 关闭 S5 控制窗口，或在统一 launch 终端按 `Ctrl+C`。
3. 等待终端依次显示停止运动、退出伺服、下使能、断电和 SDK 退出成功。
4. 在 JAKA App 中确认 `enabled=0` 且 `powered_on=0`。
5. 按现场规程处理控制柜电源。

默认 `auto_power_off_on_exit:=true`。若关机步骤报错或窗口因超时保持打开，必须立即查看终端中的 SDK 错误，并使用 JAKA App 完成下使能和断电。软件关机不能替代实体急停。

## 常见问题

### 找不到 `libjakaAPI.so`

重新编译并加载工作空间：

```bash
cd /home/banana/jaka_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jaka_planner --symlink-install
source install/setup.bash
```

### RViz 姿态与实机不一致

不要执行运动。检查 `moveit_server` 是否正常连接 `192.168.66.200`，并重新读取 `/joint_states`。

### 控制器没有收到命令

```bash
ros2 action info /jaka_s5_controller/follow_joint_trajectory
```

应同时看到 Action client 和 Action server。


####################
记录所有ros话题：
/home/banana/Logs_r/record_all_topics.sh
文件保存在Logs_r下xxxx
####################
查看下使能
/home/banana/Logs_r/record_disable_reason.sh
文件保存在Logs_r/disable_XXXXXXXX

---

> **以下为后续开发产物**

## 11. S5 六关节控制器低速点动验收

重新编译时必须同时生成新增的点动服务接口：

```bash
cd /home/banana/jaka_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to control_s5_controller --symlink-install
source install/setup.bash
ros2 launch control_s5_controller s5_joint_controller.launch.py \
  ip:=192.168.66.200 use_rviz:=true
```

- 将界面速度设为 `5%`，点动单次步长设为 `0.1°`。
- J1 至 J6 每行左箭头为负向，右箭头为正向；每次只单击一个按钮。
- 每次点动后等待界面显示完成，并核对该关节的角度和弧度反馈。
- 依次检查六个关节的正、负方向，其他五个关节不应出现指令性移动。
- 在一次点动未完成时确认重复提交已锁定，并验证“停止运动”可中断点动。

SDK 点动不经过 MoveIt 碰撞规划。点动前必须确认对应方向安全，并保持实体急停随时可用。

## 12. 软件退出自动关机验收

1. 使用默认的 `auto_power_off_on_exit:=true` 启动统一 launch。
2. 分别在无运动、轨迹运动和点动过程中关闭控制窗口，确认系统先停止运动，再依次退出伺服模式、下使能并调用 `power_off()`。
3. 重启后在统一 launch 终端按 `Ctrl+C`，确认执行相同的关机流程并退出 SDK 登录。
4. 使用 `auto_power_off_on_exit:=false` 重启，关闭窗口后确认只停止运动、退出伺服模式和 SDK 登录，不执行下使能与机器人断电。
5. 在 JAKA App 中复核默认模式的最终状态为 `enabled=0`、`powered_on=0`。

若窗口报告关机失败或超时，窗口应保持打开并允许重试；不得把软件关机当作实体急停使用。代码只调用机器人 `power_off()`，不会调用关闭整个控制柜的 `shut_down()`。

## 13. 初始位与任务预备位验收

1. 首次测试前将界面速度和加速度都设为 `3%`，确认实体急停可用并完成工作区清场。
2. 使用 `auto_move_to_initial_on_start:=false` 启动，点击“初始位”，确认后核对最终关节角度为 `[89.816, 109.950, -132.277, 201.880, 94.806, -74.132]°`。
3. 点击“预备位”，确认后核对最终关节角度为 `[-179.753, 90.057, -90.199, 90.196, 91.724, -64.680]°`。
4. 使用默认的 `auto_move_to_initial_on_start:=true` 重启，确认收到完整关节状态后只自动前往一次初始位；规划失败时不得自动重复尝试。
5. 在六关节模式输入低风险测试目标并执行，确认窗口显示“当前位置 → 任务预备位 → 用户目标”，并确认两段轨迹依次完成。
6. 在预备位阶段点击“停止运动”，确认用户目标阶段不会启动。
7. 验证单关节规划和点动不会先去预备位，完成后也不会自动回初始位。

任务结束后由操作者点击“初始位”回到停止位。关闭控制窗口时不执行回位运动，而是直接停止当前运动并进入下使能、断电流程。

## 14. `joint_move` 控制链路验收

新版控制窗口不再通过 MoveIt 下发轨迹。初始位、预备位、单关节和六关节任务均调用
`/jaka_planner/joint_move`，由 JAKA 控制器的 S 规划器完成关节空间插补。旧的
`FollowJointTrajectory`/`servo_j` 接口仅为兼容保留，控制窗口不会调用。

首次测试必须关闭自动回位，并将界面速度、加速度都设为 `3%`：

```bash
ros2 launch control_s5_controller s5_joint_controller.launch.py \
  ip:=192.168.66.200 use_rviz:=false \
  auto_move_to_initial_on_start:=false \
  joint_move_max_speed_rad_s:=1.57 \
  joint_move_max_acceleration_rad_s2:=1.57
```

1. 执行 `ros2 service list`，确认存在 `/jaka_planner/joint_move`、`/jaka_planner/stop_motion` 和 `/jaka_driver/set_rapid_rate`。
2. 分别验证单关节、初始位和预备位；每次运动完成后核对六轴实测位置，其他未选关节不得出现指令性变化。
3. 验证六关节任务仍严格按“当前位 → 任务预备位 → 用户目标”执行，第一阶段失败或停止后不得发送第二阶段。
4. 在运动过程中点击“停止运动”，确认 `motion_abort()` 生效，界面显示已停止且允许重新发起动作。
5. 运动期间尝试修改上限参数应被拒绝；空闲时执行以下命令应成功：

```bash
ros2 param set /control_s5_motion_server joint_move_max_speed_rad_s 1.0
ros2 param set /control_s5_motion_server joint_move_max_acceleration_rad_s2 1.0
```

6. 测试结束后恢复计划采用的上限值。速度和加速度参数单位分别为 `rad/s` 和 `rad/s²`。

`joint_move` 实际路径由机器人控制器生成，不等同于 MoveIt 规划轨迹，也不会使用 MoveIt
规划场景进行避障。每个目标执行前必须人工确认整段关节空间路径安全；RViz 仅用于可选观察，不能作为该控制链路的碰撞安全依据。
`use_rviz:=true` 只加载模型、TF、`move_group` 和 RViz，不启动 MoveIt 假硬件控制器。

## 15. GUI 速度反馈与动态超时验收

`control_s5_motion_server` 通过 `/jaka_driver/set_rapid_rate` 调用 SDK `set_rapidrate()`。GUI 参数区
应显示 SDK 命令速度、加速度、控制器实际倍率、估算有效速度和最大剩余误差。GUI 设置的
倍率是控制器全局状态，动作结束后保持，不会自动恢复原值。

1. 保持自动回位关闭，先将 GUI 速度和加速度设为 `3%`，选择经确认安全的小幅单关节目标。
2. 依次使用 `3%`、`10%`、`30%` 执行相同位移，确认 SDK `joint_move` 命令速度保持为配置上限，控制器 `rapidrate` 与 GUI 百分比一致，运动时间随倍率提高而缩短。
3. 分别执行单次点动、初始位、预备位、单关节和六关节两阶段任务，确认每次实际运动前均成功设置倍率，点动和 `joint_move` 均没有重复缩放速度。
4. 运动期间查看 `/jaka_planner/joint_move_status`，确认状态约每 200 ms 更新，剩余误差总体下降，正常运动不再中途误报超时。
5. 在可控、安全的条件下验证连续 10 秒无位置进展时，服务端返回 `joint_move stalled for 10 seconds` 并调用 `motion_abort()`。
6. 分别验证“停止运动”、软限位和 SDK 通信异常不会被统一显示为 timeout，而是给出对应的结束原因。

估算有效速度为 `joint_move 命令速度 × rapidrate`。加速度主要影响加减速阶段，
在长距离匀速段中视觉差异可能不明显。所有实机验收仍必须保持实体急停可用。

`/jaka_driver/set_rapid_rate` 由 `control_s5_motion_server` 提供。测试期间不得同时启动原
`jaka_driver` 节点，否则会产生服务名冲突和两个 SDK 客户端竞争同一机器人连接的问题。
