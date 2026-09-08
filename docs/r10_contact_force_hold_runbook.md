# R10 真机世界 Z 接触搜索与低力保持

## 1. 测试边界

R10 只验证 JAKA 控制柜内部恒力柔顺能力：按摩头从已经到达的预接触位沿世界 `-Z` 搜索固定软块，达到 `1 N` 后保持 `3 s`，随后关闭导纳并恢复测试前配置。

R10 不调用 MoveIt、`servo_p`、`servo_j` 或完整推拿状态机，不执行推、按、揉切向轨迹，不接触人体，也不会自动退出或回待机。

测试入口分为三个门：

1. `activate=false`：安全空闲，不创建柔顺控制器，不清零、不配置、不运动。
2. `activate=true, execute_contact_search=false`：清零、配置与读回预检，导纳保持关闭，不运动，结束时恢复原配置。
3. `activate=true, execute_contact_search=true`：实际接触搜索与恒力保持，仅由现场人员执行。

## 2. 软测试块布置

- 使用硅胶或泡棉软块，厚度建议 `30-50 mm`，顶面至少 `100 x 100 mm`。
- 在 `5 N` 内不能压到底，底面使用防滑垫或夹具固定，禁止手持。
- 顶面水平，中心位于按摩头正下方；按摩头局部 `+Z` 应与世界 `-Z` 对齐，允许误差不超过 `3 deg`。
- 首轮实际搜索时，按摩头与软块顶面的无接触间隙设为 `1-2 mm`。确认 `2 mm/s` 速度限制真机读回有效前，不扩大间隙。
- 保留向上退出空间。测试结束后必须先确认导纳关闭且 `control_owner=idle`，再降低或移走测试块，最后使用已经验证的自由空间程序退出。
- 人员不得把手或身体放在按摩头与测试块之间；急停必须随时可用。

## 3. 构建

```bash
cd ~/jaka_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select jaka_msgs jaka_driver

cd ~/xieffield/pratice/massage_robot_ws
source /opt/ros/humble/setup.bash
source ~/jaka_ros2/install/setup.bash
colcon build --symlink-install --packages-up-to massage_bringup
```

## 4. 启动与只读门禁

终端 A 启动真机基础节点；R10 不需要 MoveGroup：

```bash
source /opt/ros/humble/setup.bash
source ~/jaka_ros2/install/setup.bash
source ~/xieffield/pratice/massage_robot_ws/install/setup.bash

ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 \
  connect:=true \
  start_move_group:=false \
  use_rviz:=false \
  auto_home:=false
```

终端 B 登录、上电、使能并检查状态：

```bash
source /opt/ros/humble/setup.bash
source ~/jaka_ros2/install/setup.bash
source ~/xieffield/pratice/massage_robot_ws/install/setup.bash

ros2 service call /jaka_driver/login std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/power_on std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/enable_robot std_srvs/srv/Trigger "{}"
ros2 topic echo /jaka_driver/robot_states --once
ros2 service call /jaka_driver/get_admittance_state \
  jaka_msgs/srv/GetAdmittanceState "{}"
```

必须满足：`motion_state=0, power_state=1, servo_state=1, collision_state=0`，并且 `force_control_enabled=false, control_owner=idle`。
还必须从已经完成的 R2 世界系六维力记录确认：软块对向下运动按摩头产生的向上反力表现为正的 world `Fz`。若符号相反，应先修复传感器/TF 约定，不得通过取绝对值绕过。

## 5. 分级执行

### R10-A：安全入口

```bash
ros2 launch massage_bringup real_contact_force_hold.launch.py
```

通过标志：`R10 CONTACT FORCE HOLD: SAFE IDLE`。这一阶段不要求机器人连接。

### R10-B：配置与恢复预检，不运动

按摩头悬空、无接触。R10-B 不运动，因此只记录当前工具姿态，不要求已经
对准世界 `-Z`：

```bash
ros2 launch massage_bringup real_contact_force_hold.launch.py \
  activate:=true \
  parameters_confirmed:=true \
  execute_contact_search:=false
```

通过标志：`R10 CONFIGURE-ONLY: PASS`。同时必须看到恒力 `type=1`、传感器补偿 `1`、线速度上限 `2.0 mm/s` 的读回，以及原始配置恢复成功。没有 `R10 ACTIVE`，机械臂不得移动。

### R10-C0：自由空间到达预接触位

这一步只执行 MoveIt 自由空间轨迹，不启用导纳。流程固定为：

1. 当前位姿到接触点上方工作准备位：生成多个 IK 分支；每个分支必须同时能够完成后续垂直 LIN，才进入整条 `PTP + LIN` 方案评分，避免只选中 PTP 最短但无法继续接近的分支。
2. 工作准备位到软块上方预接触位：保持工具姿态的世界 `-Z` 方向 LIN。
3. PTP 终点和整段 LIN 均检查按摩头局部 `+Z` 对准世界 `-Z`，误差不超过 `3 deg`。
4. 真机执行前检查静止、上电、使能、无碰撞；每段执行超时由轨迹预期时长加余量计算，并在每段后检查关节终点误差。
5. PTP 全轨迹不得低于“当前 TCP 高度与预接触平面高度中的较低值”；允许从较低初始位只向上离开，但禁止路线进一步下探患者/软块区域。

R10-C0 需要 MoveGroup，因此基础节点必须用 `start_move_group:=true` 启动。
先只规划，其中 `contact_z` 是固定软块顶面的世界系高度：

```bash
ros2 launch massage_bringup r10_precontact_real.launch.py \
  execute:=false \
  contact_x:=-0.471238630147741 \
  contact_y:=0.156275068796867 \
  contact_z:=0.281381721182422 \
  precontact_clearance:=0.002
```

通过标志：`R10 PRECONTACT PLAN-ONLY: PASS`。确认 PTP/LIN 目标、IK 分支、
姿态门禁和 RViz 轨迹合理后，才能执行同一组参数：

```bash
ros2 launch massage_bringup r10_precontact_real.launch.py \
  execute:=true \
  parameters_confirmed:=true \
  contact_x:=-0.471238630147741 \
  contact_y:=0.156275068796867 \
  contact_z:=0.281381721182422 \
  precontact_clearance:=0.002 \
  velocity_scale:=0.05 \
  acceleration_scale:=0.05
```

通过标志：`R10 PRECONTACT EXECUTION: PASS`。这一阶段不能出现 `R10 ACTIVE`，
也不应改变 JAKA 导纳配置。执行结束后现场复核按摩头与软块顶面间隙确实为
`1-2 mm`；若实际间隙不符，应修正软块顶面的 `contact_z` 后重新从只规划开始，
不能用增大 R10-C 搜索深度代替测量。

### R10-C：固定软块实际接触

完成 R10-B 和 R10-C0 后摆放并固定软块，确认无接触间隙为 `1-2 mm`：

```bash
ros2 launch massage_bringup real_contact_force_hold.launch.py \
  activate:=true \
  parameters_confirmed:=true \
  execute_contact_search:=true \
  contact_x:=-0.471238630147741 \
  contact_y:=0.156275068796867 \
  contact_z:=0.281381721182422 \
  precontact_clearance:=0.002 \
  maximum_precontact_position_error:=0.003 \
  target_wrench_z:=-1.0 \
  expected_motion_direction_z:=-1.0 \
  expected_measured_force_sign_z:=1.0 \
  compliance_linear_speed_limit_mm_s:=2.0 \
  approach_linear_speed_limit_mm_s:=2.0 \
  contact_timeout:=5.0 \
  hold_duration:=3.0 \
  maximum_force:=5.0 \
  maximum_torque:=1.0 \
  maximum_linear_displacement:=0.012 \
  maximum_transverse_displacement:=0.002 \
  maximum_joint_displacement:=0.05
```

正常日志顺序：

```text
R10 就绪通过
R10 恒力 profile 已读回
R10 ACTIVE
R10 CONTACT CONFIRMED
R10 TARGET FORCE REACHED
R10 CLEANUP VERIFIED
R10 CONTACT FORCE HOLD: PASS
```

R10-C 使用的 `contact_x/y/z` 与 `precontact_clearance` 必须和已经执行通过的
R10-C0 完全一致。启动导纳前会同时检查工具局部 `+Z` 对齐世界 `-Z`，以及
当前 `massage_tool_tip` 到预接触目标的三维位置误差不超过 `3 mm`；任一不满足
都会在启用导纳之前拒绝执行。

## 6. 通过指标

- 目标力 `1.0 N`，稳态容差 `+-0.3 N`，容差内样本比例至少 `80%`。
- 保持有效时长至少 `2.7 s`，反馈最大样本间隔不超过 `0.2 s`。
- 稳态标准差不超过 `0.25 N`，平均绝对误差不超过 `0.25 N`，末样本误差不超过 `0.3 N`。
- 法向峰值严格小于 `5 N`，任一力矩轴严格小于 `1 N*m`。
- TCP 总位移不超过 `12 mm`，横向漂移不超过 `2 mm`，反方向位移不超过 `1 mm`。
- 任一关节相对启动位置不超过 `0.05 rad`。
- 全程 `collision_state=0, power_state=1, servo_state=1`。
- 退出后 `force_control_enabled=false, control_owner=idle`，原始六轴导纳配置、力控坐标系、软限幅和柔顺 profile 读回一致。
- CSV 位于日志中的 `/tmp/massage_r10_contact_hold_*.csv`。

控制柜碰撞阈值即使设置为 `125 N`，也不改变 R10 的独立 `5 N` 软件硬限力。

## 7. 失败处理

出现碰撞、异常声音、绿灯变蓝、非预期下使能或错误方向运动时立即急停。软件失败后不得直接发送 MoveIt/Servo 退出命令；先执行：

```bash
ros2 service call /jaka_driver/get_admittance_state \
  jaka_msgs/srv/GetAdmittanceState "{}"
ros2 topic echo /jaka_driver/robot_states --once
```

只有确认 `force_control_enabled=false, control_owner=idle` 后，才可降低或移走测试块并决定后续自由空间退出。若日志出现“未确认导纳关闭”或“恢复被拒绝”，停止本轮，不自动重试。
