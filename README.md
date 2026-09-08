# JAKA S5 推拿机器人控制工程

基于 ROS 2 Humble、MoveIt 2、Pilz、ros2_control 和 Gazebo Sim 的 JAKA S5
工程

## 仓库与依赖策略

工程采用两个并列仓库，不复制或裁剪供应商源码：

```text
robot_project/
├── jaka_ros2/              # JAKA 官方仓库的个人 Fork
└── massage_robot_ws/       # 本仓库，项目业务代码
```

本项目在 GitHub 上的仓库名是 `XieFField/JAKA_S5`，克隆到本地时使用
`massage_robot_ws` 作为 ROS 2 工作区目录名。Git 仓库名与本地目录名无需一致；后续文档中的
`massage_robot_ws` 均表示本地工作区目录，不要求修改 GitHub 仓库名。

`jaka_ros2` 保留官方所有型号，项目修改放在 `feature/massage-s5-integration` 分支。
这样能够继续合并官方修复，也能清楚审计我们对驱动和 S5 MoveIt 配置的修改。删除其他
型号只会制造难维护的私有副本，并不会明显降低部署体积。

不要把两个仓库合并成一个仓库，也不要把 `jaka_ros2` 复制进本仓库的 `src/`。
项目仓库只保存业务层和适配层；供应商仓库只保存供应商代码及必要的通用驱动修改。
部署时用两个固定提交组合出一个可复现版本。

本项目当前依赖的 JAKA 集成版本：

```text
branch: feature/massage-s5-integration
commit: 504362f
```

## 当前进度

- Gazebo 已完成推、按、揉三种手法、工具轴对齐、三次循环、安全回程、世界系六维力变换和初步
  模拟接触/导纳状态机验证。
- 真机自由空间执行已经切换为混合原生后端：PTP 使用 `joint_move`，LIN/CIRC 使用对应 JAKA SDK
  接口；`servo_j` 只保留作诊断和以后确有需要的在线控制。
- 真机自由空间按法和揉法已完成端到端 PASS，推法还需用当前版本完成一次完整回待机验收。
- 真实接触搜索、恒力和切向轨迹的复合控制仍未完成真机验证；当前自由空间 PASS 不能作为真实
  推拿闭环通过依据。
- 当前优先问题是 native PTP 实际速度约 `0.032 rad/s`，明显低于名义速度；LIN 速度正常，问题已
  限定在 `joint_move`/关节 PTP 链路。

新成员应先阅读 `doc/项目架构与阶段交接说明.md`，其中包含完整架构、验证边界、已知问题和后续
开发顺序。

## 软件环境

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 / Pilz Industrial Motion Planner
- ros2_control、`admittance_controller`
- Gazebo Sim Fortress、`ros_gz_sim`、`ros_gz_bridge`
- JAKA 官方 SDK 动态库（随 `jaka_driver` 目录提供，许可遵循供应商条款）

系统依赖应使用 rosdep 安装；不要提交 `build/`、`install/` 或 `log/`：

```bash
sudo rosdep init  # 仅首次安装 rosdep 时执行
rosdep update
rosdep install --from-paths \
  ~/robot_project/jaka_ros2/src \
  ~/robot_project/massage_robot_ws/src \
  --ignore-src -r -y --rosdistro humble
```

## 构建

始终从两个工作区根目录构建，并按 ROS -> JAKA -> 项目的顺序 source：

```bash
source /opt/ros/humble/setup.bash

cd ~/robot_project/jaka_ros2
colcon build --symlink-install \
  --packages-up-to jaka_driver jaka_s5_moveit_config
source install/setup.bash

cd ~/robot_project/massage_robot_ws
colcon build --symlink-install
source install/setup.bash
```

运行测试：

```bash
cd ~/robot_project/jaka_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select jaka_driver
colcon test-result --verbose

cd ~/robot_project/massage_robot_ws
source /opt/ros/humble/setup.bash
source ~/robot_project/jaka_ros2/install/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

## 代码结构

```text
massage_robot_ws/
└── src/
    ├── massage_description/ # 项目 Xacro、按摩头、TCP、FT 和世界
    ├── massage_motion/      # 规划、执行、柔顺公共接口和后端
    │   ├── include/
    │   ├── src/             # 仅库实现
    │   ├── demo/            # 可执行示例
    │   └── test/
    ├── massage_task/        # 任务状态机与故障恢复
    │   ├── src/             # 仅库实现
    │   ├── demo/
    │   └── test/
    ├── massage_jaka/        # JAKA 生命周期、柔顺和错误适配
    │   ├── include/         # 真机适配公共接口
    │   ├── src/             # 适配层实现与只读诊断
    │   ├── demo/            # 真机运动/柔顺冒烟入口
    │   └── test/            # 错误映射与保护逻辑测试
    └── massage_bringup/     # 仿真/真机启动与控制器配置
```

运行时边界：

```text
massage_task
├── IMotionPlanner -> MoveIt/Pilz
├── ITrajectoryExecutor -> Gazebo 或 JAKA FollowJointTrajectory
└── IComplianceController
    ├── Ros2ControlComplianceController -> Gazebo
    └── JakaComplianceController -> jaka_driver -> JAKA SDK
```

Gazebo、Robot State Publisher、MoveIt 和 RViz 均使用
`massage_description/urdf/jaka_s5_massage.urdf.xacro`。规划和工艺目标统一使用
`massage_tool_tip`，不能再把法兰 `Link_06` 当作接触点。

推法轨迹的安全边界目前停留在自由空间：位置轨迹与人体接触后的力位复合控制是两个独立阶段，
真机入口在后一阶段完成并验证前不会发送推法运动命令。

## 仿真启动

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_project/jaka_ros2/install/setup.bash
source ~/robot_project/massage_robot_ws/install/setup.bash
```

启动通用仿真（默认不自动运动）：

```bash
ros2 launch massage_bringup sim.launch.py use_rviz:=true
```

启动柔顺按压验收：

```bash
ros2 launch massage_bringup compliant_press_task_demo.launch.py
```

业务回待机目标保存在 `massage_bringup/default_targets.hpp` 的
`kMassageHomeJointPositions` 常量中。通用仿真与 MoveIt 已启动后，默认先生成 3 个候选并择优，
但不执行：

```bash
ros2 launch massage_bringup return_home_sim.launch.py \
  execute:=false planning_attempts:=3
```

确认日志中的六轴计划行程后才单独设置 `execute:=true`。该 launch 不会自行启动 Gazebo 或 MoveIt。

demo 完成后仿真基础设施会继续运行，便于检查 TF、Planning Scene 和控制器。检查完成后
在 launch 终端按 `Ctrl-C` 统一关闭。保护路径示例仅用于仿真：

```bash
ros2 launch massage_bringup compliant_press_task_demo.launch.py \
  force_limit:=0.2 expect_limit_exceeded:=true
```

## 真机启动

首次连接控制柜时，必须依次完成物理连接、现场网络配置、SDK 只读状态和 MoveIt 只规划验证。
推荐网络拓扑为：

~~~text
电脑/工作服务器 -> 现场路由器（固定地址绑定） -> JAKA 控制柜 -> JAKA S5
~~~

该固定地址由现场路由器按照控制柜 MAC 地址分配，不等同于已经确认控制柜网卡内部保存了静态
地址。因此当前保留路由器，不改成电脑与控制柜单线直连；launch 仍要求显式传入控制柜 IP。

### 1. 只读启动

真机入口要求显式传入控制柜 IP，但不会自动登录、上电、使能或运动：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 \
  connect:=true \
  start_move_group:=false \
  use_rviz:=false
```

先核对接口和状态：

```bash
ros2 action list | rg follow_joint_trajectory
ros2 topic echo /joint_states --once
ros2 topic echo /jaka_driver/robot_states --once
ros2 topic echo /jaka_driver/wrench --once
ros2 run tf2_ros tf2_echo world massage_tool_tip
```

没有登录时 Action Goal 必须被驱动拒绝。`connect:=false` 可用于只检查项目节点和
launch 参数，但仍要求提供一个显式 `robot_ip`，避免把现场地址写入仓库。

### 2. 显式生命周期

按顺序调用：

```bash
ros2 service call /jaka_driver/login std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/power_on std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/enable_robot std_srvs/srv/Trigger "{}"
```

确认 `/joint_states` 与真机一致后，再重启或另行启动 MoveIt：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 connect:=true start_move_group:=true use_rviz:=true
```

### 3. 真机运动冒烟

`real_motion_smoke_demo` 不使用硬编码绝对点位，而是从当前 `/joint_states` 构造一个单关节
相对目标。默认 `execute=false`，只验证当前状态、真实关节限位和 Pilz PTP 规划：

```bash
ros2 launch massage_bringup real_motion_smoke_demo.launch.py
```

显式执行时仍使用同一条 MoveIt `ExecuteTrajectory` 到 JAKA
`FollowJointTrajectory` 链路，并在结束后检查轨迹终点误差：

```bash
ros2 launch massage_bringup real_motion_smoke_demo.launch.py \
  execute:=true joint_name:=joint_1 joint_delta:=0.01 \
  endpoint_tolerance:=0.002 \
  velocity_scale:=0.02 acceleration_scale:=0.02
```

节点会拒绝零增量、超过 `maximum_joint_delta` 的增量、无效关节状态、非有限参数以及
终点误差超限；`endpoint_tolerance` 还必须严格小于 `abs(joint_delta)`，避免机械臂未实际
到达目标时误判成功。驱动和独立 demo 的默认终点容差均为 `0.002 rad`。该入口要求
`real.launch.py` 已以 `start_move_group:=true` 启动。

收紧驱动终点容差为 `0.002 rad` 后，真机已完成一次 `joint_1 +0.01 rad` 小位移闭环：
15 个 Servo 分段在 `0.141695 s` 内连续入队，队列饥饿为零；最终实际位移
`0.008140218 rad`、终点误差 `0.001859782 rad`，Action、MoveIt 和 demo 均返回成功。
该结果接近容差边界，因此不能单独证明重复精度、反向运动和长轨迹；这些项目已由后续重复性、
中距离往返和完整回待机测试分别完成验证。

正反向重复性批次使用固定基准和固定正向目标，默认只规划首个目标。显式执行后按
`positive -> negative` 完成三轮，任一失败立即停止，并将每程结果写入
`/tmp/massage_motion_repeatability_<timestamp>.csv`：

```bash
ros2 launch massage_bringup real_motion_repeatability_demo.launch.py \
  execute:=true parameters_confirmed:=true \
  joint_name:=joint_1 cycles:=3 joint_delta:=0.01 \
  maximum_joint_delta:=0.02 endpoint_tolerance:=0.002 \
  minimum_completion_ratio:=0.90 maximum_position_range:=0.001 \
  velocity_scale:=0.02 acceleration_scale:=0.02
```

重复性批测要求运行中的驱动终点容差与完成度门槛一致。对于
`joint_delta:=0.01`、`minimum_completion_ratio:=0.90`，启动唯一一套真机栈时使用：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 connect:=true start_move_group:=true \
  use_rviz:=false auto_home:=false \
  trajectory_goal_tolerance:=0.0008 trajectory_goal_timeout:=15.0
```

批测节点会读取 `/jaka_driver` 的实际参数；容差过宽时会在发送第一个
Action Goal 前拒绝执行，不依赖人工核对。

批次要求六程全部通过、每程完成率至少 90%，且同方向三次最终位置极差不超过
`0.001 rad`。这组门禁比驱动的单程 Action 成功条件更严格。
2026-08-18 真机六程已通过。

中距离门禁首先只生成当前位置到业务待机位姿 `25%` 处的固定绝对目标，并进行三次竞争规划。
该入口没有 `execute` 参数，也不创建轨迹执行器：

```bash
ros2 launch massage_bringup home_segment_planning_real.launch.py \
  segment_ratio:=0.25 planning_attempts:=3 \
  maximum_joint_travel:=0.15 \
  velocity_scale:=0.02 acceleration_scale:=0.02
```

它要求唯一 MoveIt 栈和 `/joint_states` 已就绪。通过日志必须包含六轴起点/待机/分段目标、
三个候选评分以及 `HOME SEGMENT PLAN-ONLY: PASS`，不会发送运动命令。

分段只规划通过后，使用独立入口执行一次固定目标往返。节点启动时保存六轴基准，去程目标为
`q_start + ratio * (q_home - q_start)`，返程目标始终是保存的 `q_start`，不会累计相对位移：

```bash
ros2 launch massage_bringup home_segment_round_trip_real.launch.py \
  execute:=true parameters_confirmed:=true \
  segment_ratio:=0.25 planning_attempts:=3 \
  maximum_joint_travel:=0.15 \
  endpoint_tolerance:=0.002 \
  velocity_scale:=0.02 acceleration_scale:=0.02 \
  execution_timeout_margin:=10.0
```

该入口必须在唯一真机栈以 `allow_trajectory_execution:=true` 启动后使用。执行前会读取驱动
`trajectory_goal_tolerance` 和 `trajectory_goal_timeout`，并检查新鲜关节反馈、机器人状态、规划
起点以及每段最大关节行程。每程分别进行三次竞争规划并使用轨迹预期时长加余量计算超时；去程
失败会禁止返程。两程逐轴终点数据和候选指标写入
`/tmp/massage_home_segment_round_trip_<timestamp>.csv`，只有最后出现
`HOME SEGMENT ROUND-TRIP: PASS` 才表示通过。

2026-08-19 已依次完成 `ratio=0.25`、`ratio=0.50` 往返和完整回待机。五段轨迹均无队列饥饿，
完整回待机最大单关节行程为 `0.484582 rad`、最大终点误差为 `0.001832293 rad`。

不改变目标关节位置的 Action 协议验证用于检查 feedback、并发 Goal 拒绝、取消和取消后恢复。
默认只检查 `/joint_states` 与 Action 是否存在；显式启用后，节点将当前六轴位置作为目标，
不会生成计划位移：

```bash
ros2 launch massage_bringup real_action_protocol_demo.launch.py \
  run_protocol_test:=true \
  test_external_stop:=true
```

通过日志应同时包含 `feedback` 计数、`并发 Goal 已拒绝`、`取消与恢复成功`，并且最大实际
关节位移不超过 `maximum_stationary_delta`。该入口直接连接驱动 Action，不要求启动 MoveIt，
但驱动必须已经登录、上电并使能。真机已经验证完整 feedback、并发 Goal 拒绝、标准 Action
取消、legacy motion 互斥、`stop_move` 抢占以及两种停止路径后的新 Goal 恢复；零位移测试测得
最大实际关节变化和最大反馈误差均为 `0 rad`。

### 4. 真机 FT 被动观测

阶段二先把 FT 数据验证与柔顺启用分开。该入口只订阅
`/jaka_driver/wrench`，不会创建柔顺控制器，也不会调用运动、上电、使能或导纳服务。驱动连接并
登录后，保持机械臂静止，按日志依次完成无外载基线、人工加载和卸载恢复三个时间窗：

```bash
ros2 launch massage_bringup real_ft_observation_demo.launch.py
```

默认采集 `5 s + 10 s + 5 s`，检查 `Link_06` 坐标系、有限六维数值、阶段内采样率和最大采样
间隔，并将原始样本写入 `/tmp/massage_ft_observation_<timestamp>.csv`。首次运行默认
`required_response:=none`，只采集真实数据，不用未经验证的阈值判定传感器是否合格。CSV 确认
符号、噪声和加载幅值后，可以显式启用响应门禁，例如：

```bash
ros2 launch massage_bringup real_ft_observation_demo.launch.py \
  required_response:=either \
  minimum_force_delta:=0.5 \
  minimum_torque_delta:=0.05
```

`required_response` 支持 `none/force/torque/either/both`。力单位按消息定义为 N，力矩为 N*m；
门限是否适合当前 FT 硬件必须由第一份 CSV 决定。即使采集中断或验收失败，节点也会尝试保存
失败前已收到的样本。

### 5. 真机柔顺冒烟

默认入口只检查 `/joint_states`、`/jaka_driver/wrench` 和
`world -> massage_tool_tip`，不会启用导纳：

```bash
ros2 launch massage_bringup real_compliance_smoke_demo.launch.py
```

`axis` 的 `0..5` 固定表示 FT 坐标系中的 `X/Y/Z/RX/RY/RZ`。只有显式设置
`activate:=true parameters_confirmed:=true` 才会配置软限幅、导纳参数并进行定时启停；
`target_wrench` 表示 SDK 的恒力目标 `ftConstant`；`maximum_speed_wrench` 表示达到最大柔顺
速度所需的外力/力矩 `ftUser`；`rebound_wrench` 表示回到初始位置的恢复能力
`ftReboundFK`。`maximum_speed_wrench` 默认是 `0`，启用轴未显式设置正值时程序会拒绝启动。
这些参数、六维上限和位移上限必须先按实际 FT/SDK 定义确定，仓库不提供可直接用于真机的
默认工艺参数。运行期间适配器持续检查：

- FT 与关节反馈是否过期；
- 任一已配置的六维力/力矩上限；
- 最大关节位移与 TCP 直线位移；
- 总运行时间；
- 导纳关闭服务是否得到确认。

当前真机柔顺入口是独立导纳冒烟，不发送名义轨迹。完整按压状态机仍需确认 JAKA SDK
对“目标力、内部参考和外部名义轨迹”的组合语义；在该语义确认前，项目不会同时开启
轨迹命令通道和导纳命令通道。

### 6. 业务回零

项目回零位是业务待机位姿常量，不是编码器标定或六轴全零。真机驱动与唯一 MoveIt
实例启动后，默认只规划多个候选，按关节路径长度和轨迹时长评分并输出选择结果：

```bash
ros2 launch massage_bringup return_home_real.launch.py \
  execute:=false planning_attempts:=3
```

真机执行还需要显式双重确认，并受 `maximum_joint_travel`、机器人状态和终点误差门控：

```bash
ros2 launch massage_bringup return_home_real.launch.py \
  execute:=true parameters_confirmed:=true \
  velocity_scale:=0.02 acceleration_scale:=0.02 \
  execution_timeout_margin:=15.0 \
  maximum_joint_travel:=0.55 endpoint_tolerance:=0.002
```

`execution_timeout_margin` 不是绝对执行时限。节点从竞争规划器选中的轨迹读取最终
`time_from_start`，实际超时为“轨迹预期时长 + 余量”，并在执行前打印这三个数值。
JAKA Action 驱动按 SDK 固定的 8 ms 控制器插补周期工作。每个 MoveIt 轨迹段根据
`dt / 0.008` 计算 `step_num`，超过 `trajectory_maximum_servo_step_num` 的长段才线性拆分；
随后按 SDK 约定连续提交全部分段，由控制柜按插补时间轴执行。轨迹执行期间后台状态轮询
暂停占用 SDK 会话；连续队列饥饿会停止提交并返回明确错误。每次执行将调用
时刻、调用耗时、队列饥饿量、期望/实际关节位置及控制状态写入
`/tmp/jaka_trajectory_<timestamp>.csv`。正常完成只退出 servo mode；只有故障或取消才调用
`motion_abort()`。

真机启动可显式覆盖调度门限：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 connect:=true start_move_group:=true \
  trajectory_maximum_servo_step_num:=50 \
  trajectory_servo_filter_cutoff_hz:=0.5 \
  trajectory_maximum_queue_starvation:=0.008 \
  trajectory_maximum_consecutive_starvations:=1
```

`trajectory_maximum_servo_step_num` 是单次控制柜插补步数上限，不是主机发送周期。
`trajectory_servo_filter_cutoff_hz=0` 表示显式关闭 Servo 滤波；默认 `0.5` 与官方
MoveIt 示例一致。队列实现已经通过小位移、正反向重复性、中距离往返和完整回待机真机验证；
这些结果只覆盖自由空间轨迹执行，不覆盖接触状态下的力位复合控制。

该 launch 只启动回零 demo，不会重复启动驱动或 MoveIt。2026-08-19 已依次通过 25%、50%
分段往返和完整回待机；完整轨迹最大单关节行程 `0.484582 rad`、最大终点误差
`0.001832293 rad`，172 个轨迹点对应的 171 个 Servo 队列段无饥饿。

执行模式会在创建规划器之前按关节名称比较当前反馈与业务待机常量。最大误差不超过
`endpoint_tolerance` 时输出 `AUTO HOME ALREADY-AT-TARGET: PASS`，不创建 MoveIt 规划器、
不发送轨迹 Goal；否则进入竞争规划和受保护执行。两条成功路径均输出逐轴误差、机器人状态和
`/tmp/massage_auto_home_<timestamp>.csv`。

基础 `real.launch.py` 仍默认不运动。需要在机器人进入上电、使能且静止状态后自动执行回待机任务时，
使用：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 connect:=true start_move_group:=true \
  auto_home:=true auto_home_confirmed:=true \
  trajectory_goal_timeout:=15.0
```

主 launch 的自动回待机默认最大单关节行程为已经验证的 `0.55 rad`，并在启动任何节点前校验
速度、加速度、规划时限、动态超时余量、反馈时限、最大行程和终点容差。驱动
`trajectory_goal_timeout` 表示控制柜计划结束后的终点收敛余量，不是固定的总执行时长；自动回待机
要求该余量不小于 `home_execution_timeout_margin`，独立回待机节点也会在规划前读回并校验。回待机节点会等待机器人
进入可执行状态；它不会代替登录、上电或使能。`auto_home` 仍默认关闭。

结束时按以下顺序关闭：

```bash
ros2 service call /jaka_driver/stop_move std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/enable_admittance std_srvs/srv/SetBool "{data: false}"
ros2 service call /jaka_driver/disable_robot std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/power_off std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/logout std_srvs/srv/Trigger "{}"
```

## 部署到工作服务器

在服务器创建同样的并列目录，并使用固定提交而不是复制开发机的 `install/`：

```bash
mkdir -p ~/robot_project
cd ~/robot_project

git clone <YOUR_JAKA_FORK_URL> jaka_ros2
cd jaka_ros2
git checkout feature/massage-s5-integration
git checkout b3a315dbf19508f6b881cb3a986d26b555dbbeb8

cd ~/robot_project
git clone <YOUR_MASSAGE_REPOSITORY_URL> massage_robot_ws
cd massage_robot_ws
git checkout <MASSAGE_PROJECT_COMMIT_OR_TAG>
```

然后在服务器重新执行 rosdep、构建和测试。设备 IP、FT 参数及控制参数放在服务器私有
配置或启动参数中，不提交到 Git。部署前记录：两个仓库提交号、ROS 版本、SDK 库校验
值、控制柜/机器人固件版本和验收结果。

## 常见问题

- `package not found`：检查 source 顺序，并确认从工作区根目录构建。
- 新 launch 找不到：重新构建对应包并重新 source 项目 overlay。
- 控制器重复加载或 Gazebo 端口/实体冲突：先确认上一次 Ignition/Gazebo、
  `controller_manager` 和 `move_group` 进程已经退出，再重新启动；不要在残留仿真上叠加第二套 bringup。
- MoveIt 看不到按摩头：必须启动 `massage_bringup` 中的 MoveIt 入口，不能直接使用官方
  裸 S5 `move_group`。
- Action 被拒绝：依次检查 SDK 登录、上电、使能、控制权占用和机器人故障状态。
- 柔顺控制器无法激活：轨迹控制器与导纳控制器占用同一组命令接口，切换时必须先停用
  当前控制器。
- FT 有静态偏置：先确认坐标系、重力/偏置补偿数据类型和零点流程，禁止仅靠软件减常数
  作为真机标定。
- ROS 2 Humble 的 MoveIt/Gazebo 组合在 `Ctrl-C` 关闭时，个别上游进程可能以 `-11` 或
  `-2` 退出。先区分“正常运行阶段故障”和“仅关闭阶段退出码”；重新启动前仍要确认所有
  相关进程已经结束。

## License

本项目自有代码采用 BSD-3-Clause。JAKA 官方源码、SDK 动态库、机器人模型和其他第三方
组件继续遵循各自许可证；本项目许可证不会覆盖供应商组件。
