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
commit: 13c05ce60e2ea57051d5fd1be4c14850a2c68ee9
```

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
├── docs/                    # 架构、手法/视觉方案、真机检查表
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

当前程序化柔顺按压阶段的目标、实现边界和实施顺序见
[当前阶段：程序化柔顺按压与基础业务动作](docs/current_stage_programmatic_compliance.md)。

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

首次连接控制柜时，先按照
[JAKA S5 首次运动前上机验证 Runbook](docs/jaka_pre_motion_runbook.md)
完成物理连接、固定 IP-MAC 绑定核对、SDK 只读状态和 MoveIt 只规划验证。当前控制柜信息为
JKCab23、192.168.66.200、00:18:7D:ED:79:7B，推荐网络拓扑为：

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
  execute:=true joint_name:=joint_1 joint_delta:=0.02 \
  endpoint_tolerance:=0.005 \
  velocity_scale:=0.02 acceleration_scale:=0.02
```

节点会拒绝零增量、超过 `maximum_joint_delta` 的增量、无效关节状态、非有限参数以及
终点误差超限。该入口要求 `real.launch.py` 已以 `start_move_group:=true` 启动。

真机已经完成 `joint_1` 的 `+0.02/-0.02 rad` 正反向低速执行、终点误差检查、超时取消和
取消后恢复执行。更大幅度或其他关节仍须继续使用相对目标和独立参数复核。

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
  maximum_joint_travel:=3.5 endpoint_tolerance:=0.01
```

`execution_timeout_margin` 不是绝对执行时限。节点从竞争规划器选中的轨迹读取最终
`time_from_start`，实际超时为“轨迹预期时长 + 余量”，并在执行前打印这三个数值。
JAKA Action 驱动按 SDK 固定的 8 ms 控制器插补周期工作，但不会要求主机每 8 ms
完成一次同步网络调用。默认 `trajectory_servo_step_num=4`，因此主机每 32 ms 下发一个
`servo_j(..., step_num=4)` 参考，由控制柜完成本段内部插补。轨迹执行期间后台状态轮询
暂停占用 SDK 会话；连续调度超限会停止发送过期设定点并返回明确错误。每次执行将调用
时刻、调用耗时、调度延迟、期望/实际关节位置及控制状态写入
`/tmp/jaka_trajectory_<timestamp>.csv`。正常完成只退出 servo mode；只有故障或取消才调用
`motion_abort()`。

真机启动可显式覆盖调度门限：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 connect:=true start_move_group:=true \
  trajectory_servo_step_num:=4 \
  trajectory_maximum_lateness:=0.008 \
  trajectory_maximum_consecutive_overruns:=1
```

`trajectory_servo_step_num` 与主机周期不能独立配置，主机周期始终由
`step_num * 0.008 s` 推导。当前 32 ms 默认值来自首轮真机同步调用数据，仍需通过连续
小位移运行确认，不能据离线测试直接判定长轨迹已经可用。

该 launch 只启动回零 demo，不会重复启动驱动或 MoveIt。本功能当前只完成代码与离线验证，首次
仿真执行和真机执行应分别保留运行验收记录。

基础 `real.launch.py` 仍默认不运动。需要在机器人进入上电、使能且静止状态后自动执行回待机任务时，
使用：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 connect:=true start_move_group:=true \
  auto_home:=true auto_home_confirmed:=true
```

回待机节点会等待机器人进入可执行状态，再运行竞争规划和既有任务状态机；它不会代替登录、上电或使能。

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
git checkout 13c05ce60e2ea57051d5fd1be4c14850a2c68ee9

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

## 文档

- [项目总体架构与进度](docs/project_outline.md)
- [当前阶段：程序化柔顺按压与基础业务动作](docs/current_stage_programmatic_compliance.md)
- [推拿手法与视觉反馈方案](docs/massage_technique_vision_plan.md)
- [JAKA S5 首次运动前上机验证 Runbook](docs/jaka_pre_motion_runbook.md)
- [JAKA 真机联调检查表](docs/jaka_real_hardware_integration.md)
- [规划执行实验记录](docs/planning_execution_demo_report.md)

## License

本项目自有代码采用 BSD-3-Clause。JAKA 官方源码、SDK 动态库、机器人模型和其他第三方
组件继续遵循各自许可证；本项目许可证不会覆盖供应商组件。
