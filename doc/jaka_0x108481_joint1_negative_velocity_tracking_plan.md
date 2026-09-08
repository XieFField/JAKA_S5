# JAKA 0x108481：ROS 轨迹到 `servo_j` 转接参数专项测试方案

## 1. 目标

本轮只验证以下链路：

```text
MoveIt RobotTrajectory
  -> FollowJointTrajectory Goal
  -> 轨迹重采样与分段
  -> time_from_start 到 step_num 的转换
  -> servo_j(关节绝对位置, step_num)
  -> JAKA 控制柜
```

目标是确认 ROS 轨迹经过适配层后，下发给 JAKA 的关节位置、段时长、隐含速度、滤波参数和队列行为是否正确，并判断该转换是否可能触发 joint_1 报警 `0x108481`。

报警含义：joint_1 实际转速与控制柜设定转速反向，并且速度误差超过报警阈值。

## 2. 测试边界

### 2.1 本轮包含

- `joint_names` 到 JAKA 六个轴的顺序映射；
- 弧度单位、正负方向和 `MoveMode::ABS` 语义；
- 实时起始关节位置与第一条 `servo_j` 命令的连续性；
- MoveIt `time_from_start` 到 `step_num * 8 ms` 的量化；
- 原始轨迹点到 `servo_j` 目标点的重采样和插值；
- MoveIt `velocities[]`、`accelerations[]` 与适配层隐含速度、加速度的差异；
- `maximum_step_num` 的拆段行为；
- `servo_j` 队列连续填充、SDK 调用耗时和队列饥饿；
- joint LPF、joint NLF、none filter 和 foresight 等 SDK 配置的调用语义；
- 故障前 joint_1 命令位置、隐含速度、实际位置和估算实际速度的同步遥测；
- 正常完成、取消、故障退出时的 servo mode 清理行为。

### 2.2 本轮明确不包含

- IK 解选择、周期角等价分支和关节绕行；
- MoveIt 规划器质量、路径几何和肘上约束；
- 碰撞检测和环境障碍物；
- 控制柜电机方向、编码器、减速比、位置环或速度环标定；
- 修改 `0x108481` 的控制柜报警阈值；
- 力控、导纳控制和推拿业务状态机。

上述项目可能影响最终真机行为，但不是本轮变量。专项测试必须使用固定输入轨迹，避免规划结果变化干扰转接参数对比。

## 3. 当前实现与风险点

当前适配层主要把相邻关节位置和段时长转换成：

```text
servo_j(target_joint_position, ABS, step_num)
```

其中：

```text
scheduled_dt[i] = step_num[i] * 0.008
implicit_velocity[i] = (q[i] - q[i-1]) / scheduled_dt[i]
```

MoveIt 轨迹中的 `velocities[]` 和 `accelerations[]` 没有直接传给 `servo_j`。因此控制柜看到的设定速度取决于位置差、`step_num`、控制柜插补和 servo filter，而不一定等于 MoveIt 原始速度。

当前实现还使用 `0.5 Hz` joint LPF。它是否导致 `0x108481` 尚未得到证明，只能作为专项对照变量，不能直接判定为根因。

已有失败日志显示：

- `servo_j` 队列完成填充；
- `starved_segments=0`；
- SDK 调用耗时没有明显失控；
- joint_1 命令位置在故障前保持单调负向。

因此本轮重点不是再次确认“有没有发送命令”，而是确认“发送给控制柜的每段位置和时间组合实际表达了什么速度”。

## 4. 转接层根因假设

### H1：`time_from_start` 到 `step_num` 的量化存在累计偏差

如果每段独立取整，单段误差可能沿整条轨迹累计，使实际调度总时长偏离 MoveIt 轨迹总时长，并改变隐含速度。

需要检查：

- `step_num >= 1`；
- `step_num` 是否被 `maximum_step_num` 正确拆分；
- 每段和整条轨迹的量化时间误差；
- 是否采用余数累计或绝对时间量化，保证总误差不持续累积；
- 最后一段是否被异常缩短或拉长。

### H2：重采样后的位置差形成了异常隐含速度或加速度

对每个关节、每个发送段计算：

```text
dt[i] = step_num[i] * 0.008
v_adapter[i] = (q[i] - q[i-1]) / dt[i]
a_adapter[i] = (v_adapter[i] - v_adapter[i-1])
               / ((dt[i] + dt[i-1]) / 2)
```

需要与 MoveIt 原始 `velocities[]`、`accelerations[]` 比较，并检查：

- 局部速度峰值；
- 局部加速度峰值；
- 非预期速度符号变化；
- 轨迹末端制动段是否仍保持合理；
- 拆分长段后是否重复或遗漏端点。

### H3：第一命令点与真实状态不连续

进入 servo mode 到发送第一点之间，机器人反馈可能已经变化。若第一发送点仍采用旧规划起点，控制柜会看到位置跳变。

需要记录：

- 规划起点；
- Goal 第一轨迹点；
- 进入 servo mode 前的实际位置；
- 第一条 `servo_j` 发送时的实际位置；
- 第一条命令相对实际位置的误差。

### H4：filter 配置与队列式 `servo_j` 使用方式不匹配

需要确认以下接口的官方语义、参数单位、互斥关系和生效时机：

- `servo_move_use_none_filter()`；
- `servo_move_use_joint_LPF(cutoffFreq)`；
- `servo_move_use_joint_NLF(max_vr, max_ar, max_jr)`；
- `servo_speed_foresight(max_buf, kp)`。

在未确认 JAKA 官方推荐范围之前，不得把滤波参数组合用于大位移真机试验。

### H5：快速填充队列时 `step_num` 语义理解错误

需要确认 `step_num` 表示该目标的控制周期数，而不是调用间隔或重复发送次数，并确认快速连续调用 `servo_j` 时控制柜是否按预期缓存所有段。

## 5. 固定测试输入

专项测试至少保留三类可重复输入，全部绕过在线重新规划：

1. 已成功的小位移 FollowJointTrajectory Goal；
2. 已失败并触发 `0x108481` 的历史 Goal；
3. 从同一历史 Goal 截取的短前缀轨迹。

三类输入均须保存：

- `joint_names`；
- 每个点的 `positions[]`；
- 每个点的 `velocities[]`；
- 每个点的 `accelerations[]`；
- `time_from_start`；
- Goal tolerance。

若历史 CSV 只有适配后的点而没有原始 Goal，则它只能用于部分回放，不能用于判断 MoveIt 到适配层之间的数据损失。后续必须增加原始 Goal 落盘。

## 6. 代码实施阶段

### A：离线转换器与诊断器

把“ROS 轨迹转换成 `servo_j` 发送序列”的计算从硬件调用中分离为纯函数：

```text
TrajectoryPoint[] -> ServoSetpoint[]
```

`ServoSetpoint` 至少包含：

- 六轴绝对目标位置；
- `step_num`；
- 计划段时长；
- 量化后段时长；
- 隐含速度和加速度；
- 原始轨迹点号和拆分段号。

验收条件：不启动 ROS 图、不连接机器人即可回放固定 Goal，并得到确定且可比较的发送序列。

### B：转接算法单元测试

覆盖：

- 关节名乱序后的轴映射；
- 缺少、重复和未知关节名；
- 弧度正负方向保持不变；
- 非递增和重复 `time_from_start`；
- 小于 8 ms 的轨迹段；
- 超过 `maximum_step_num * 8 ms` 的长轨迹段；
- 量化余数累计；
- 零位移段；
- 单调负向轨迹不得生成正向隐含速度；
- 最终目标必须与原始 Goal 最终目标完全一致；
- NaN、Inf 和维度错误必须拒绝。

验收条件：测试覆盖所有边界，历史失败输入能够稳定复现相同的 `ServoSetpoint[]`，不依赖真机。

### C：只记录模式

增加不调用 SDK 的转换预览模式，输出：

- 原始点数和发送段数；
- 原始时长和量化后总时长；
- 每轴最大隐含速度、加速度和符号变化次数；
- 第一命令点连续性；
- `step_num` 最小值、最大值和分布；
- filter 配置；
- 完整 CSV。

验收条件：同一个 Goal 在只记录模式和真机模式下生成完全相同的发送序列。

### D：真机同步遥测

发送线程和反馈线程使用同一稳态时钟记录：

- `command_index`；
- `source_point_index`；
- `steady_timestamp`；
- 六轴发送位置；
- `step_num`；
- `scheduled_dt`；
- joint_1 隐含命令速度；
- joint_1 实际位置；
- SDK 可提供的 joint_1 实际速度；
- 无实际速度接口时，由高频位置差分得到的估算速度；
- SDK 调用耗时；
- `powered/enabled/in_servo_mode/errcode`；
- 当前 filter 类型及参数。

原始位置反馈和差分后的速度都要保存，不能只保存平滑后的结果。反馈采样率应达到驱动可稳定提供的最高频率，并在报告中记录实际采样周期和抖动。

验收条件：发生故障时可以还原故障前至少 1 秒的命令与实际速度关系。

### E：filter 单变量对照

先通过 JAKA SDK 文档确认支持的 filter 配置，再使用同一个固定 Goal 做对照。每轮只改变一个参数：

1. 当前 `0.5 Hz joint LPF` 基线；
2. 调整 LPF cutoff；
3. none filter，仅允许低速小位移验证；
4. joint NLF，仅在参数单位和推荐值确认后验证；
5. foresight，仅在缓存语义和推荐值确认后验证。

不得在同一轮同时修改速度比例、轨迹点、`maximum_step_num` 和 filter。

## 7. 离线通过指标

- 六轴映射、单位和符号完全保持；
- 最终发送位置等于原始 Goal 终点；
- 所有 `step_num >= 1` 且不超过配置上限；
- 原始总时长与量化后总时长误差不随点数线性累积；
- 单调负向 joint_1 输入不会生成正向 `v_adapter`；
- 不产生超出机器人和项目配置上限的隐含速度、加速度；
- 只记录模式和真实发送模式产生相同的 `ServoSetpoint[]`；
- 所有异常输入在调用 JAKA SDK 前被拒绝并给出点号和原因。

## 8. 真机测试顺序

本轮真机测试使用固定关节轨迹，不调用 IK，也不重新运行 MoveIt 规划。

| 编号 | 固定输入 | 唯一变化量 | 目的 | 通过条件 |
| --- | --- | --- | --- | --- |
| S0 | 小位移轨迹 | 只记录，不调用 SDK | 校验转接结果 | 离线指标全部通过 |
| S1 | joint_1 负向 0.02 rad | 当前转接配置 | 校验方向和首点连续性 | 命令/实际方向一致，无报警 |
| S2 | joint_1 负向 0.10 rad | 位移 | 校验连续负向跟踪 | 无速度异号，无报警 |
| S3 | 固定中等位移轨迹 | `maximum_step_num` | 对比分段语义 | 终点、总时长和速度趋势一致 |
| S4 | 与 S3 相同 | LPF cutoff | 评估 LPF 影响 | 遥测可解释且无报警 |
| S5 | 与 S3 相同 | 官方认可的 filter 类型 | 对比 filter 策略 | 遥测优于基线且无报警 |
| S6 | 历史失败轨迹的短前缀 | 使用已通过配置 | 接近故障工况 | 无速度异号，无报警 |
| S7 | 历史失败固定轨迹 | 使用已通过配置 | 最终专项回归 | 全程无 `0x108481` |

任何一级发生以下情况，立即停止升级：

- 实际速度与隐含命令速度在有效速度区间内异号；
- `enabled=false`；
- `errcode` 非零或控制柜报警；
- 轨迹起始跳变；
- 队列饥饿；
- SDK 调用周期持续失控。

报警后不自动重试、不自动重新使能。

## 9. 数据判定

命令速度使用适配层实际发送数据计算，不能用 MoveIt 原始速度代替：

```text
v_commanded[i] = (q_sent[i] - q_sent[i-1])
                 / (step_num[i] * 0.008)
```

实际速度优先使用 JAKA SDK 提供的控制器反馈；若只能通过位置差分估算，则必须记录采样间隔，并设置接近零速的死区，避免用编码器量化噪声误判方向。

专项结论必须回答：

1. `0x108481` 前，控制柜接收到的 joint_1 目标位置和段时长是什么；
2. 这些数据对应的隐含设定速度是否连续、是否始终为负；
3. joint_1 实际速度何时出现异号，持续多久；
4. 异号与 `step_num` 边界、拆段点、filter 状态或队列状态是否相关；
5. 单独修改哪一个转接参数能够稳定消除异常。

## 10. 当前结论

本轮不讨论轨迹为什么产生大幅 joint_1 运动，也不检查 JAKA 控制柜内部电机参数。测试输入一旦选定就保持不变，只验证同一条 ROS 轨迹如何被转换并送入 `servo_j`。

在取得故障前同步速度证据之前，不能认定 `0.5 Hz LPF`、`step_num` 或 JAKA SDK 中的任何一个单项已经构成根因，也不应修改控制柜报警阈值。

## 11. 2026-09-07 实施状态

已在 JAKA fork 的 `jaka_driver` 中完成阶段 A、B 和 C 的第一版：

- `build_queued_servo_schedule()` 是生产发送路径与离线预览共用的纯转换器；
- `QueuedServoSetpoint` 已记录来源点号、拆分段号、计划/量化段时长、六轴隐含速度与加速度，以及原始点速度/加速度；
- 时间量化改为相对整条轨迹的绝对 8 ms 时间轴，不再对每个源段独立向上取整；
- 不能映射到独立控制周期的过密源点会在调用 SDK 前被拒绝；
- `trajectory_adapter_diagnostic` 可使用内置固定输入或原始 Goal CSV 做完全离线预览；
- 真机 Action 收到 Goal 后，会在进入 servo mode 前写出同一时间戳的原始 Goal CSV 和实际发送调度 CSV；落盘失败则不开始运动。
- 真机发送路径增加 `trajectory_start_tolerance` 起点连续性门禁，默认 `0.01 rad`；超限 Goal 在进入 servo mode 前拒绝。

阶段 D 的第一版同步遥测接口见 11.3，但尚未经过真机采样率和 SDK `instVel` 有效性验证；阶段 E 的 filter 单变量对照尚未开始。因此当前仍不能单独证明 `0x108481` 的控制柜侧根因。

### 11.1 离线 S0

此步骤不需要启动 ROS 图，不连接机器人，也不需要放置测试块：

```bash
source /opt/ros/humble/setup.bash
source ~/jaka_ros2/install/setup.bash

ros2 run jaka_driver trajectory_adapter_diagnostic \
  --preset joint1_negative_quintic_0p02 \
  --output /tmp/jaka_0x108481_quintic_0p02.csv \
  --maximum-start-error 0.002 \
  --maximum-velocity 0.05 \
  --maximum-acceleration 0.5
```

预期关键词：

```text
joint_1: ... positive_segments=0, negative_segments=20, sign_changes=0
TRAJECTORY ADAPTER OFFLINE: PASS
```

再运行 `joint1_negative_quintic_0p10` preset，其他参数不变。两个五次曲线
preset 都通过后，S0 完成。

`joint1_negative_0p02` 和 `joint1_negative_0p10` 现在专门保留为旧线性输入的
离线对照。诊断器已把静止到第一段速度、最后一段速度到静止的边界阶跃纳入
等效加速度；在 `--maximum-acceleration 0.5` 下，这两个旧线性 preset 应被拒绝，
不能再把它们的 `PASS` 当成真机升级条件。

### 11.2 历史 Goal 离线回放

驱动的新版本在每次 Action 执行前产生：

```text
/tmp/jaka_source_goal_<同一时间戳>.csv
/tmp/jaka_servo_schedule_<同一时间戳>.csv
/tmp/jaka_trajectory_<同一时间戳>.csv
```

原始 Goal 可直接重新送入离线诊断器：

```bash
ros2 run jaka_driver trajectory_adapter_diagnostic \
  --input /tmp/jaka_source_goal_<时间戳>.csv \
  --output /tmp/jaka_0x108481_history_preview.csv \
  --maximum-start-error 0.002 \
  --maximum-velocity <项目确认上限> \
  --maximum-acceleration <项目确认上限>
```

阶段 S1 至 S7 都是自由空间固定关节轨迹试验，不使用人体、软垫或推拿测试块。测试块仅属于 R10 接触恒力测试，两类试验不能混在同一轮。真机专项测试前还必须确认系统中只有一个 `jaka_driver` 进程和一个 FollowJointTrajectory Action server。

### 11.3 S1 固定 Goal 接口和同步速度遥测

补充实现后，阶段 D 已具备第一版真机采集接口：

- `joint1_negative_tracking.launch.py` 生成固定点数、固定持续时间的 joint_1 负向 Goal，不调用 MoveIt 规划器；
- `activate` 默认为 `false`，实际发送还要求 `parameters_confirmed=true`；
- 仅允许 joint_1 负向 `[-0.10,-0.001)` rad，持续时间不得短于 2 秒；
- 发送前要求新鲜六轴反馈、机器人空闲/上电/使能/无碰撞，以及唯一 Action server；
- 驱动 CSV 增加实际采样周期、调度隐含命令速度、原始位置差分速度和可选 SDK `instVel`；
- 位置和差分速度始终通过较快的 `get_joint_position()` 采集；完整 `get_robot_status()` 只按独立低频周期读取，避免它阻塞每个位置反馈周期；
- `trajectory_capture_sdk_joint_velocity=true` 时读取 SDK `instVel`，`trajectory_sdk_joint_velocity_period` 默认 `1.0 s`；
- SDK 头文件没有注明 `instVel` 单位，因此 CSV 字段命名为 `sdk_inst_velocity_raw_joint_*`，并记录 `sdk_velocity_sample_elapsed_s` 和 `sdk_status_call_duration_s`。在获得官方单位依据前，仅用它判断符号，不与 `rad/s` 直接比较；
- SDK 辅助速度读取失败只记录警告并继续使用位置差分速度；机器人安全状态仍由简化状态接口独立检查。

启动真机驱动时启用专项采集：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.66.200 \
  connect:=true \
  start_move_group:=false \
  auto_home:=false \
  trajectory_feedback_period:=0.02 \
  trajectory_status_period:=0.05 \
  trajectory_capture_sdk_joint_velocity:=false \
  trajectory_servo_filter_mode:=joint_lpf \
  trajectory_servo_filter_cutoff_hz:=0.5
```

登录、上电和使能后，先执行不会发送 Goal 的门禁：

```bash
ros2 launch jaka_driver joint1_negative_tracking.launch.py \
  activate:=false \
  joint_delta:=-0.02 \
  duration:=2.0
```

必须看到 `JOINT1 NEGATIVE TRACKING: SAFE IDLE`。清空运动范围、确认 joint_1 负向仍有机械余量后，才执行 S1：

```bash
ros2 launch jaka_driver joint1_negative_tracking.launch.py \
  activate:=true \
  parameters_confirmed:=true \
  joint_delta:=-0.02 \
  duration:=2.0 \
  endpoint_tolerance:=0.002
```

S1 通过后收集同一时间戳的三份 CSV。位置差分速度是本阶段的主判据；SDK `instVel` 是低频辅助判据。驱动结束日志会直接给出 `joint_1_sign[...]` 汇总，要求 `estimated_positive=0`、`sdk_raw_positive=0`，且位置采样不再被每次完整状态读取持续阻塞。若发生报警，禁止自动重试或重新使能，直接保留 CSV 和驱动日志。

### 11.4 S1 首轮结果与遥测修正

2026-09-07 首轮 `joint_delta=-0.02 rad, duration=2.0 s` 真机测试通过：

- 原始轨迹和控制柜调度时长均为 `2.0 s`，起点误差与调度时间误差均为零；
- 20 个调度段全部为 joint_1 负向，正向段和符号变化均为零；
- `starved_segments=0`，机器人全程上电、使能且错误码为零；
- 最终误差 `0.001547245 rad`，小于 `0.002 rad` 门限；
- 位置差分速度和 SDK `instVel` 原始值在有效样本中均为负向，没有出现 `0x108481`。

首轮同时发现完整 `get_robot_status()` 将位置反馈周期拉长到 `0.054306-0.531765 s`，2 秒轨迹仅获得 8 个运动中样本。SDK 原始速度约为 `-0.345` 至 `-0.563`，数值上符合 `deg/s` 的可能性，但 SDK 头文件没有注明单位。

因此在进入 S2 前先完成以下修正并重复 S1：

1. 高频位置反馈固定使用 `get_joint_position()`；
2. SDK `instVel` 默认每 `1.0 s` 低频辅助采样一次；
3. CSV 明确保留 SDK 原始值、采样时刻和调用耗时；
4. 日志直接汇总 joint_1 正负速度样本，减少人工 CSV 检查。

### 11.5 S2 失败后的设施补齐（2026-09-08）

固定线性 Goal `joint_delta=-0.10 rad, duration=5.0 s, point_count=21` 在约
`1.93 s` 后停止，示教器再次显示 `0x108481`。驱动记录显示 20 个发送段均为
负向、没有符号变化且没有队列饥饿，但最终只完成约 `0.011433 rad`，随后
机器人下使能。组内其他成员在未使用本 fork 轨迹适配层时也遇到相同报警，
因此当前证据说明：适配层不是触发 `0x108481` 的必要条件。组员观察到的
ROS Service 高频调用导致“越动越慢”属于另一项队列供给问题，不应和该报警
合并成同一个根因。

旧诊断遗漏了速度边界瞬态。对上述线性 Goal，第一段隐含速度会从零直接跳到
近似恒定负速度，结束时再直接回零；`servo_j` 不消费 Goal 中填写的
`velocities[]` 和 `accelerations[]`，所以把数组首尾写成零并不能让控制柜收到
平滑启停。固定 Goal 和离线 preset 因此新增五次时间标定曲线：

```text
s(u) = 10u^3 - 15u^4 + 6u^5
```

它保持位置单调，并使解析起终点速度、加速度为零。此次修改是降低测试输入的
启停激励、建立可比较基线，不是宣称已找到控制柜报警根因。

驱动定位设施同时补齐：

- `trajectory_status_period` 独立于位置反馈周期，默认每 `0.05 s` 检查一次
  `powered/enabled/errcode/in_servo_mode`；一旦异常立即保存 `controller_fault`
  样本并中止，不再等终点超时；
- 预检日志和离线诊断同时输出内部加速度、起停边界速度阶跃及按单个 `8 ms`
  周期计算的边界等效加速度；
- 每次 Goal 在进入 servo mode 前显式选择且只选择一种 filter：`none`、
  `joint_lpf`、`joint_nlf` 或 `foresight`；基线仍为 `joint_lpf(0.5 Hz)`；
- `get_last_error()` 只有在提供 `trajectory_error_code_file_path` 时启用，且读取
  发生在 `motion_abort()` 之前。该文件未随当前仓库提供，需要向 JAKA 获取与
  当前 SDK/控制器匹配的错误码文件；未配置时仍保留简化状态和示教器报警记录；
- `trajectory_capture_sdk_joint_velocity` 默认关闭，位置差分速度仍是主要判据，
  避免完整状态读取阻塞高频反馈；成功、取消和异常退出都会打印相同的速度方向
  与采样周期汇总，不必先人工读取 CSV 才能决定是否停止升级测试。

真机升级必须从 `quintic -0.02 rad / 2 s` 重新开始。旧线性 `-0.10 rad` 仅做
离线对照；固定 Goal 程序会拒绝 `activate=true` 与 `motion_profile=linear` 的
组合。filter 对照要等五次曲线小位移基线稳定通过后进行，
每次只改变 filter，不能同时更改位移、持续时间或点数。

## 12. 下一轮真机测试清单

本专项全程处于自由空间，不放置推拿测试块，不接触人体，不启动 MoveIt。每一级
只执行一次并检查结果；出现报警、下使能、退出 servo mode、速度异号或队列饥饿
时立即停止，不自动重试、不自动清错或重新使能。

### Q0：唯一进程与状态门禁

1. 人员离开机械臂工作空间，确认 joint_1 负向至少还有 `0.12 rad` 机械余量，
   急停可随时触及。
2. 关闭旧终端中的 driver、MoveIt 和固定 Goal，确认系统只有一个 driver 和一个
   FollowJointTrajectory Action server。
3. 使用 11.3 的基线参数启动 `real.launch.py`，再依次登录、上电、使能。
4. `/jaka_driver/robot_states` 必须为
   `motion=0,power=1,servo=1,collision=0`。
5. 先以 `activate=false` 运行固定 Goal；必须出现
   `JOINT1 NEGATIVE TRACKING: SAFE IDLE`，且机械臂不运动。

### Q1：平滑小位移基线

```bash
ros2 launch jaka_driver joint1_negative_tracking.launch.py \
  activate:=true \
  parameters_confirmed:=true \
  joint_delta:=-0.02 \
  duration:=2.0 \
  point_count:=21 \
  motion_profile:=quintic \
  endpoint_tolerance:=0.002
```

通过条件：

- 固定 Goal 输出 `JOINT1 NEGATIVE TRACKING: PASS`，终点误差不大于
  `0.002 rad`；
- 预检中 `positive=0`、`negative=20`、`sign_changes=0`；
- 日志明确显示 `servo filter ... joint_lpf(cutoff=0.500000 Hz)`；
- 没有 `controller_fault`、没有下使能、没有 `0x108481`；
- 速度汇总中 `estimated_positive=0`；SDK 速度默认关闭，因此
  `sdk_samples=0` 是预期结果；
- 三份同时间戳 CSV 均生成：`jaka_source_goal`、`jaka_servo_schedule` 和
  `jaka_trajectory`。

### Q2：历史故障同位移/时长的平滑轨迹

只有 Q1 日志复核通过后才执行：

```bash
ros2 launch jaka_driver joint1_negative_tracking.launch.py \
  activate:=true \
  parameters_confirmed:=true \
  joint_delta:=-0.10 \
  duration:=5.0 \
  point_count:=21 \
  motion_profile:=quintic \
  endpoint_tolerance:=0.002
```

Q2 使用和历史失败测试相同的位移、持续时间和点数，只把位置时间律从线性改为
五次曲线。通过条件与 Q1 相同。Q2 通过一次后，需要回到可比较的起始关节区间，
再独立重复两次以验证稳定性；不要连续累积负向位移而忽略关节余量。

### Q3：filter 单变量对照

只有 Q2 连续三次通过后再设计。每轮重启 driver，只改变
`trajectory_servo_filter_mode` 或其一个参数，固定 Q1 的小位移 Goal。首个对照为
`none`，仅用于低速小位移；`joint_nlf` 和 `foresight` 必须等 JAKA 提供推荐参数
及错误码文件后再开放。不得直接用 `none` 执行 `-0.10 rad`。
