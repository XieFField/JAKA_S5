# MuJoCo 仿真功能使用说明

本文说明如何在 Ubuntu 22.04 + ROS 2 Humble 宿主机上直接安装、编译和使用
JAKA S5 MuJoCo 仿真。仿真入口不会连接或控制真实机械臂。

## 1. 功能范围

当前 MuJoCo 入口包含：

- JAKA S5 与按摩头动力学仿真、原生 Viewer 和六轴位置/力矩控制；
- MoveIt 轨迹规划、MoveIt Servo 笛卡尔控制、键盘控制和 TCP 控制球；
- `/joint_states`、仿真时钟、末端力传感器、接触状态和控制权状态；
- 关键点示教、保存和回放；
- TCP 目标/实际位姿、速度和轨迹诊断；
- 恒力接近、掉力重建、切向贴面运动和超力退出；
- 可选俯卧假人、局部柔性背部及 MoveIt 人体/按摩床碰撞场景；
- 接触曲线监控与无界面 YAML 回归实验。

俯卧假人、RViz 和曲线监控默认关闭。假人的柔性背部会显著增加计算量，只在需要
背部接触实验时开启。

## 2. 原生环境安装

支持的运行基线是 Ubuntu 22.04 和 ROS 2 Humble Desktop。不要在 Ubuntu 24.04
中混装 Humble 二进制包。

先按根目录 README 构建配套 `jaka_ros2` 工作区（固定版本沿用 README）。
保持 `jaka_ros2` 与本仓库 `massage_robot_ws` 并列。若供应商工作区放在其他位置，
先设置 `export JAKA_VENDOR_WORKSPACE=/实际路径/jaka_ros2`。然后在本仓库根目录执行：

```bash
./scripts/native-install-deps.sh
./scripts/native-colcon-build.sh
```

安装脚本会使用 `rosdep` 安装 ROS 依赖，在仓库 `.venv` 中安装固定版本的
`mujoco==3.12.0` 和 `numpy==1.26.4`，并串行构建 MoveIt 2.5.9 退出修复到
`.native/moveit_shutdown_overlay`。

固定 NumPy 1.x 是为了兼容 Ubuntu 22.04 上的 ROS 2 Humble Python ABI。MoveIt
overlay 解决 `move_group` 退出时控制器插件和回调组析构顺序导致的段错误；脚本会
校验系统包版本，只对已验证的 MoveIt 2.5.9 应用补丁。

每个新终端都要先加载环境：

```bash
cd ~/robot_project/massage_robot_ws
source scripts/native-env.bash
```

## 3. 启动仿真

### 3.1 标准启动

```bash
ros2 launch massage_bringup mujoco_sim.launch.py
```

默认开启 MuJoCo Viewer、MoveIt、Servo、键盘和 TCP 控制球；默认关闭 RViz、
俯卧假人和曲线监控。

### 3.2 低负载启动

无图形界面、无键盘、无 MoveIt 的最低负载模式：

```bash
ros2 launch massage_bringup mujoco_sim.launch.py \
  use_mujoco_viewer:=false \
  use_moveit:=false \
  use_servo:=false \
  use_keyboard:=false \
  use_tcp_control_ball:=false
```

需要 MoveIt 但不需要图形界面时：

```bash
ros2 launch massage_bringup mujoco_sim.launch.py \
  use_mujoco_viewer:=false use_rviz:=false use_keyboard:=false
```

### 3.3 RViz 与监控曲线

```bash
ros2 launch massage_bringup mujoco_sim.launch.py use_rviz:=true
ros2 launch massage_bringup mujoco_sim.launch.py \
  use_monitor:=true plot_group:=contact
```

`plot_group` 可选 `joint`、`contact`、`tcp_position` 或 `tcp_velocity`。

## 4. 机械臂交互控制

### 4.1 MuJoCo Viewer

Viewer 右侧 `Control` 中包含六个关节位置滑块和六个关节力矩滑块。拖动滑块会
中止当前轨迹，并把控制权切换到 `MUJOCO_JOINT_CONTROL`。

按摩头末端的青色球是 World XYZ TCP 目标：

- `Ctrl + 鼠标右键拖动`：在当前视图的竖直平面移动；
- `Ctrl + Shift + 鼠标右键拖动`：在水平平面移动。

TCP 控制球保持末端姿态，只改变位置，指令经过 MoveIt Servo 的关节限位、奇异点
和碰撞检查后进入 MuJoCo。

### 4.2 键盘

启动命令所在终端接收键盘输入：

| 按键 | 功能 |
| --- | --- |
| `w/s`、`a/d`、`r/f` | X、Y、Z 平移 |
| `u/j`、`i/k`、`o/l` | Roll、Pitch、Yaw 旋转 |
| `1`、`2` | World、Tool 坐标系 |
| `q` | 退出键盘节点 |

如需独立键盘终端，主启动命令加 `use_keyboard:=false`，再在已加载原生环境的新终端
运行：

```bash
ros2 run massage_motion cartesian_jog_keyboard
```

### 4.3 控制模式

```bash
ros2 service call /massage_mujoco/set_position_mode std_srvs/srv/Trigger
ros2 service call /massage_mujoco/set_hold_mode std_srvs/srv/Trigger
ros2 service call /massage_mujoco/set_gravity_mode std_srvs/srv/Trigger
```

- `POSITION`：接受轨迹、Servo、键盘和 TCP 控制；
- `HOLD`：保持当前位置；
- `GRAVITY`：关闭位置执行器，只保留配置的重力补偿。

## 5. 俯卧背部按摩场景

普通入口不加载假人。背部场景的默认启动同样不加载假人：

```bash
ros2 launch massage_bringup mujoco_prone_massage.launch.py
```

显式开启俯卧假人和柔性背部：

```bash
ros2 launch massage_bringup mujoco_prone_massage.launch.py \
  use_prone_mannequin:=true
```

需要同时观察 MoveIt 场景时：

```bash
ros2 launch massage_bringup mujoco_prone_massage.launch.py \
  use_prone_mannequin:=true use_rviz:=true
```

开启假人后使用背部对准初始姿态：TCP 位于柔性背部上方，工具局部 `+Z` 指向世界
`-Z`，初始表面间隙约 42 mm。MoveIt PlanningScene 同时注入背部、躯干、头部、
四肢和按摩床共 9 个碰撞体，仅允许按摩头与柔性背部接触。关闭假人后恢复普通
MuJoCo 初始姿态。

柔性背部使用 `5 × 7 × 3` 体积网格，默认杨氏模量为 `1000 Pa`。这些参数只是
控制开发初值，没有经过人体组织标定。

## 6. 轨迹示教与回放

先用 Viewer、键盘或 MoveIt 移动机械臂。等控制权回到 `IDLE` 后记录关键点：

```bash
ros2 service call /massage_mujoco/teach/record_point std_srvs/srv/Trigger
```

移动到下一姿态并重复记录，然后保存和回放：

```bash
ros2 service call /massage_mujoco/teach/save std_srvs/srv/Trigger
ros2 service call /massage_mujoco/teach/replay std_srvs/srv/Trigger
ros2 topic echo /massage_mujoco/teach/status --qos-durability transient_local
```

停止、清空和重新加载：

```bash
ros2 service call /massage_mujoco/teach/stop std_srvs/srv/Trigger
ros2 service call /massage_mujoco/teach/clear std_srvs/srv/Trigger
ros2 service call /massage_mujoco/teach/load std_srvs/srv/Trigger
```

默认文件为 `log/mujoco/teach_trajectory.yaml`。可用
`teach_trajectory_file:=绝对路径` 修改保存位置。回放会检查关节限位并参与控制权
仲裁，但不会重新调用 MoveIt 做碰撞规划。

## 7. TCP 诊断

```bash
ros2 topic echo /massage_mujoco/tcp/diagnostics
```

主要话题：

| 话题 | 内容 |
| --- | --- |
| `/massage_mujoco/tcp/target_pose` | 目标 TCP 位姿 |
| `/massage_mujoco/tcp/actual_pose` | 实际 TCP 位姿 |
| `/massage_mujoco/tcp/actual_velocity` | 仿真时间下的实际线速度 |
| `/massage_mujoco/tcp/diagnostics` | 速度、位置误差、速度比和控制权 |
| `/massage_mujoco/tcp/traces` | RViz 目标/实际轨迹 MarkerArray |

## 8. 恒力贴面运动

俯卧背部入口已设置沿世界 `-Z` 接近、沿 `+X` 切向移动。开启假人并启动仿真后，
在另一个已加载环境的终端执行：

```bash
ros2 service call /massage_mujoco/surface_follow/start std_srvs/srv/Trigger
ros2 topic echo /massage_mujoco/surface_follow/status \
  --qos-durability transient_local
```

停止运动：

```bash
ros2 service call /massage_mujoco/surface_follow/stop std_srvs/srv/Trigger
```

状态顺序为 `APPROACH`、`ESTABLISH_FORCE`、`FOLLOW`，最后进入 `COMPLETE` 或
`FAULT`。掉力后会停止切向运动并重新建力；接近超时、超过配置中的仿真力阈值或
发生非预期碰撞时会输出零速度并退出。

## 9. 配置参数

物理、接触、假人材料和人体布局参数统一位于
`src/massage_mujoco/config/mujoco.yaml`。常用 Launch 参数如下：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `use_prone_mannequin` | `false` | 开启俯卧假人和柔性背部 |
| `use_rviz` | `false` | 开启 RViz |
| `use_mujoco_viewer` | `true` | 开启 MuJoCo Viewer |
| `use_keyboard` | `true` | 开启键盘控制节点 |
| `use_monitor` | `false` | 开启 rqt_plot |
| `realtime_factor` | `1.0` | 目标实时倍率 |
| `surface_follow_target_force` | `2.0` | 仿真目标法向力，单位 N |
| `surface_follow_duration` | `3.0` | 有效切向运动时间，单位 s |
| `surface_follow_maximum_approach_distance` | `0.05` | 最大接近距离，单位 m |

查看完整参数：

```bash
ros2 launch massage_bringup mujoco_sim.launch.py --show-args
```

## 10. 低负载验证

构建后运行单元测试：

```bash
./scripts/native-colcon-test.sh
```

只验证 MuJoCo 模型加载和位置控制，不启动 GUI、MoveIt 或 ROS 图：

```bash
source scripts/native-env.bash
ros2 run massage_mujoco mujoco_smoke --duration 1.0 --settle-duration 1.0
```

确认默认配置没有加载假人：

```bash
ros2 launch massage_bringup mujoco_sim.launch.py --show-args | \
  grep use_prone_mannequin
```

## 11. 使用边界

本功能是仿真开发与软件回归工具，不是真人按摩或真机安全验收。仿真中的材料、
接触力阈值、摩擦和控制增益均需单独标定后，才能进入经授权的真机测试。MoveIt
碰撞检查降低了规划穿模风险，但不能替代真机限位、急停、力传感器、工具/TCP、
负载和人体接触安全验证。

移植说明：MuJoCo 使用包内独立初始姿态配置，默认六轴角度为 `[-180, 90, -90, 90, 90, -75]` 度；不修改供应商工作区的初始姿态。

本次上游移植验证：基于供应商 `504362f` 的描述资源，23 项定向无界面测试通过，覆盖示教/TCP/贴面逻辑、轨迹、接触代理、控制映射和模型运动学。Python、XML、Shell 语法及文档链接检查通过。测试限制为 1 CPU、2 GiB 内存。本次未执行完整 C++ 构建、GUI 联调或真机验证；上述结果不能代替双工作区的完整运行验收。
