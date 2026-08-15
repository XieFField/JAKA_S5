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

demo 完成后仿真基础设施会继续运行，便于检查 TF、Planning Scene 和控制器。检查完成后
在 launch 终端按 `Ctrl-C` 统一关闭。保护路径示例仅用于仿真：

```bash
ros2 launch massage_bringup compliant_press_task_demo.launch.py \
  force_limit:=0.2 expect_limit_exceeded:=true
```

`0.1 N` 接触阈值、`1.0 N` 上限和仿真导纳参数都不是人体或真机参数。

## 真机启动

### 1. 安全的只读启动

真机入口要求显式传入控制柜 IP，但不会自动登录、上电、使能或运动：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.x.x \
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

只有现场人员确认急停、工作空间和机器人状态后，才按顺序调用：

```bash
ros2 service call /jaka_driver/login std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/power_on std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/enable_robot std_srvs/srv/Trigger "{}"
```

确认 `/joint_states` 与真机一致后，再重启或另行启动 MoveIt：

```bash
ros2 launch massage_bringup real.launch.py \
  robot_ip:=192.168.x.x connect:=true start_move_group:=true use_rviz:=true
```

结束时按以下顺序关闭：

```bash
ros2 service call /jaka_driver/stop_move std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/enable_admittance std_srvs/srv/SetBool "{data: false}"
ros2 service call /jaka_driver/disable_robot std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/power_off std_srvs/srv/Trigger "{}"
ros2 service call /jaka_driver/logout std_srvs/srv/Trigger "{}"
```

首次真机验收必须依次完成：只读 -> 上电使能 -> 非人体单关节小角度 -> 低速 PTP ->
刚性工装 FT -> 柔性测试块按压。详细检查项见
[`docs/jaka_real_hardware_integration.md`](docs/jaka_real_hardware_integration.md)。

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

然后在服务器重新执行 rosdep、构建和测试。现场 IP、FT 参数及安全上限放在服务器私有
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
- [推拿手法与视觉反馈方案](docs/massage_technique_vision_plan.md)
- [JAKA 真机联调检查表](docs/jaka_real_hardware_integration.md)
- [规划执行实验记录](docs/planning_execution_demo_report.md)

## License

本项目自有代码采用 BSD-3-Clause。JAKA 官方源码、SDK 动态库、机器人模型和其他第三方
组件继续遵循各自许可证；本项目许可证不会覆盖供应商组件。
