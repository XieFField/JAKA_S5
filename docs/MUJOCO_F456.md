# MuJoCo F4-F6：接触、监控和无界面实验

F4-F6 延续原来的 Gazebo / MuJoCo 双后端结构，不涉及强化学习。厂家 URDF
保持原样；以下新增参数均位于 MuJoCo 配置中。

## 启动

进入工程并加载原生环境：

```bash
cd ~/robot_project/massage_robot_ws
source scripts/native-env.bash
ros2 launch massage_bringup mujoco_sim.launch.py use_rviz:=false use_monitor:=true plot_group:=joint plot_joint:=1
```

键盘仍从启动 launch 的终端读取；操作时让该终端获得焦点。
`use_monitor` 默认 `false`，原启动命令继续可用。停止仍使用 Ctrl+C。

## F4：接触与碰撞

新增 `massage_msgs` 标准 ROS 接口包：

| 话题 | 消息 | 内容 |
| --- | --- | --- |
| `/massage_mujoco/contact_state` | `massage_msgs/msg/ContactState` | 每次状态更新的接触点、总法向力、状态和本次边沿事件 |
| `/massage_mujoco/contact_events` | 同上 | 仅发生状态边沿时发布完整快照 |
| `/massage_ft_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` | 原有传感器坐标系六维力/力矩，保持原语义 |

每个接触点包含两个 body/geom 名称、世界坐标位置、由 A 指向 B 的单位法向、
穿透深度（m）、法向力（N）和是否属于按摩接触对。法向力来自
[`mj_contactForce`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-contactforce)，
不会把 FT 的模长当成接触法向力。

默认按摩接触对为 `massage_head_link` / `contact_pad`，两者顺序无关。
法向力累计只包含该接触对；其他有力接触单独标记 `unexpected_contact`。
阈值配置位于 `src/massage_mujoco/config/mujoco.yaml`：

- `contact_force: 0.05` N：指定接触对存在且累计力达到阈值才进入接触状态。
- `over_force: 20.0` N：超力状态。
- `impact_force_rate: 200.0` N/s：接触力正向增长率阈值。

事件包括 `CONTACT_STARTED/CONTACT_ENDED`、`OVER_FORCE/OVER_FORCE_CLEARED`、
`IMPACT/IMPACT_CLEARED`、`UNEXPECTED_CONTACT/UNEXPECTED_CONTACT_CLEARED`。
这组阈值是仿真诊断初值，不是人体接触验收值，也不触发自动停机。
默认每 10 个 1 ms 物理步检测一次，即仿真时间 100 Hz；`IMPACT` 是采样力增长率，
不是冲量测量，可能漏掉采样间隔内的短暂事件。Viewer Restart 会清空检测历史。

`collision_geometry: mesh` 保留原碰撞模型。
可选 `box` 为机械臂碰撞网格建立包围盒代理，保留 STL 显示和 URDF 质量、惯量、FK；
相邻关节连接体显式排除互撞，其他碰撞对继续检测。按摩头和接触块原有基础几何保留。
包围盒更保守，接触面与网格不同，不能将其接触结果当成网格模型等价结果。

要试用代理，复制完整配置、把 `collision_geometry` 改为 `box`，通过
`config_path:=/绝对路径/你的配置.yaml` 传给原 launch。

## F5：曲线和 Viewer 调试显示

预设在 `src/massage_bringup/config/mujoco_plots.yaml`，使用依赖安装脚本安装的
`rqt_plot`：

| `plot_group` | 曲线和单位 |
| --- | --- |
| `joint` | `plot_joint:=1` 到 `6`，该轴目标/实际/误差，rad |
| `effort` | 六轴执行器广义力，N·m |
| `force` | FT 局部 Fx/Fy/Fz，N |
| `torque` | FT 局部 Tx/Ty/Tz，N·m |
| `contact` | 指定接触对累计法向力，N |
| `tcp_position` | TCP 目标/实际 XYZ，m |
| `tcp_velocity` | TCP 实际 XYZ 速度，m/s |
| `all` | 六个关节窗口加六个传感器窗口，共 12 个 |

仿真已启动时，也可以在另一个已加载环境的终端单独开曲线：

```bash
source scripts/native-env.bash
ros2 launch massage_bringup mujoco_monitor.launch.py plot_group:=contact
```

以下可选 launch 参数默认关闭，也可在 Viewer 原生 Visualization 中调整：

- `show_contact_points:=true`：接触点。
- `show_contact_forces:=true`：接触力箭头。
- `show_site_frames:=true`：FT/TCP site 坐标轴。
- `show_collision_proxies:=true`：group 3 碰撞代理（仅 box 配置会生成）。

Simulation / Visualization / Control 面板继续保留。Control 手动目标仍受现有
控制权规则约束；右侧六个 actuator 滑块分别接管 J1-J6，末端青色球通过 Viewer
原生 perturb 操作接管 World XYZ。曲线监控本身不获取控制权。rqt_plot 使用
自身时间轴，数值来自仿真话题；加速、暂停、Restart 后不要将屏幕时间跨度直接
当成物理响应延迟。

## F6：无界面场景、报告和对照

直接运行实验，不需要启动 Viewer 或 ROS 仿真节点：

```bash
source scripts/native-env.bash
ros2 run massage_mujoco mujoco_regression run src/massage_mujoco/scenarios/regression.yaml --output log/mujoco-f456/regression
```

示例套件覆盖预备姿态、微小关节轨迹、工具外力和低速压入接触块。
预备姿态扫描 mesh/box，接触扫描 500/600 N/m，每个配置重复两次，共 12 次实验。
不连接硬件，不发布 ROS 控制话题，可与另一隔离仿真并存。

YAML 格式：

- 顶层 `version: 1`、`repeat`、`determinism_tolerance`、`scenarios`。
- 场景 `name`、`duration`；可选 `initial_positions` 为六关节 rad，默认项目预备姿态。
- `sample_period` 默认 0.01 s；时间必须是物理步长的整数倍。
- `trajectory` 使用从实验开始计时的递增 `time`，点内使用绝对 `positions` 或
  相对初始姿态的 `delta`，按物理步线性插值，最后目标保持至实验结束。
- `control_mode` 可选 POSITION/HOLD/GRAVITY，后两者不接受轨迹。
- `external_wrenches` 指定 `start/end/force/torque`；在 FT 原点施加，使用每一步
  当前传感器坐标系，多个重叠载荷相加。
- `model_overrides` 递归覆盖 MuJoCo 配置，例如 `contact_pad` 的尺寸、位置、刚度。
- `sweep` 用点分隔配置路径定义扫描值，多个参数取笛卡尔积。
- `assertions` 对指标指定 `min/max`；`required_events` 指定必须观察到的事件。

输出目录包括 `report.json`（配置、版本、指标、事件、失败原因）、每次实验的
CSV 曲线以及 `junit.xml`。指标包括终点/峰值/RMS 关节误差、接触力、最大穿透、
执行器力矩、FT 力模长、非预期接触采样数和重复差异。峰值均基于采样记录。
每次重复重新创建模型，逐项比较完整数值曲线；当前同机重复默认容差 `1e-10`。
断言失败退出码为 1，输入/文件错误为 2；报告不会把失败实验标成成功。

相同实验的跨后端报告对照：

```bash
ros2 run massage_mujoco mujoco_regression compare log/gazebo/report.json log/mujoco-f456/regression/report.json --tolerances src/massage_mujoco/scenarios/comparison_tolerances.yaml --output log/mujoco-f456/comparison.json
```

比较器接受 Gazebo 导出的同格式报告：顶层 `backend` 和 `cases`，每个 case 具有
`case_id`、`passed`、`metrics`、可选 `parameters`。case ID 格式为
`场景名[扫描序号]#重复序号`。缺用例、失败用例、缺指标、不同扫描参数或指标差异
超过显式绝对容差均判失败。示例容差面向低力接触工位，需要按实验调整。
此入口比较已经采集的报告，不自动运行 Gazebo 或注入力；本轮不以自比较代替
Gazebo 动态验收。

性能对照命令：

```bash
ros2 run massage_mujoco mujoco_regression benchmark src/massage_mujoco/scenarios/regression.yaml --output log/mujoco-f456/benchmark.json
```

每个场景/扫描参数先预热，再交替进行 5 对运行，取中位数。基线与监控组均包含
相同物理仿真和数值曲线整理，区别是是否读取和分类接触；速度下降超过 20% 返回失败。
此结果衡量 core 及采样接触诊断开销，不包括 ROS 传输、rqt_plot 或 Viewer 绘制，
也不是键盘端到端延迟。

## 本轮验证记录（2026-09-07）

- 当时的 11 包构建与 MuJoCo 49 项测试及 flake8/pep257 均通过。
- 12 次 YAML 实验全部通过；每组两次运行的完整数值曲线差异为 0。
- 500/600 N/m 接触块的稳态法向力分别为 0.196517 / 0.226208 N；
  真实接触点和 `CONTACT_STARTED` 已通过 ROS 消息订阅验证。
- 完整 launch 能启动仿真、MoveIt、Servo、桥接与 rqt_plot；第六轴曲线的目标、
  实际、误差均建立了订阅。rqt_plot 使用 Qt offscreen 做启动验证。
- MuJoCo Viewer 实际启动通过，四个调试显示参数可用，测试结束后正常退出。
- 五对交替性能测试中，六个场景配置的速度降幅为 16.07%、0.00%、0.33%、
  3.17%、15.12%、19.64%，均在 20% 内；对应原始报告位于
  `log/mujoco-f456/final-benchmark.json`。该文件与实验报告属于本地运行产物，
  不纳入 Git。
- 本轮没有重新跑 Gazebo 动态实验；跨后端比较器已验证差异、缺用例和失败判定，
  Gazebo 的历史验收记录继续见迁移计划。
