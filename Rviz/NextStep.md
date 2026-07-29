# MoveIt 控制真实 Franzi 机器人的下一步

## 目标

通过 RViz 的 MoveIt 交互界面规划运动轨迹，进行碰撞检测和轨迹预览，再将轨迹
安全地下发到真实机器人执行。

```text
RViz / MoveIt
     │  FollowJointTrajectory
     ▼
franzi_hardware_bridge
     │  MoveJ / CommandJointPosition / MoveStop
     ▼
franzi_sdk → 真实控制柜 / 机器人

真实反馈 → bridge → /joint_states → robot_state_publisher → MoveIt / RViz
```

## 现有基础

`src/franzi_sdk` 已经提供以下底层能力：

- 真实关节反馈：`GetMotionRobotState()`、`ReadJointState()`、
  `GetCurJointPos()`。
- 运动命令：`MoveJ(joint_path=...)`、`CommandJointPosition(...)`。
- 停止和安全状态：`MoveStop()`、`SoftStop()`、急停、故障、使能和运动状态查询。
- MoveIt 配置已定义六个轨迹控制器接口：躯干、头部、左右臂、左右夹爪。

当前 RViz 演示中的虚拟控制器只用于可视化，不会控制真实硬件。

## 缺少的核心组件

需要新建 ROS 2 包，例如 `franzi_hardware_bridge`。它是 MoveIt 与 SDK 之间的
唯一硬件接口，至少应完成以下功能：

| 功能 | 要求 |
|---|---|
| 状态发布 | 周期读取 SDK 的关节位置和速度，按 URDF 的关节名称发布 `/joint_states`。 |
| 轨迹 action | 提供 `FollowJointTrajectory` action，例如 `/left_arm_controller/follow_joint_trajectory`。 |
| 轨迹转换 | 将 MoveIt 的关节名称、位置、速度、加速度和时间戳转换成 SDK 支持的命令。 |
| 执行反馈 | 用真实关节反馈持续发布 action feedback；只有真实到位时才返回成功。 |
| 停止/取消 | 收到 Cancel、超时、偏差过大、故障或急停时，调用 `MoveStop()` 或 `SoftStop()`，并向 MoveIt 返回失败。 |
| 安全检查 | 执行前检查通信、急停、故障、使能、控制模式、关节数量和起点误差。 |

## 轨迹下发策略

SDK 的 `MoveJ(joint_path)` 可接收路径点，但其接口不包含 MoveIt 的
`time_from_start`、速度和加速度。因此 SDK 可能自行插补或重规划，实际运动路径
不一定与 MoveIt 已碰撞检查的轨迹完全一致。

### 推荐方案

确认 SDK 的 `CommandJointPosition` 是否支持稳定的定周期下发，并确认其控制模式、
看门狗、超时和限幅语义。bridge 将 MoveIt 的带时间戳轨迹重采样后，按控制周期
发送给 SDK。这种方式最接近 MoveIt 规划的轨迹。

### 初期联调方案

将 MoveIt 的路点交给 `MoveJ(joint_path)`。此方案只能用于低速、空场景、单臂
验证；不能把 MoveIt 的碰撞检查结果视为对 SDK 自行插补轨迹的完整保证。

## 真实机器人模式

真实机器人接入时，必须关闭当前 RViz 演示的虚拟驱动，避免它与真实 bridge 重复
发布 `/joint_states` 或注册同名 action：

```bash
ros2 launch franzi_moveit_config demo.launch.py \
  publish_demo_joint_states:=false
```

之后由 `franzi_hardware_bridge` 独占发布真实状态，并提供轨迹 action。

bridge 应运行在 ROS 使用的系统 Python 环境中。不要通过激活 Conda 来启动 ROS
节点，否则 Conda Python 可能覆盖 ROS 的依赖环境。

## 必须完成的校准与验证

1. 关节映射：逐关节核对 SDK 关节顺序、名称、正负方向和零位偏置，使真实 `q`
   与 RViz 姿态严格一致。
2. 坐标系：标定 `world → robot base`、工具 TCP 与末端负载参数。
3. 限位：用厂家给出的真实速度、加速度和关节限位更新
   `src/franzi_moveit_config/config/joint_limits.yaml`。当前数值仅为保守规划默认值。
4. 单元接口：确认头部、躯干、夹爪的 SDK 控制接口、`side`/机械单元编号、关节
   顺序和可执行的轨迹类型。现有 SDK 对左右臂的七轴支持最明确，其他单元不能
   直接假定兼容同一种接口。
5. 安全链：确认硬件急停、控制柜限位、碰撞检测和故障响应独立有效；MoveIt 不可
   替代硬件安全机制。

## 碰撞检测与仿真范围

- 机器人自碰撞：URDF 已包含 `<collision>` 几何，MoveIt 可执行自碰撞检查。
- 环境碰撞：需将桌面、工装、夹具等加入 Planning Scene，或接入相机/雷达的
  OctoMap。当前配置没有有效的 3D 感知插件，不能自动检测真实环境障碍物。
- 仿真：当前虚拟控制器可在 RViz 中回放 MoveIt 轨迹；它适合 UI、规划和轨迹
  可视化验证，不代表真实控制器的动力学、时延或安全行为。

## 建议实施顺序

1. **Read-only bridge**：只发布真实 `/joint_states`，不提供执行 action；验证真实
   机器人与 RViz 姿态、零位和坐标系完全同步。
2. **低速单臂执行**：实现一个 `FollowJointTrajectory` action，执行时限制到 5%
   速度；在空场景、无人接触且急停有效的条件下验证。
3. **反馈与取消**：接入真实到位判定、起点容差、超时、Cancel、`MoveStop()`、
   急停与故障处理。
4. **双臂与其余单元**：处理双臂同步、躯干、头部和夹爪，并确认每个控制接口。
5. **场景与感知**：添加固定碰撞物、抓取物体附着逻辑和 3D 感知输入。
6. **上线验证**：在厂家给定的限位、控制模式和安全流程下进行完整测试。

## 开始实现 bridge 前需确认

- SDK 是否有正式的定周期轨迹/流式位置命令接口，以及控制周期、超时和看门狗要求。
- `CommandJointPosition` 的控制模式与安全语义。
- 左右臂、躯干、头部和夹爪各自的 SDK 调用方式、关节顺序与单位。
- 双臂是否支持同一条同步轨迹，或需要并行调用两个单臂接口。
- 真实硬件的速度、加速度、关节位置和碰撞阈值。
