# 移动机械臂雕刻机自动上下料 · Isaac Sim 数字孪生

Franzi 人形移动机器人在仿真车间内完成雕刻机自动上下料的**全流程系统**:
SLAM 建图 → Nav2 导航 → AprilTag 毫米级精停靠 → MoveIt 2 抓取 →
入槽上料 → 互锁加工 → 下料 → 成品入格,全程由数据驱动状态机编排,
从墙角待命点出发、也回到墙角收尾。

核心是一套 **Isaac Sim 数字孪生**:控制栈与真机完全同构,Isaac 只扮演
"传感器与世界"——渲染真实相机图像、仿真 RTX 激光雷达,感知从几何 mock
变成**从像素实测**,而上层代码一行不改。

## 演示视频(`IssacSim/renders/`)

| 视频 | 内容 |
|---|---|
| [cell_overview.mp4](IssacSim/renders/cell_overview.mp4) | 上帝视角:车间全景一镜到底,完整任务周期 |
| [robot_cameras.mp4](IssacSim/renders/robot_cameras.mp4) | 机载四相机 2×2 同步拼接(头/胸 D435 + 双腕 D405,与真机部署一致) |
| [rviz_navigation.mp4](IssacSim/renders/rviz_navigation.mp4) | 建图成果 + 代价地图 + 规划路径 + 实走轨迹 |

## 实测指标(Isaac 真视觉链路)

| 指标 | 结果 |
|---|---|
| AprilTag 视觉精停靠 | **0.5–6 mm / ≤0.28°**(指标 8 mm/0.8°) |
| 工件视觉定位(tag→抓取点 vs 勘测) | **4–7 mm** |
| 完整周期 IDLE→DONE | 多次复现通过,约 5.5 min |
| 纯逻辑单元测试 | 39 项(互锁/状态表/停靠闭环/格位簿) |

## 架构

```text
franzi_task_manager      任务层:YAML 状态机(超时/重试/恢复路由即数据)
      │ 8 个 ROS 2 Action
franzi_skills            技能层:导航/精停靠/检测/抓取/上下料/放置/安全姿态
      │ MoveItPy · TF2 · apriltag_ros · Nav2
franzi_pick_place        构件库:底盘驱动、MoveIt 封装、夹爪、场景、tag 管线
franzi_machine_bridge    雕刻机桥:互锁、超时、可换后端 MachineIO(仿真/Modbus)
franzi_engraving_interfaces   全部 msg / srv / action 定义
─────────────────────────────────────────────────────
IssacSim/                Isaac Sim 数字孪生(传感器与世界,见下)
```

每一层迁移真机时只换最底下的实现(逐层接缝表见
[franzi_skills/README.md](Rviz/src/franzi_skills/README.md)),真机导航为
"给坐标"式接口时仅需替换 navigate 技能内十几行的执行器适配。

## Isaac Sim 数字孪生(`IssacSim/`)

本项目的仿真核心。设计原则:**Isaac 不跑任何控制**,它镜像 ROS 栈的
关节与底盘位姿,产出 RViz 给不了的东西——真实渲染与真实传感。

- **镜像执行架构**:订阅 `/joint_states` 与 `/base/pose`(OmniGraph
  通用订阅,Isaac 进程内无系统 rclpy),逐帧写入 articulation;夹持的
  工件在抓取瞬间刚性挂载到手腕物理位姿,与手同帧渲染;
- **真相机**:头/胸 D435、双腕 D405 按 RealSense 话题布局发布 RGB/深度/
  内参,光学帧与 TF 树严格对齐(`link ∘ Rz(−90°)`,启动时打印交叉核对);
- **RTX 激光雷达**:SICK TIM781(2D,喂 SLAM/Nav2)+ 32 线点云,
  wall-clock 时间戳与 ROS 栈同一时间线;
- **真感知闭环**:apriltag_ros 从渲染像素解码 36h11,发布与 mock 同名的
  `tag_N` TF——上层零感知切换,这就是"仿真↔真机"的感知接缝;
- **场景**:按 DMG MORI 现场照片建模的雕刻机(围栏/回转台/虎钳/主轴/
  信号灯塔)、料堆、货架、仓库,布局从 `task.yaml` 单一真源读取,
  与 MoveIt 规划场景永不漂移;
- **上色与展示**:机器人白壳/黑面罩涂装、胸口徽标,外部环绕相机出
  宣传片(`showcase.py`)。

工程上最有价值的部分是**踩坑记录**([IssacSim/README.md](IssacSim/README.md)):
IsaacLab 相机挂 articulation 下渲染不随物理、物理位姿写回 stage 的
节拍、umich 与 OpenCV 的 tag 帧约定差半圈、RTX 雷达嵌套 prim、
sim/wall 时间戳拼接等——每一条都对应一次"检测系统性偏差"的实战排查。

## 快速开始

```bash
# 构建(ROS 2 Jazzy)
cd Rviz && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  franzi_engraving_interfaces franzi_machine_bridge franzi_task_manager \
  franzi_skills franzi_pick_place franzi_description franzi_moveit_config

# 几何级快速验证(RViz 后端,mock 检测)
ros2 launch franzi_skills tending_cell.launch.py

# Isaac 数字孪生(真渲染 + 真检测 + Nav2 导航),两个终端:
IssacSim/run_ros2_cell.sh --cameras head_d435 body_d435 left_wrist_d405 right_wrist_d405
ros2 launch franzi_skills tending_cell.launch.py backend:=isaac nav:=true

# 测试
python3 -m pytest src/franzi_machine_bridge/test src/franzi_task_manager/test src/franzi_skills/test
```

建图、录像、参数与故障排查见
[franzi_skills/README.md](Rviz/src/franzi_skills/README.md) 与
[IssacSim/README.md](IssacSim/README.md)。

## 文档

- [docs/mobile_manipulator_engraving_machine_detailed_plan.md](docs/mobile_manipulator_engraving_machine_detailed_plan.md)
  —— 系统方案(已按实施修订,含实测数据与实施笔记)
- [Rviz/src/franzi_skills/README.md](Rviz/src/franzi_skills/README.md)
  —— 运行手册与真机迁移接缝表
- [IssacSim/README.md](IssacSim/README.md) —— Isaac 侧详解与踩坑记录
- [Rviz/NextStep.md](Rviz/NextStep.md) —— 真机硬件桥方案

## 路线图

- [x] 几何级闭环(RViz/MoveIt)与技能/状态机架构
- [x] Isaac 传感器级:真渲染、真 AprilTag、RTX 雷达
- [x] SLAM 建图 + Nav2 导航 + 视觉精停靠
- [ ] 物理级:接触抓取、ros2_control 执行(Isaac 侧)
- [ ] 真机:硬件桥(franzi_sdk)、Modbus 机器桥、标定与示教
