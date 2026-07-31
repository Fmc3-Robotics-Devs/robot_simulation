# Isaac Sim 阶段

Isaac 在这套系统里的角色:**传感器与世界**。控制栈(MoveIt、技能、任务
状态机)与 RViz 阶段完全同一份,Isaac 通过镜像跟随它,并产出 RViz 给不了
的东西 —— 真渲染的相机图像与雷达点云,让感知从"几何 mock"变成"实际测量"。

```text
ROS 栈(不变)                     Isaac(本目录)
/joint_states  /base/pose  ──────▶  镜像:写关节 + 底盘根位姿
                                    ├─ /head_d435/color/…(RGB/深度/内参)
apriltag_ros  ◀──────────────────── │   frame: head_d435_optical
/scan /mid360/points ◀───────────── └─ RTX 雷达 ×2
```

## 运行

```bash
./run_ros2_cell.sh                    # 镜像模式(默认,配合 tending launch)
./run_ros2_cell.sh --standalone --gui # 自发布模式,单独看场景
./run_ros2_cell.sh --check           # 起来自检几秒后退出
```

配套的 ROS 侧启动见 `Rviz/src/franzi_skills/README.md`(`backend:=isaac`)。
布局单一真源是 `Rviz/src/franzi_pick_place/config/task.yaml`,`cell.py`
直接读它,两个仿真器不会漂移。

## 场景

`ros2_cell.py --scene {warehouse,center,none}`:默认 warehouse(原 cell:
手搭雕刻机 + 货架 + 料堆);`center` 加载客户 STEP 转换的智能制造中心
(`usd/machining_center.usd`,scene/智能制造中心.stp 经 Isaac 的 HOOPS
CAD converter 转换,毫米单位)。center 场景里车间本身就是世界 —— 只放
任务必需的三张桌台和 tag,machine 工位也用桌台(CAD 的 DMG 在走廊对面
作为真实背景),不 spawn 手搭机床/货架/料堆。变换:绕 z 转 90° +
平移 (6.3, -20, ground_z),使车间主走廊沿三工位一列铺开;ROS 侧配套
`scene:=center`(地图与待命点自动切换,见 franzi_skills/README.md)。

## 文件

- `ros2_cell.py` — 主入口:建 cell、镜像、发布相机与雷达。
- `cell.py` — 从 task.yaml 读布局。
- `apriltags.py` — 生成/解码 36h11 marker(OpenCV,黑边外缘即 tag size)。
- `render_cell.py` / `render_cameras.py` — 离线渲染与自检管线。
- `convert_urdf.py` — URDF → USD。
- `paint.py` — 机器人上色(URDF 材质全白):白壳/黑面罩/深灰关节,胸口
  FMC3-Robotics 徽标贴片(位置由 torso 包围盒推出,跟随 link)。
  ros2_cell / render_cell / showcase 建场景时统一调用。
- `scenery.py` — 三个入口共用的静态场景:室内灯光(穹顶+吊灯+机床内灯)、
  深色大台面、feeder 料堆(平放铝块阵,tag 走廊留空,`/World/workpiece` 仍是
  任务镜像的那一块,路径与 translate op 不能动)、四组货架(摆位避开现有
  直线行驶走廊,是将来 Nav2 避障测试的障碍物)。机器人的 home 位 = odom
  原点,各脚本 `--dock home` 停在那里。
- `machine.py` — machine 工位的雕刻机(按 `scene/` 里的 DMG MORI 现场照片建):
  围栏+门洞、回转台/托盘/卡盘、虎钳工装、主轴、信号灯塔。托盘顶面就是
  task.yaml 的台面平面,虎钳复刻 MoveIt 的 pocket 尺寸,门朝停靠侧,
  围栏避开手臂与搬运走廊 —— 控制栈与检测完全无感。围栏只在 Isaac 里,
  MoveIt 碰撞世界仍是 bench+pocket,改尺寸时别伸进手臂工作区。
- `showcase.py` — 外部相机给机器人拍照与环绕视频帧:
  `conda run -n env_isaaclab python -u IssacSim/showcase.py --test`(只出照片),
  去掉 `--test` 再出 240 帧环绕序列,用 ffmpeg 合成 H.264。

## 踩过的坑(修复都在代码里,这里记为什么)

1. **IsaacLab `Camera` 传感器挂在 articulation link 下,渲染位姿不跟随
   物理姿态**(用的是 authored/导入姿态)。所有检测系统性偏 ~20°/0.3 m,
   而且 TF 与物理怎么对都对不上 —— 因为渲染那一环压根不在这两者里。
   解法:相机用裸 USD prim(随 Fabric 走),绕 x 转 180° 使视轴对齐
   link +z;光学帧与 link 帧重合,ROS 侧静态 TF 是恒等。
2. **umich apriltag(apriltag_ros)与 OpenCV ArUco 对图案帧的定义差半圈**。
   场景里 tag 贴图按勘测约定旋转 180° 安装(`spawn_textured_quad` 的
   `yaw_degrees`),对应真机上"tag 板按标定方向装"这件事(方案 §7.9)。
3. **RTX 雷达**:`IsaacSensorCreateRtxLidar` 返回的是嵌套在资产里的
   OmniLidar prim,render product 必须指向它,指外层 Xform 会报
   "Render product not attached to RTX Lidar"。config 名大小写取 USD
   资产拼写(`SICK_TIM781`),JSON 文件的拼写无效。
4. **时间戳**:相机/雷达 helper 全部 `useSystemTime`,否则 sim 时间戳的
   检测和 wall 时钟的 TF 树拼不到一条时间线上。
5. **底盘位姿的镜像通道**:桥没有 TF 订阅节点,`MobileBase` 额外发布
   `geometry_msgs/PoseStamped` 到 `base/pose`,Isaac 用通用
   `ROS2Subscriber` 读(属性按消息字段展开,如 `outputs:pose:position:x`)。
6. Isaac 进程只带 ROS 的 C 库(见 `run_ros2_cell.sh`),不能 import 系统
   rclpy —— 所有订阅走 OmniGraph 节点。

## 实测(2026-07,RTX 渲染 + apriltag_ros + 完整任务栈)

- 三站真视觉停靠误差 ≤ 3.2 mm / 0.11°(方案指标 8 mm / 0.8°);
- tag 推算的部件位姿与勘测差 7–8 mm;
- 完整雕刻周期 IDLE → DONE 通过。

## 下一步(物理阶段)

镜像换成执行:给 articulation 配真实刚度,Isaac 侧实现
FollowJointTrajectory 控制器与 cmd_vel 差速/全向底盘,MoveIt 的控制器
配置指过去;工件加物理属性,抓取从 attach 变为接触。控制栈仍然不动。
