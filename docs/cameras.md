# 四路相机

Wheel Bot 的原始 `wheel_robot_4.0.urdf`（`origin/franzi`）已经包含 D435/D405 的实体 link 和固定 joint；本工程不复制或第二次建模这些壳体。相机传感器配置集中在 `franzi_sim.cameras`，由 `usd/assets/robots/wheel_bot/camera_sensors.usda` 作为传感器层交付；可直接引用的完整机器人入口是 `usd/assets/robots/wheel_bot/wheel_bot_with_cameras.usda`。

| 逻辑名 | URDF 父 link | 相机 | ROS 图像话题 | ROS CameraInfo |
| --- | --- | --- | --- | --- |
| `head_d435` | `head_d435_Link` | D435，1280×720 | `/franzi/camera/head_d435/color/image_raw` | `/franzi/camera/head_d435/color/camera_info` |
| `chest_d435` | `body_d435_Link` | D435，1280×720 | `/franzi/camera/chest_d435/color/image_raw` | `/franzi/camera/chest_d435/color/camera_info` |
| `left_wrist_d405` | `left_wrist_d405_Link` | D405，1280×720 | `/franzi/camera/left_wrist_d405/color/image_raw` | `/franzi/camera/left_wrist_d405/color/camera_info` |
| `right_wrist_d405` | `right_wrist_d405_Link` | D405，1280×720 | `/franzi/camera/right_wrist_d405/color/image_raw` | `/franzi/camera/right_wrist_d405/color/camera_info` |

所有传感器都相对已有壳体 link 固定在原点，分辨率均为 1280×720，USD `clippingRange` 统一为 **`(0.05, 100) m`**。Isaac USD 相机沿局部 `-Z` 看向前方、局部 `+Y` 为图像上方，而四个 URDF 壳体并不共享同一光学 frame：导入 USD 后，头部和胸部 D435 使用 `(180, 0, -90)`，左右腕 D405 使用 `(-90, 0, 90)`。这两组旋转同时校正光轴和画面滚转，经过世界坐标轴计算与 RTX 实际出图复核；校准场景中的红、绿竖直标杆必须在四路画面中保持竖直。

头部和胸部 D435 直接随原 URDF 的 `head_d435_joint` / `body_d435_joint` 固定位置运动；左右腕 D405 分别随同名 wrist fixed joint 运动，绝不交叉映射。

## 静态检查

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest tests/test_camera_configuration.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest tests/test_optical_frame_tf.py tests/test_apriltag_assets.py
uv run python scripts/verify_camera_configuration.py
uv run python scripts/verify_optical_frame_tf.py
```

## Isaac GUI 画面复核

在正式场景 `usd/scenes/warehouse_box_transfer.usda` 中，Wheel Bot 位于 `/WheelBotBoxTransfer/WheelBot`。启动时间线后，逐一在 Camera 视口选择以下 prim：

`/WheelBotBoxTransfer/WheelBot/head_d435_Link/head_d435_optical_frame/camera`、`/WheelBotBoxTransfer/WheelBot/body_d435_Link/chest_d435_optical_frame/camera`、`/WheelBotBoxTransfer/WheelBot/left_wrist_d405_Link/left_wrist_d405_optical_frame/camera`、`/WheelBotBoxTransfer/WheelBot/right_wrist_d405_Link/right_wrist_d405_optical_frame/camera`。

每路保存一张 viewport 截图，并确认头/胸画面面向前方，左右腕视角的左右标记、箱体区域与逻辑名称对应。2026-07-29 的正式工位四路 RTX 截图与总览已归档到 [`evidence/workcell/2026-07-29/final/`](../evidence/workcell/2026-07-29/final/)：`head_d435` 在正式图像中解码了贴于蓝箱正面的 Tag 0；胸部与双腕当前视角有效但尚未覆盖标签，后续在抓取姿态中验收“对应侧夹爪进入画面”和 Tag 检测覆盖率。这是静态检查无法替代的真实渲染验收。

当前 `Image` / `CameraInfo` 的 header 使用 `*_optical_frame`。`franzi_sim.optical_frame_tf` 现在为每个 URDF 相机壳体 link 到该 optical frame 提供一个固定边：头/胸 D435 为 `(180, 0, -90)`，双腕 D405 为 `(-90, 0, 90)`（均为 XYZ Euler 度）。这些数值与 `camera_sensors.usda` 的 optical-frame transform 完全相同；运动链其余部分仍应由 URDF/robot-state publisher 发布。

在启用 ROS 2 bridge 后，以已有 `rclpy` node 调用：

```python
from franzi_sim.optical_frame_tf import publish_optical_frame_static_transforms

publish_optical_frame_static_transforms(node)
```

它使用 ROS 2 的 `tf2_ros.StaticTransformBroadcaster` 在 `/tf_static` 发布四条可持久化边。离线检查不能代替真实 DDS 验证：在 Isaac/ROS 环境中应运行 `ros2 run tf2_ros tf2_echo head_d435_Link head_d435_optical_frame`（其余三路同理），并确认 `/tf_static` 中每个 child frame 只有一个发布者。

启用 `isaacsim.ros2.bridge` 后，调用 `franzi_sim.ros2_camera_bridge.add_ros2_camera_publishers()` 会为每路相机创建独立的 Render Product、`ROS2CameraHelper` 和 `ROS2CameraInfoHelper`。因此图像与内参均来自同一个 Camera prim，不维护第二套手写内参。

## ROS 2 运行验证

在项目 Isaac Sim 已启动正式场景 ROS 2 Bridge 时，于系统 Jazzy 终端运行以下命令，以真实 DDS 消息生成验证证据：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 scripts/verify_ros2_camera_topics.py \
  --timeout 45 \
  --output evidence/cameras/YYYY-MM-DD/formal_ros2/ros2_topics.json
```

验证器不会分别接受每个 topic 的首包。每路 `Image` 和 `CameraInfo` 只有在 `header.stamp.sec` 与 `header.stamp.nanosec` 都相同后才会计入 `topics`，并在 `matched_pairs` 写入相机名、两个 topic 与该精确时间戳。`passed: true` 还要求四路都已配对、所有字段契约有效；没有匹配的端点会保留在 `missing_topics`，而已到达但未配对的端点另列在 `unmatched_topics`。该严格验证器已实现，但最新正式场景的 live ROS 2、TF 和 Tag 位姿闭环尚未重采；历史证据只能说明当时采集到的消息，不能因验证器改动而回填或宣称已经通过这一同步门。
