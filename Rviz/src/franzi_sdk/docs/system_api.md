# Python SDK — 系统层接口文档

本文档仅描述 **SystemModule** 对外暴露的系统层能力，以及与之配套的就绪检查、软急停接口。

- Python 实现：`core/client/system.py`（SystemModule 路径）、`core/client/core.py`（`IsReady` / `SoftStop`）
- 服务端定义：`sdk_service_system.h`
- 客户端入口：`ArmClient`

运动模式、关节状态读取等接口不在本文档范围内。

---

## 1. 接口一览

| Python 方法 | RPC | 说明 |
|-------------|-----|------|
| `SetMechUnitLifecycle` | `arm_sdk.SetMechUnitLifecycle` | 上/下使能、复位 |
| `GetMechUnitState` | `arm_sdk.GetMechUnitState` | 查询机械单元状态 |
| `GetMechUnitErrorInfo` | `arm_sdk.GetMechUnitErrorInfo` | 查询机械单元故障摘要 |
| `GetEmergencyStopState` | `arm_sdk.GetEmergencyStopState` | 查询整机急停快照 |
| `IsEmergencyStopActive` | `arm_sdk.GetEmergencyStopState` | 急停是否有效（封装） |
| `IsReady` | `arm_sdk.IsReady` | 执行层是否就绪 |
| `SoftStop` | `arm_sdk.SoftStop` | 触发/解除 SDK 软急停 |

调用链：

```
ArmClient  →  SdkServerModule  →  SystemModule INO
                                  （lifecycle / state / error / estop）
```

---

## 2. 快速开始

```python
import sys, os
sys.path.insert(0, os.path.abspath("sdk/sdk_py"))

from core.client import ArmClient
from core.types import (
    MechUnitType,
    MechUnitLifecycleCmd,
    SdkStatus,
    sdk_status_message,
    mech_unit_state_name,
)

arm = ArmClient(host="192.168.23.30", port=8000, raise_on_error=False)

try:
    if not arm.IsReady():
        print("执行层未就绪")
    if arm.IsEmergencyStopActive():
        print("急停有效")
    else:
        ret = arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.ENABLE)
        print(sdk_status_message(ret))
finally:
    arm.disconnect()
```

---

## 3. 类型定义

### MechUnitType — 机械单元

| 枚举 | 值 |
|------|----|
| `LEFTARM` | 0 |
| `RIGHTARM` | 10 |
| `WAIST` | 20 |
| `HEAD` | 30 |
| `LEFTHAND` | 40 |
| `RIGHTHAND` | 50 |

### MechUnitLifecycleCmd — 生命周期命令

| 枚举 | 值 | 说明 |
|------|----|------|
| `ENABLE` | 1 | 上使能 |
| `DISABLE` | 2 | 下使能 |
| `RESET` | 3 | 复位 |

### MechUnitStateType — 机械单元状态

| 枚举 | 值 | 说明 |
|------|----|------|
| `FAULT` | 0 | 故障 |
| `RESETTING` | 10 | 复位中 |
| `DISABLED` | 20 | 未使能 |
| `DISABLING` | 25 | 下使能中 |
| `ENABLING` | 30 | 上使能中 |
| `ENABLED` | 40 | 已使能 |
| `RUNNING` | 50 | 运动中 |

辅助函数：`mech_unit_state_name(value) -> str`、`format_mechunit_error_code(value) -> str`（如 `0x00001234`）

### SdkStatus — 常用返回码

| 值 | 说明 |
|----|------|
| `0` | 成功 |
| `-5` | 执行层/INO 未就绪 |
| `-8` | 急停有效，禁止上使能 |

辅助函数：`sdk_status_message(status) -> str`

---

## 4. 接口说明

### 4.1 SetMechUnitLifecycle

经 SystemModule 下发机械单元生命周期命令。

```python
ret = arm.SetMechUnitLifecycle(component_type, lifecycle_cmd)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `component_type` | `int` / `MechUnitType` | 目标机械单元 |
| `lifecycle_cmd` | `int` / `MechUnitLifecycleCmd` | Enable / Disable / Reset |

| 返回 | 说明 |
|------|------|
| `int` | `SdkStatus`，`0` 表示成功 |

请求体：`{"component_type": int, "lifecycle_cmd": int}`

---

### 4.2 GetMechUnitState

查询机械单元当前聚合状态。

```python
status, unit_state = arm.GetMechUnitState(component_type)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `component_type` | `int` / `MechUnitType` | 目标机械单元 |

| 返回 | 说明 |
|------|------|
| `status` | `SdkStatus` |
| `unit_state` | `MechUnitStateType` 整型值 |

请求体：`{"component_type": int}`

---

### 4.3 GetMechUnitErrorInfo

查询机械单元故障摘要。

```python
info = arm.GetMechUnitErrorInfo(component_type)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `component_type` | `int` / `MechUnitType` | 目标机械单元 |

| 返回字段 | 类型 | 说明 |
|----------|------|------|
| `status` | int | 返回码 |
| `mech_state` | int | 机械单元状态 |
| `unit_has_fault` | int | 是否有故障 |
| `fault_joint_count` | int | 故障关节数 |
| `primary_error_code` | int | 主故障码 |
| `primary_joint_index` | int | 主故障关节 |
| `joints` | list | 关节故障详情 |

`joints[]` 每项含：`joint_index`、`error_code`（uint32，可用 `format_mechunit_error_code()` 格式化为 `0x........`）、`has_fault`、`servo_state`、`follow_error_active`

---

### 4.4 GetEmergencyStopState / IsEmergencyStopActive

查询 SystemModule 聚合的整机急停状态。

```python
rsp = arm.GetEmergencyStopState()
active = arm.IsEmergencyStopActive()
```

| 返回字段 | 类型 | 说明 |
|----------|------|------|
| `status` | int | 返回码 |
| `estop_active` | bool | `True` 表示急停有效 |
| `estop_reg` | int | 急停寄存器快照 |
| `timestamp_us` | int | 时间戳（微秒） |

`IsEmergencyStopActive()` 返回 `bool`；RPC 失败时返回 `False`。

**独立测试程序**：`demo/query_estop.py`（见第 6.1 节）。

---

### 4.5 IsReady

查询 SdkServer 执行层是否就绪。软急停按下时返回 `False`。

```python
ready = arm.IsReady()  # bool
```

---

### 4.6 SoftStop

触发或解除 SDK 软急停。

```python
arm.SoftStop(active=True)   # 按下
arm.SoftStop(active=False)  # 弹起
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `active` | `bool` | `True` 按下，`False` 弹起 |

| 返回 | 说明 |
|------|------|
| `bool` | 操作是否成功 |

与 `GetEmergencyStopState` 的区别：`SoftStop` 是主动触发；`GetEmergencyStopState` 是读取当前急停快照。

---

## 5. 典型流程

### 上使能

```python
assert arm.IsReady()
if arm.IsEmergencyStopActive():
    raise RuntimeError("急停有效，禁止上使能")
# 也可先运行: python demo/query_estop.py

ret = arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.ENABLE)
if ret != SdkStatus.OK:
    raise RuntimeError(sdk_status_message(ret))

_, state = arm.GetMechUnitState(MechUnitType.LEFTARM)
```

### 故障复位后再使能

```python
info = arm.GetMechUnitErrorInfo(MechUnitType.LEFTARM)
if info.get("unit_has_fault"):
    arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.RESET)
arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.ENABLE)
```

### 下使能

```python
arm.SetMechUnitLifecycle(MechUnitType.LEFTARM, MechUnitLifecycleCmd.DISABLE)
```

---

## 6. 示例与测试程序

以下 demo 均只依赖 `core`，不 import `demo` 包内其它文件。运行前进入 SDK 根目录：

```bash
cd sdk/sdk_py
```

### 6.1 `query_estop.py` — 急停状态查询

专门调用 `GetEmergencyStopState` / `IsEmergencyStopActive`，打印急停快照并给出简要说明。

| 项目 | 说明 |
|------|------|
| 路径 | `demo/query_estop.py` |
| 接口 | `GetEmergencyStopState`、`IsEmergencyStopActive` |
| 依赖 | 仅 `core` |

```bash
# 查询一次
python demo/query_estop.py

# 指定控制器地址
python demo/query_estop.py --host 192.168.23.30 --port 8000

# 每 2 秒轮询（Ctrl+C 退出）
python demo/query_estop.py --watch 2
```

**典型输出**（急停未激活）：

```text
estop_active=False reg=0x0000 timestamp_us=... IsEmergencyStopActive=False
急停未激活 — 可尝试上使能（仍需满足其它安全条件）
```

**典型输出**（急停有效）：

```text
estop_active=True reg=0x.... timestamp_us=... IsEmergencyStopActive=True
急停有效 — 禁止上使能，需复位后再 Enable
```

---

### 6.2 `system_test.py` — 系统层联调测试

双臂场景下的系统层端到端联调：查询状态 → 上使能 → 简单运动 → 下使能。

| 项目 | 说明 |
|------|------|
| 路径 | `demo/system_test.py` |
| 系统层接口 | `GetMechUnitState`、`SetMechUnitLifecycle`（Enable / Disable） |
| 联调运动 | `MoveJ`（验证使能后 `RUNNING` 等状态，非本文档核心接口） |
| 依赖 | 仅 `core` |

**流程**：

1. 上使能前打印左/右臂 `GetMechUnitState`
2. 对左、右臂依次 `SetMechUnitLifecycle(ENABLE)`
3. 上使能后再次打印状态
4. 循环 5 次 `MoveJ` 往返（左臂 `side=0`，右臂 `side=1`）
5. 对左、右臂 `SetMechUnitLifecycle(DISABLE)`

```bash
python demo/system_test.py
```

运行前请确认：控制器已启动、`IsReady()` 为真、急停未激活（可用 `query_estop.py` 检查）、当前拓扑包含双臂。

> 单臂拓扑（如 `single_arm_hard_left`）请改用 `enable.py` / `disable.py`，或修改 `system_test.py` 中的 `MechUnitType` 与 `MoveJ` 侧别。

---

### 6.3 `test_movl.py` — 笛卡尔直线 MoveL 联调

双臂场景下先关节运动到预备姿态，再循环执行笛卡尔直线 `MoveL`，并在运动前后打印整机快照（含 `IsReady`、使能、关节/笛卡尔状态），用于验证 MOVL 链路与运动前后状态一致性。

| 项目 | 说明 |
|------|------|
| 路径 | `demo/test_movl.py` |
| 运动接口 | `MoveJ`、`MoveL`（`core/client/motion_traj.py`） |
| 状态/就绪 | `IsReady`、`GetEnableState`、`ReadCartesianPose` 等（脚本内 `log_snapshot`） |
| 场景 | 双臂：MoveJ 预备 → 快照 → 循环 MoveL |
| 依赖 | `core`（`log_snapshot` 定义在脚本内，不依赖其它 demo 文件） |

**流程**：

1. 连接控制器，等待 3 秒
2. 左、右臂各执行一次 `MoveJ`，目标关节 `[0, 0, 0, 0.4, 0, 0, 0]` rad（`side=0` 左臂，`side=1` 右臂）
3. 调用 `log_snapshot` 打印左臂快照（含 `IsReady`、使能、关节角、笛卡尔位姿等）
4. 进入 `while True` 循环：左/右臂依次 `MoveL`，每轮间隔 2 秒；**Ctrl+C 退出**
5. `finally` 中 `disconnect()`

**路点格式**（脚本内 `left_cartesian_path` / `right_cartesian_path`）：

- 每点为 6 维 `[x, y, z, rx, ry, rz]`，单位 m + rad（XYZ 欧拉角）
- **不含起点**；起点由控制器当前关节经 FK 得到
- 脚本中路点基于关节角 `[0, 0, 0, 0.4, 0, 0, 0]` rad 标定；若预备姿态不同，需按 FK 重算路点

**运行**：

```bash
python demo/test_movl.py
```

脚本顶部可修改 `HOST`、`PORT`（默认 `192.168.23.30:8000`）。

**运行前请确认**：

| 条件 | 说明 |
|------|------|
| 控制器已启动 | `arm.IsReady()` 为 `True` |
| 急停未激活 | 可用 `query_estop.py` 检查 |
| 机械臂已使能 | 脚本内 `SetEnableState` 默认注释；需先运行 `enable.py`，或取消注释相关行 |
| 拓扑为双臂 | 单臂请只保留对应 `side` 的 `MoveJ` / `MoveL`，并修改路点 |
| SDK / 服务端版本 | RPC 名为 `arm_sdk.MoveL`；旧版服务端仅注册 `Movl` 时会报 `unknown function: 495760112`，需升级 `sdk_server_module` 或使用带 `MoveL`/`Movl` 回退的 `motion_traj.py` |

**`side` 参数约定**（与 `MoveJ` 相同）：

| `side` | 含义 |
|--------|------|
| `0` | 左臂 |
| `1` | 右臂 |

> 注意：此处使用整型 `0`/`1`，与 C++ `SdkArmSide`（Left=0, Right=1）一致；**不要**使用 `ArmSide.LEFT`（Python 枚举值为 `1`，会映射到右臂）。

**典型日志片段**（MoveJ 后快照）：

```text
[SDK] version=1.0.0 IsReady=True AxisCount=14
[LEFT 臂]
  使能: True
  当前关节角(rad): [0.0000, 0.0000, 0.0000, 0.4000, 0.0000, 0.0000, 0.0000]
  笛卡尔位姿: [...]
```

**常见问题**：

| 现象 | 可能原因 |
|------|----------|
| `unknown function: 495760112` | Python 调用 `MoveL`，服务端未注册同名 RPC（见上表） |
| 快照中 `使能: False` | 未上使能，先运行 `enable.py` 或取消脚本内 `SetEnableState` 注释 |
| `MoveL` 返回非 0 | IK 失败、路点不可达、或未在 `RUNNING` 状态；查看控制器日志 |
| 循环不退出 | 脚本设计为 `while True`，需 **Ctrl+C** 中断 |

---

### 6.4 其它示例

| 场景 | 文件 | 主要接口 |
|------|------|----------|
| 上使能 | `demo/enable.py` | `SetMechUnitLifecycle(ENABLE)`、`GetMechUnitState` |
| 下使能 | `demo/disable.py` | `SetMechUnitLifecycle(DISABLE)`、`GetMechUnitState` |
| 故障复位 | `demo/reset.py` | `SetMechUnitLifecycle(RESET)`、`GetMechUnitState` |
| 触发软急停 | `demo/soft_stop.py` | `SoftStop`、`IsReady` |
| MechUnit 状态巡检 | `demo/lookforstate.py` | `GetMechUnitState`、`GetMechUnitErrorInfo` |
| 笛卡尔 MoveL 联调 | `demo/test_movl.py` | `MoveJ`、`MoveL`、`IsReady`（见 6.3 节） |

---

*服务端 C++ 定义：`src/module/sdk_server_module/include/sdk_server_module/service/sdk_service_system.h`*
