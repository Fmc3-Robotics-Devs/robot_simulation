# 开发约定

- 每个公开类、函数、枚举和 dataclass 使用一句简短 docstring 描述责任。
- 对状态转换、单位换算、持久化和安全退出等非显然逻辑，注释解释原因而不是复述代码。
- 场景配置使用不可变 dataclass；运行状态只保存在状态机实例中，以便无 Isaac 的单元测试和批量复位。
- USD 采用组合层，不在仓库复制 NVIDIA Warehouse。项目资产放在 `usd/assets/`，场景入口放在 `usd/scenes/`。
- 新的 NVIDIA API、资产和社区参考需登记版本、链接、许可证和采用范围。
# Control contract smoke checks

`source/franzi_sim/franzi_sim/control/joint_map.json` is the single machine-readable
URDF-to-USD-to-ROS/MoveIt/SDK name contract.  It includes all 30 movable joints:
the 24 joints with current MoveIt controllers and six intentionally SDK-only base
joints.  Validate source names during ordinary development with:

```bash
python3 scripts/verify_joint_mapping.py --skip-usd
python3 scripts/control_smoke.py --print-trajectory
```

In an Isaac Sim or USD (`pxr`) Python environment, omit `--skip-usd`; the verifier
then opens the binary USD and requires every mapped joint prim.  The head-yaw smoke
trajectory is deliberately a 0.05 rad, one-second non-GUI command payload.  Runtime
execution remains gated on an articulation adapter binding its DOFs through this map.
