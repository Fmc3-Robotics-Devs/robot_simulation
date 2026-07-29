# 资产约定

本机 NVIDIA 资产根目录为：

```text
/home/fmc3/FermiBotNas/SIM_ASSETS/5.1.0/Assets/Isaac/5.1
```

该目录必须包含 `Isaac/` 与 `NVIDIA/`。预检优先发现官方 Warehouse USD，并把它以相对符号链接暴露为 `vendor/isaac_assets`；因此提交的 USD 在任何开发机上都可保持同一相对引用。

当前正式截图使用的 NVIDIA `PackingTable/packing_table.usd` 在独立依赖扫描中会报告 13 个纸箱纹理缺失；Isaac 5.1 RTX 实际渲染仍可正常显示工位，`OmniPBR.mdl` 由 Kit 搜索路径解析。该提示属于本地官方素材包的背景道具依赖，不影响项目自有蓝箱、AprilTag、机器人或主作业台，但不得据此宣称第三方资产依赖“完全无缺失”。若后续这些纸箱成为任务对象，应改用依赖完整的项目包装层或补齐经授权的官方素材后重新采证。

`report/手机放置槽.STL` 是 3 列 × 6 行、18 槽的泡沫箱源模型。它是用户资料，工程代码不得修改它。导入后生成的项目 USD 放入 `usd/assets/foam_box_18_slots.usda`，STL 以毫米输入、USD 以米为单位。

`report/i17_AIR_DUMMY_stls/` 是当前手机 STL 来源，包含机身与镜头等 5 个组件；尺寸、碰撞和装箱姿态均以这些 STL 为准。后续导入的派生 USD 和简化碰撞资产应交付到 `usd/assets/`，原始资料保持只读。
