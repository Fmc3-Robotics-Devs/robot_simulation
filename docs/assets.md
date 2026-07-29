# 资产约定

本机 NVIDIA 资产根目录为：

```text
/home/fmc3/FermiBotNas/SIM_ASSETS/5.1.0/Assets/Isaac/5.1
```

该目录必须包含 `Isaac/` 与 `NVIDIA/`。预检优先发现官方 Warehouse USD，并把它以相对符号链接暴露为 `vendor/isaac_assets`；因此提交的 USD 在任何开发机上都可保持同一相对引用。

当前正式截图使用的 NVIDIA `PackingTable/packing_table.usd` 在独立依赖扫描中会报告 13 个纸箱纹理缺失；Isaac 5.1 RTX 实际渲染仍可正常显示工位，`OmniPBR.mdl` 由 Kit 搜索路径解析。该提示属于本地官方素材包的背景道具依赖，不影响项目自有蓝箱、AprilTag、机器人或主作业台，但不得据此宣称第三方资产依赖“完全无缺失”。若后续这些纸箱成为任务对象，应改用依赖完整的项目包装层或补齐经授权的官方素材后重新采证。

`report/手机放置槽.STL` 是 3 列 × 6 行、18 槽的泡沫箱源模型。它是用户资料，工程代码不得修改它。导入后生成的项目 USD 放入 `usd/assets/foam_box_18_slots.usda`，STL 以毫米输入、USD 以米为单位。

`report/i17_AIR_DUMMY_stls/` 是当前手机 STL 来源，包含机身与镜头等 5 个组件；尺寸、碰撞和装箱姿态均以这些 STL 为准。后续导入的派生 USD 和简化碰撞资产应交付到 `usd/assets/`，原始资料保持只读。

## Hugging Face 外部数据

Hugging Face 候选数据登记在 `docs/references.md`。它们不是当前正式场景的运行依赖，也不会由 `uv sync` 或场景启动脚本自动下载。

- `HF_ASSET_ROOT` 是项目自定义变量，不会被 Hugging Face 自动读取。它用于区分 `${HF_ASSET_ROOT}/cache/` 原始缓存和 `${HF_ASSET_ROOT}/approved/<org>/<repo>/<commit>/` 审核通过的只读快照；本机建议根目录为 `/home/fmc3/FermiBotNas/SIM_ASSETS/huggingface/`。按需把 `HF_HOME` 或 `HF_HUB_CACHE` 指向其中的 cache，不能写入 Isaac 5.1 官方资产目录或 `.venv`。
- 仓库内忽略提交的 `vendor/huggingface/` 只作为指向 approved 快照的相对挂载，不得直接挂载可变化的 raw cache。
- 只允许以固定 `revision` 和明确的 Python `allow_patterns` 或 CLI `--include` 下载所需 README、元数据或小样本；先 dry-run 估算体积，禁止默认拉取 TB 级仓库。
- 下载后记录来源 URL、commit、许可证原文、署名/修改说明、再分发许可、文件清单与 SHA-256。许可证缺失、用途不兼容或需要执行未知自定义代码时，仅可阅读公开数据卡，不下载文件。
- 公共仓库下载设置 `HF_HUB_DISABLE_IMPLICIT_TOKEN=1`；正式场景运行设置 `HF_HUB_OFFLINE=1`。正式运行缺少已审核快照时应直接失败，不得临时联网补文件。
- gated 仓库如确需评估，只使用最小权限 token；token 不写入命令、日志、配置、USD 或证据文件。
- 不使用 `trust_remote_code=True`，不执行仓库脚本，不加载来源不明的 pickle、checkpoint、NumPy object array 或任意 Python 对象。模型如确需评估，优先使用 Safetensors 等可审计格式，并在隔离工作目录中进行。
- 压缩包解包前检查绝对路径、`..`、硬链接和越界符号链接；仓库显示“扫描安全”不能替代本地输入校验。
- 外部 USD/STL 先在隔离 stage 检查单位、坐标系、材质、引用、碰撞和 Isaac Sim 5.1 兼容性；拒绝网络或越出 approved 根目录的 reference/payload、未知 schema、Python Script 和未审核 OmniGraph，并复验 MDL 离线编译、重力/接触/reset 与 RTX。通过后再生成带来源说明的项目包装层。
- HF 中的其他机器人轨迹只迁移任务阶段、数据 schema、相机记录和验收方法；未经关节重定向、限位和物理验证，不能作为 Wheel Bot 控制命令。
