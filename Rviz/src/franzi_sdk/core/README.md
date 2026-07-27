# core 包

SDK 横切能力与机械臂客户端实现根目录。

## 日志命名空间

- 根：`core`
- 子模块：`core.rpc`（RPC 轨迹）、`core.arm`（错误/建连）等

## 默认格式

```
INFO 20260528-19:23:230 具体文本内容
```

`LEVEL` + `YYYYMMDD-HH:MM:SS` + 3 位毫秒（紧跟在秒后，如 `19:23:230` 表示 19:23:23.000）

## 行为

首次 `import core` 时自动配置控制台 Handler，SDK 内部日志直接输出，**demo 无需再配置**。

## 二开承接（可选）

```python
import logging
logging.getLogger("core").addHandler(your_handler)
```

如需自定义格式或落盘，可使用 `core.logging.configure_default_logging`、`set_log_format`、`attach_rotating_file_handler`。

错误码定义见 `errors.py`。ArmClient 与 RPC 域实现见 `client/`，索引见 [docs/sdk_api_catalog.md](../../../docs/sdk_api_catalog.md)。
